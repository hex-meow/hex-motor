//! 常驻 task：监听 TPDO1（0x180..=0x1FF）和 TPDO2（0x280..=0x2FF）帧。
//!
//! 每帧做三件事：
//!
//! 1. **liveness**：盖 `last_tpdo` 时间戳；offline → online 时发
//!    [`Cia402Event::NodeOnline`]。
//! 2. **decode**（M4+）：用 [`super::codec::decode_tpdo1`] /
//!    [`super::codec::decode_tpdo2`] 把字节翻译成 `Measurements` 字段；TPDO2
//!    顺带用 `status_word_to_logic` 算出 [`Logic`]。
//! 3. **error edge detection**：`logic` 从 非-Error 跳到 Error 时发一次
//!    [`Cia402Event::EnteredError`]。
//!
//! 设计取舍：原来设计是 per-motor runner，最终决定**全局监听器一把做完**。
//! 节省 N×2 路 subscription + 一个 task per motor，又方便单点维护
//! `last_tpdo`/`logic`/`measurements` 三者的一致性。详见 `DESIGN.md` §5。

use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFilter, CanFrame, CanId, CanIoError};
use tokio::sync::broadcast;
use tokio_util::sync::CancellationToken;

use super::codec::{
    decode_tpdo1, decode_tpdo2, status_word_to_logic, Tpdo1Frame, Tpdo2Frame,
};
use super::events::Cia402Event;
use super::motor_entry::{MotorEntry, MotorEntryInner};
use super::types::Logic;

/// function code 掩码：取出 11-bit ID 的高 4 位，匹配 0x180/0x280/0x380/...
const TPDO_FC_MASK: u16 = 0x780;
const TPDO1_BASE: u16 = 0x180;
const TPDO2_BASE: u16 = 0x280;

/// 哪条 TPDO 被收到了 —— 决定按 12B 还是 10B 解码。
#[derive(Copy, Clone)]
enum TpdoKind {
    Tpdo1,
    Tpdo2,
}

impl TpdoKind {
    fn tag(self) -> &'static str {
        match self {
            TpdoKind::Tpdo1 => "TPDO1",
            TpdoKind::Tpdo2 => "TPDO2",
        }
    }
}

pub(crate) async fn run_tpdo_listener(
    bus: Arc<dyn CanBus>,
    motors: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events_tx: broadcast::Sender<Cia402Event>,
    velocity_window: Duration,
    cancel: CancellationToken,
) {
    let mut rx1 = match bus
        .subscribe(CanFilter::standard(TPDO1_BASE, TPDO_FC_MASK))
        .await
    {
        Ok(r) => r,
        Err(e) => {
            log::error!("TPDO listener: subscribe TPDO1 failed: {e}");
            return;
        }
    };
    let mut rx2 = match bus
        .subscribe(CanFilter::standard(TPDO2_BASE, TPDO_FC_MASK))
        .await
    {
        Ok(r) => r,
        Err(e) => {
            log::error!("TPDO listener: subscribe TPDO2 failed: {e}");
            return;
        }
    };

    loop {
        tokio::select! {
            _ = cancel.cancelled() => {
                log::debug!("TPDO listener cancelled");
                return;
            }
            res = rx1.recv() => handle_frame(res, TpdoKind::Tpdo1, &motors, &events_tx, velocity_window),
            res = rx2.recv() => handle_frame(res, TpdoKind::Tpdo2, &motors, &events_tx, velocity_window),
        }
    }
}

fn handle_frame(
    res: Result<CanFrame, CanIoError>,
    kind: TpdoKind,
    motors: &Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events_tx: &broadcast::Sender<Cia402Event>,
    velocity_window: Duration,
) {
    let frame = match res {
        Ok(f) => f,
        Err(e) => {
            log::warn!("TPDO listener ({}) rx error: {e}", kind.tag());
            return;
        }
    };
    let CanId::Standard(cob_id) = frame.id() else {
        return;
    };
    let nid = (cob_id & 0x7F) as u8;
    if nid == 0 {
        return;
    }

    let entry = motors.read().unwrap().get(&nid).cloned();
    let Some(entry) = entry else {
        // TPDO 来自我们没记录过的 nid。通常不会发生 —— HB 会先到。
        // 不主动建条目（避免假数据污染列表）。
        return;
    };

    let now = Instant::now();

    // ===== 解码 + 更新 measurements / logic =====
    //
    // 解码失败 (长度不够等) 不影响 liveness：仍然走 last_tpdo / online 更新。

    let (live_state, error_event, online_event) = {
        let mut inner = entry.inner.lock().unwrap();
        inner.last_tpdo = Some(now);
        let became_online = !inner.online;
        inner.online = true;

        let mut error_event: Option<Cia402Event> = None;
        match kind {
            TpdoKind::Tpdo1 => {
                if let Some(f) = decode_tpdo1(frame.data()) {
                    apply_tpdo1(&mut inner, f, velocity_window);
                } else {
                    log::warn!(
                        "TPDO1 from nid 0x{nid:02X}: bad length {} (want >=12)",
                        frame.data().len()
                    );
                }
            }
            TpdoKind::Tpdo2 => {
                if let Some(f) = decode_tpdo2(frame.data()) {
                    apply_tpdo2(&mut inner.measurements, f);
                    // 重新计算 logic + 边沿事件
                    let new_logic = status_word_to_logic(
                        f.status_word,
                        inner.target_mode,
                        f.error_code,
                    );
                    let was_error = matches!(inner.logic, Some(Logic::Error { .. }));
                    let is_error = matches!(new_logic, Logic::Error { .. });
                    inner.logic = Some(new_logic.clone());
                    if !was_error && is_error {
                        if let Logic::Error { kind, raw_code } = new_logic {
                            error_event = Some(Cia402Event::EnteredError {
                                nid,
                                kind,
                                raw: raw_code,
                            });
                        }
                    }
                } else {
                    log::warn!(
                        "TPDO2 from nid 0x{nid:02X}: bad length {} (want >=10)",
                        frame.data().len()
                    );
                }
            }
        }

        let online_event = became_online.then_some(Cia402Event::NodeOnline { nid });
        // 把当前 inner 拍成 LiveState 给 publish 用 —— 锁还在；publish 在锁外做。
        (inner.build_live_state(now), error_event, online_event)
    };
    // ↑ 锁已 drop，可以发事件 / publish 了（都不能在持锁中做）

    if let Some(ev) = online_event {
        let _ = events_tx.send(ev);
    }
    if let Some(ev) = error_event {
        let _ = events_tx.send(ev);
    }
    // 把最新 measurements / logic / connection 推给 status() / subscribe_status()。
    entry.publish(live_state);
}

fn apply_tpdo1(inner: &mut MotorEntryInner, f: Tpdo1Frame, velocity_window: Duration) {
    // 先取出标量再借 measurements，避免同时可变 + 不可变借 inner。
    let peak = inner.peak_torque_nm;
    // 滤波速度：用电机时间戳对单圈位置做解卷绕 + 滑动窗口最小二乘。
    let velocity = inner
        .vel_filter
        .update(f.timestamp_us, f.position_rev, velocity_window);

    let m = &mut inner.measurements;
    m.position_rev = Some(f.position_rev);
    m.timestamp_us = Some(f.timestamp_us);
    // torque: i16 ‰ of peak → Nm；没缓存 peak_torque 时留空。
    m.torque_nm = peak.map(|p| f.torque_permille as f32 / 1000.0 * p);
    // 样本不足 / 刚重置时 velocity 为 None，保留上一帧的值不动。
    if let Some(v) = velocity {
        m.velocity_rev_per_s = Some(v);
    }
    // error_code 已经在 TPDO2 路径里处理；TPDO1 的 err 字段作为冗余日志触发器
    if f.error_code != 0 {
        log::trace!("TPDO1 error_code = 0x{:04X}", f.error_code);
    }
}

fn apply_tpdo2(m: &mut super::types::Measurements, f: Tpdo2Frame) {
    m.status_word = Some(f.status_word);
    m.driver_temp_c = Some(f.driver_temp_c());
    m.motor_temp_c = Some(f.motor_temp_c());
}
