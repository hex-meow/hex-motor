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

use crate::meow_motor::model_for_identity;

use super::codec::{decode_tpdo1, decode_tpdo2, status_word_to_logic, Tpdo1Frame, Tpdo2Frame};
use super::events::Cia402Event;
use super::motor_entry::{MotorEntry, MotorEntryInner};
use super::types::Logic;

/// function code 掩码：取出 11-bit ID 的高 4 位，匹配 0x180/0x280/0x380/...
const TPDO_FC_MASK: u16 = 0x780;
const TPDO1_BASE: u16 = 0x180;
const TPDO2_BASE: u16 = 0x280;

/// "这个节点的 TPDO 解不出来" 的告警限流。
///
/// 一条总线上可以合法地跑非 CiA402 电机的 CANopen 设备(例如与底盘共线的 lift driver,
/// 它的 TPDO1 是 8 字节的 位置/时间戳/模式/状态)。它们会以设备自己的 TPDO 速率
/// (lift 是 100Hz)持续触发 "bad length",把日志刷爆,而下游还会把 WARN 转发给运维。
///
/// **不能靠 vendor id 猜"这是不是电机"来跳过解码** —— 2026-08-09 就是这么错过一次:
/// 车队里的 CiA402 电机是第三方 vendor `0x4859444C`,被误判成非电机后 TPDO 不再解码,
/// status word 与 logic 永远是 None,机械臂直接起不来。设备类别不可靠,**解码照旧**,
/// 只把重复告警压掉:每个 (nid, tpdo) 首次告警后静默一段时间。
#[derive(Default)]
struct UndecodableWarnLimiter {
    last: HashMap<(u8, u8), Instant>,
}

impl UndecodableWarnLimiter {
    const QUIET: Duration = Duration::from_secs(60);

    fn should_warn(&mut self, nid: u8, tpdo: u8) -> bool {
        let now = Instant::now();
        match self.last.get(&(nid, tpdo)) {
            Some(prev) if now.duration_since(*prev) < Self::QUIET => false,
            _ => {
                self.last.insert((nid, tpdo), now);
                true
            }
        }
    }
}

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

    let mut undecodable = UndecodableWarnLimiter::default();

    loop {
        tokio::select! {
            _ = cancel.cancelled() => {
                log::debug!("TPDO listener cancelled");
                return;
            }
            res = rx1.recv() => handle_frame(res, TpdoKind::Tpdo1, &motors, &events_tx, velocity_window, &mut undecodable),
            res = rx2.recv() => handle_frame(res, TpdoKind::Tpdo2, &motors, &events_tx, velocity_window, &mut undecodable),
        }
    }
}

fn handle_frame(
    res: Result<CanFrame, CanIoError>,
    kind: TpdoKind,
    motors: &Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events_tx: &broadcast::Sender<Cia402Event>,
    velocity_window: Duration,
    undecodable: &mut UndecodableWarnLimiter,
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

    // The legacy manager remains the shared heartbeat/0x1018/0x2001 inventory
    // owner, but Meow Motor TPDOs belong exclusively to its dedicated driver.
    // Gate on the complete identity tuple so an unrelated short PDO can never
    // be reclassified by payload length alone.
    if entry
        .inner
        .lock()
        .unwrap()
        .identity
        .as_ref()
        .and_then(model_for_identity)
        .is_some()
    {
        return;
    }

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
                } else if undecodable.should_warn(nid, 1) {
                    log::warn!(
                        "TPDO1 from nid 0x{nid:02X}: bad length {} (want >=12); \
                         further identical warnings for this node are suppressed",
                        frame.data().len()
                    );
                }
            }
            TpdoKind::Tpdo2 => {
                if let Some(f) = decode_tpdo2(frame.data()) {
                    apply_tpdo2(&mut inner.measurements, f);
                    inner.tpdo2_revision = inner.tpdo2_revision.wrapping_add(1);
                    // 重新计算 logic + 边沿事件
                    let new_logic =
                        status_word_to_logic(f.status_word, inner.target_mode, f.error_code);
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
                } else if undecodable.should_warn(nid, 2) {
                    log::warn!(
                        "TPDO2 from nid 0x{nid:02X}: bad length {} (want >=10); \
                         further identical warnings for this node are suppressed",
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meow_motor::{PRODUCT_CODE_4310, VENDOR_ID};
    use crate::types::MotorIdentity;

    #[test]
    fn exact_meow_identity_is_left_to_the_dedicated_tpdo_listener() {
        let entry = Arc::new(MotorEntry::new(1));
        entry.inner.lock().unwrap().identity = Some(MotorIdentity {
            node_id: 1,
            vendor_id: VENDOR_ID,
            product_code: PRODUCT_CODE_4310,
            revision_number: 1,
            serial_number: 2,
            product_name: None,
        });
        let motors = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let (events, _) = broadcast::channel(4);
        let mut limiter = UndecodableWarnLimiter::default();
        let frame = CanFrame::new_fd(0x181, &[0; 8], true).unwrap();

        handle_frame(
            Ok(frame),
            TpdoKind::Tpdo1,
            &motors,
            &events,
            Duration::from_millis(15),
            &mut limiter,
        );

        let inner = entry.inner.lock().unwrap();
        assert!(inner.last_tpdo.is_none());
        assert!(!inner.online);
        assert_eq!(inner.measurements, Default::default());
    }

    /// **回归护栏(2026-08-09)**:第三方 vendor `0x4859444C` 的 CiA402 电机(车队实际在用的
    /// 那批)必须照常解码。上一版按 vendor id 猜"是不是电机"、只放行 HexFellow,把这批
    /// 误判成非电机 → TPDO 不再解码 → status word / logic 恒为 None → 机械臂起不来
    /// ("SDO fault state cleared but TPDO2 did not converge (cached sw=None, logic=None)")。
    /// 设备类别不可从 vendor 推断,解码必须对所有已登记节点一视同仁。
    #[test]
    fn third_party_vendor_cia402_motor_still_decodes() {
        const THIRD_PARTY_VENDOR: u32 = 0x4859_444C;

        let entry = Arc::new(MotorEntry::new(1));
        entry.inner.lock().unwrap().identity = Some(MotorIdentity {
            node_id: 1,
            vendor_id: THIRD_PARTY_VENDOR,
            product_code: 0xAAAA_0001,
            revision_number: 8,
            serial_number: 1,
            product_name: None,
        });
        let motors = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let (events, _) = broadcast::channel(4);
        let mut limiter = UndecodableWarnLimiter::default();
        // 合法 10B TPDO2 → 必须解出 status word 并推出 logic。
        let frame = CanFrame::new_fd(0x281, &[0u8; 10], true).unwrap();

        handle_frame(
            Ok(frame),
            TpdoKind::Tpdo2,
            &motors,
            &events,
            Duration::from_millis(15),
            &mut limiter,
        );

        let inner = entry.inner.lock().unwrap();
        assert!(inner.online);
        assert!(
            inner.measurements.status_word.is_some(),
            "第三方 vendor 的 CiA402 电机必须照常解码,否则 initialize 永远等不到 TPDO2 收敛"
        );
        assert!(inner.logic.is_some(), "logic 必须被推导出来");
    }

    /// 非电机设备(如与底盘共线的 lift,8 字节 TPDO1)解不出来是正常的:liveness 照记,
    /// 重复告警被压掉,但**不跳过解码尝试** —— 跳过与否不该由我们猜设备类别来决定。
    #[test]
    fn undecodable_frames_warn_once_then_stay_quiet() {
        let mut limiter = UndecodableWarnLimiter::default();
        assert!(limiter.should_warn(0x14, 1), "首次必须告警");
        assert!(!limiter.should_warn(0x14, 1), "同一 (nid,tpdo) 立即重复应静默");
        assert!(limiter.should_warn(0x14, 2), "另一条 TPDO 独立计时");
        assert!(limiter.should_warn(0x15, 1), "另一个节点独立计时");
    }

    /// 非电机设备仍计入 liveness(共享清单本就拥有 heartbeat/online)。
    #[test]
    fn non_motor_device_is_still_tracked_for_liveness() {
        let entry = Arc::new(MotorEntry::new(20));
        entry.inner.lock().unwrap().identity = Some(MotorIdentity {
            node_id: 20,
            vendor_id: MotorIdentity::HEXMEOW_VENDOR_ID,
            product_code: 0x006C_6674, // ASCII "lft"
            revision_number: 1,
            serial_number: 0x0111_B49A,
            product_name: Some("hexmeow-lift-driver".into()),
        });
        let motors = Arc::new(RwLock::new(HashMap::from([(20, entry.clone())])));
        let (events, mut rx) = broadcast::channel(4);
        let mut limiter = UndecodableWarnLimiter::default();
        // 真实 lift TPDO1:8 字节,按电机布局解不出来。
        let frame = CanFrame::new_data(0x194u16, &[0, 0, 0, 0, 0x12, 0x34, 0x01, 0x03]).unwrap();

        handle_frame(
            Ok(frame),
            TpdoKind::Tpdo1,
            &motors,
            &events,
            Duration::from_millis(15),
            &mut limiter,
        );

        let inner = entry.inner.lock().unwrap();
        assert!(inner.last_tpdo.is_some());
        assert!(inner.online);
        assert!(matches!(
            rx.try_recv(),
            Ok(Cia402Event::NodeOnline { nid: 20 })
        ));
    }

}
