//! [`MotorEntry`]：Manager 内部共享给所有后台 task 的"每电机条目"。
//!
//! 设计要点：
//! - 一个电机一个 `Arc<MotorEntry>`；map 由 Manager 持有。
//! - 全部可变字段塞在一个 [`MotorEntryInner`] 里，由单个 `std::sync::Mutex` 保护。
//!   这样不会出现"先锁 identity 再锁 lifecycle"的多锁顺序问题。
//! - 临界区都是几行字段赋值，从不在持锁时 await，所以用 std Mutex 不会阻塞 runtime。
//! - [`MotorEntry::snapshot`] 是 `ArcSwap<LiveState>`，被 [`MotorEntry::publish`]
//!   原子地刷新；[`crate::cia402::Cia402Manager::status`] 通过它实现无锁 fast path。
//! - [`MotorEntry::subscribers`] 是订阅者列表，`publish` 时遍历做 try_send fan-out。

use std::sync::{Arc, Mutex};
use std::time::Instant;

use arc_swap::ArcSwap;

use crate::canopen::nmt::NmtState;
use crate::types::{MotorIdentity, MotorMode};

use super::subscribe::Subscriber;
use super::types::{Connection, LiveState, Logic, Measurements, MotorLifecycle};
use super::velocity::VelocityEstimator;

#[derive(Debug)]
pub(crate) struct MotorEntry {
    pub node_id: u8,
    pub inner: Mutex<MotorEntryInner>,
    /// 给 `status()` 无锁读取的最近一次快照。每次 `publish` 时被原子替换。
    pub snapshot: ArcSwap<LiveState>,
    /// 每路 [`crate::cia402::StatusStream`] 在 Manager 侧的 sender 句柄。
    /// 投递失败（channel 关闭）的会在 publish 时被 retain 掉。
    pub subscribers: Mutex<Vec<Subscriber>>,
}

#[derive(Debug)]
pub(crate) struct MotorEntryInner {
    pub identity: Option<MotorIdentity>,
    pub lifecycle: MotorLifecycle,
    /// 缓存的在线状态。由 liveness monitor 更新；list() 直接读。
    pub online: bool,
    pub last_heartbeat: Option<Instant>,
    pub last_tpdo: Option<Instant>,
    pub nmt_state: Option<NmtState>,
    /// 控制逻辑。由 [`crate::cia402::tpdo_listener`] 在 TPDO2 到达时
    /// 根据 status_word + `target_mode` 解算并写入。
    pub logic: Option<Logic>,
    /// 最近一次 `set_mode` 设过的目标模式（v0.1 不读 0x6061，靠这里缓存）。
    /// 没设过 → 即使 status_word 显示 Operation Enabled 也只能报 Disabled，
    /// 因为不知道当前实际跑的是什么模式（详见
    /// [`crate::cia402::codec::status_word_to_logic`]）。
    pub target_mode: Option<MotorMode>,
    /// 峰值力矩，单位 **Nm**。`initialize()` 末尾从 0x6076 (REAL32, Nm)
    /// 直接读出来缓存；读失败保持 `None`。
    /// - `set_target(Torque { nm })` 需要它把 Nm 转成 `0x6071` 的
    ///   `i16 ‰ of peak`，缺失则返回 [`crate::Error::Internal`]。
    pub peak_torque_nm: Option<f32>,
    /// MIT KP/KD 尺度因子。`initialize()` 末尾从 `0x2003:07` (REAL32) 读出
    /// 来；读失败保持 `None`。
    /// - 公式：物理 Kp [Nm/Rev] = `kp_int` × `factor`，所以
    ///   `kp_int = round(kp_nm_per_rev / factor)`。Kd 同理。
    /// - `set_target(Mit { kp, kd, .. })` 缺失它会返回 [`crate::Error::Internal`]。
    pub mit_kp_kd_factor: Option<f32>,
    /// TPDO 解码出的测量值。由 listener 持续刷新；`list()` / `status()` 读。
    pub measurements: Measurements,
    /// 速度滤波器状态。每帧 TPDO1 用 `(0x1013 时间戳, 0x6064 位置)` 喂进去，
    /// 算出来的滤波速度写进 `measurements.velocity_rev_per_s`。
    pub vel_filter: VelocityEstimator,
}

impl MotorEntry {
    pub fn new(node_id: u8) -> Self {
        let now = Instant::now();
        Self {
            node_id,
            inner: Mutex::new(MotorEntryInner {
                identity: None,
                lifecycle: MotorLifecycle::Unknown,
                online: false,
                last_heartbeat: None,
                last_tpdo: None,
                nmt_state: None,
                logic: None,
                target_mode: None,
                peak_torque_nm: None,
                mit_kp_kd_factor: None,
                measurements: Measurements::default(),
                vel_filter: VelocityEstimator::default(),
            }),
            snapshot: ArcSwap::from(Arc::new(LiveState::empty(now))),
            subscribers: Mutex::new(Vec::new()),
        }
    }

    /// 把当前 inner state 拍成 [`LiveState`]，存入 `snapshot`，并 fan-out 给所有
    /// 订阅者。**调用方必须先释放 `inner` 锁**（避免持锁中 try_send）。
    ///
    /// 订阅者侧 channel 已 close（`StatusStream` 被 drop）的会被从列表里移除。
    pub fn publish(&self, state: LiveState) {
        // 1. 原子替换快照（status() 用）
        self.snapshot.store(Arc::new(state.clone()));

        // 2. fan-out 给订阅者；try_send 不阻塞，但仍短暂持 subscribers 锁。
        //    锁内只跑 try_send + 内存里 retain；不会触及 inner mutex。
        let mut subs = self.subscribers.lock().unwrap();
        if subs.is_empty() {
            return;
        }
        subs.retain_mut(|sub| sub.push(&state));
    }

    /// 把一个新订阅者挂入列表。
    pub fn add_subscriber(&self, sub: Subscriber) {
        self.subscribers.lock().unwrap().push(sub);
    }
}

impl MotorEntryInner {
    /// max(last_heartbeat, last_tpdo)
    pub fn last_seen(&self) -> Option<Instant> {
        match (self.last_heartbeat, self.last_tpdo) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (a, b) => a.or(b),
        }
    }

    /// 把当前 inner 拍成 [`LiveState`] —— 给 `publish()` 用，在持 inner 锁的
    /// 临界区内调用。
    pub fn build_live_state(&self, now: Instant) -> LiveState {
        LiveState {
            connection: Connection {
                last_heartbeat: self.last_heartbeat,
                last_tpdo: self.last_tpdo,
                online: self.online,
                nmt_state: self.nmt_state,
            },
            logic: self.logic.clone(),
            measurements: self.measurements.clone(),
            timestamp: now,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn new_starts_unknown_offline() {
        let e = MotorEntry::new(0x10);
        let g = e.inner.lock().unwrap();
        assert_eq!(g.lifecycle, MotorLifecycle::Unknown);
        assert!(!g.online);
        assert!(g.identity.is_none());
        assert!(g.last_heartbeat.is_none());
        assert!(g.last_tpdo.is_none());
    }

    #[test]
    fn new_snapshot_is_empty() {
        let e = MotorEntry::new(0x10);
        let s = e.snapshot.load_full();
        assert!(!s.connection.online);
        assert!(s.logic.is_none());
        assert_eq!(s.measurements, Measurements::default());
    }

    #[test]
    fn last_seen_picks_max() {
        let now = Instant::now();
        let mut inner = MotorEntry::new(0x10).inner.into_inner().unwrap();
        assert_eq!(inner.last_seen(), None);

        inner.last_heartbeat = Some(now);
        assert_eq!(inner.last_seen(), Some(now));

        let later = now + Duration::from_millis(10);
        inner.last_tpdo = Some(later);
        assert_eq!(inner.last_seen(), Some(later));

        let earlier = now - Duration::from_millis(20);
        inner.last_heartbeat = Some(earlier);
        assert_eq!(inner.last_seen(), Some(later));
    }

    #[test]
    fn publish_updates_snapshot_and_drops_closed_subscribers() {
        use crate::cia402::subscribe::{StreamOptions, Subscriber};
        let e = MotorEntry::new(0x10);

        let (sub, stream) = Subscriber::new(&StreamOptions::default());
        e.add_subscriber(sub);
        assert_eq!(e.subscribers.lock().unwrap().len(), 1);

        let now = Instant::now();
        let state = {
            let mut inner = e.inner.lock().unwrap();
            inner.online = true;
            inner.last_heartbeat = Some(now);
            inner.build_live_state(now)
        };
        e.publish(state);

        assert!(e.snapshot.load_full().connection.online);

        // 对端 drop → 下一次 publish 应清掉。
        drop(stream);
        let now2 = Instant::now();
        let state2 = e.inner.lock().unwrap().build_live_state(now2);
        e.publish(state2);
        assert_eq!(e.subscribers.lock().unwrap().len(), 0);
    }
}
