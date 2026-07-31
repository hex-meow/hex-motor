//! 入向心跳监听 + 自动 identify + 在线状态周期监控。

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex as StdMutex, RwLock};
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFilter};
use tokio::sync::broadcast;
use tokio::time::MissedTickBehavior;
use tokio_util::sync::CancellationToken;

use crate::canopen::{nmt, sdo};
use crate::error::Result;
use crate::types::MotorIdentity;

use super::device_can::{
    begin_device_can_config_read, finish_device_can_config_read, read_device_can_config,
};
use super::events::Cia402Event;
use super::motor_entry::{AutoIdentifyState, MotorEntry, MotorEntryInner};
use super::types::{MotorLifecycle, ReinitReason};

/// 0x700..=0x77F：所有节点的入向心跳。
const HB_COB_BASE: u16 = 0x700;
const HB_COB_MASK: u16 = 0x780;

/// 常驻 task：订阅 HB 区，更新条目，新节点自动 identify。
///
/// `inflight_ops` 是 Manager 共享的 "正在做某种独占 SDO 操作的 nid" 集合
/// （identify / initialize 等都注册到这里，避免 SDO 段互撞）。
pub(crate) async fn run_discovery(
    bus: Arc<dyn CanBus>,
    our_hb_node_id: u8,
    motors: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    inflight_ops: Arc<StdMutex<HashSet<u8>>>,
    events_tx: broadcast::Sender<Cia402Event>,
    sdo_timeout: Duration,
    cancel: CancellationToken,
) {
    let mut rx = match bus
        .subscribe(CanFilter::standard(HB_COB_BASE, HB_COB_MASK))
        .await
    {
        Ok(r) => r,
        Err(e) => {
            log::error!("Discovery: subscribe failed: {e}");
            return;
        }
    };

    loop {
        tokio::select! {
            _ = cancel.cancelled() => {
                log::debug!("Discovery cancelled");
                return;
            }
            frame_res = rx.recv() => {
                let frame = match frame_res {
                    Ok(f) => f,
                    Err(e) => {
                        // Lagged 也是 CanIoError 的一种；忽略一下继续。
                        log::warn!("Discovery rx error (continuing): {e}");
                        continue;
                    }
                };
                let Some((nid, state)) = nmt::parse_heartbeat(&frame) else {
                    continue;
                };
                if nid == our_hb_node_id {
                    continue;
                }
                handle_inbound_hb(
                    nid,
                    state,
                    &motors,
                    &inflight_ops,
                    &events_tx,
                    &bus,
                    sdo_timeout,
                    &cancel,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn handle_inbound_hb(
    nid: u8,
    state: nmt::NmtState,
    motors: &Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    inflight_ops: &Arc<StdMutex<HashSet<u8>>>,
    events_tx: &broadcast::Sender<Cia402Event>,
    bus: &Arc<dyn CanBus>,
    sdo_timeout: Duration,
    cancel: &CancellationToken,
) {
    let now = Instant::now();

    // 拿到 entry（不存在就建一个并发 NodeAppeared）。
    let (entry, is_new) = {
        let mut g = motors.write().unwrap();
        match g.get(&nid) {
            Some(e) => (e.clone(), false),
            None => {
                let e = Arc::new(MotorEntry::new(nid));
                g.insert(nid, e.clone());
                (e, true)
            }
        }
    };
    if is_new {
        let _ = events_tx.send(Cia402Event::NodeAppeared { nid });
    }

    // 更新可变状态。
    let identify_attempt;
    let needs_reinit_event;
    let needs_online_event;
    let live_state;
    {
        let mut inner = entry.inner.lock().unwrap();
        inner.last_heartbeat = Some(now);
        if inner.nmt_state != Some(state) {
            inner.nmt_revision = inner.nmt_revision.wrapping_add(1);
        }
        inner.nmt_state = Some(state);

        // 快速重启可能短于 offline 阈值。Boot-Up 本身就是新会话的强信号，
        // 必须立即丢弃旧身份/配置，不能等 liveness monitor 判离线。
        if state == nmt::NmtState::BootUp {
            inner.bootup_revision = inner.bootup_revision.wrapping_add(1);
            invalidate_for_new_device_session(&mut inner);
        }

        // 在线 edge：新出现 / 之前 offline → 现在 online
        needs_online_event = !inner.online;
        inner.online = true;

        // Initialized 节点离开 Operational → NeedsReinit
        needs_reinit_event = if matches!(inner.lifecycle, MotorLifecycle::Initialized)
            && state != nmt::NmtState::Operational
        {
            inner.lifecycle = MotorLifecycle::NeedsReinit {
                reason: ReinitReason::LeftOperational,
            };
            true
        } else {
            false
        };

        // 每个 session 最多真正启动一次自动识别。inflight 忙时保留 Ready；
        // 后续心跳只会重查本地锁，不发 CAN，直到能启动那唯一一次 SDO 序列。
        identify_attempt = if matches!(inner.lifecycle, MotorLifecycle::Unknown) {
            match inner.auto_identify_state {
                AutoIdentifyState::Ready => Some(inner.session_epoch),
                AutoIdentifyState::Attempted => None,
            }
        } else {
            None
        };

        live_state = inner.build_live_state(now);
    }

    if needs_online_event {
        let _ = events_tx.send(Cia402Event::NodeOnline { nid });
    }
    if needs_reinit_event {
        let _ = events_tx.send(Cia402Event::NeedsReinit {
            nid,
            reason: ReinitReason::LeftOperational,
        });
    }

    // 推给 status() / subscribe_status() —— last_heartbeat / nmt_state / online 都变了。
    entry.publish(live_state);

    if let Some(session_epoch) = identify_attempt {
        let started = spawn_identify_if_idle(
            nid,
            entry.clone(),
            session_epoch,
            inflight_ops.clone(),
            events_tx.clone(),
            bus.clone(),
            sdo_timeout,
            cancel.clone(),
        );
        if started {
            let mut inner = entry.inner.lock().unwrap();
            if inner.session_epoch == session_epoch
                && matches!(inner.lifecycle, MotorLifecycle::Unknown)
                && inner.auto_identify_state == AutoIdentifyState::Ready
            {
                inner.auto_identify_state = AutoIdentifyState::Attempted;
            }
        }
    }
}

fn invalidate_for_new_device_session(inner: &mut MotorEntryInner) {
    inner.session_epoch = inner.session_epoch.wrapping_add(1);
    inner.auto_identify_state = AutoIdentifyState::Ready;
    inner.identity = None;
    inner.can_config = crate::types::DeviceCanConfigStatus::Pending;
    inner.lifecycle = MotorLifecycle::Unknown;
    inner.last_tpdo = None;
    inner.logic = None;
    inner.target_mode = None;
    inner.peak_torque_nm = None;
    inner.mit_kp_kd_factor = None;
    inner.measurements = Default::default();
    inner.vel_filter = Default::default();
}

#[allow(clippy::too_many_arguments)]
fn spawn_identify_if_idle(
    nid: u8,
    entry: Arc<MotorEntry>,
    session_epoch: u64,
    inflight: Arc<StdMutex<HashSet<u8>>>,
    events_tx: broadcast::Sender<Cia402Event>,
    bus: Arc<dyn CanBus>,
    sdo_timeout: Duration,
    cancel: CancellationToken,
) -> bool {
    {
        let mut g = inflight.lock().unwrap();
        if !g.insert(nid) {
            // 已经有 identify / initialize 等独占操作在跑
            return false;
        }
    }
    let inflight_guard = OwnedInflightGuard { set: inflight, nid };

    tokio::spawn(async move {
        let _inflight_guard = inflight_guard;
        let res = tokio::select! {
            _ = cancel.cancelled() => None,
            r = async {
                let identity = identify_once(bus.as_ref(), nid, sdo_timeout).await?;
                if !begin_device_can_config_read(&entry, session_epoch, &identity) {
                    return Ok::<Option<MotorIdentity>, crate::error::Error>(None);
                }
                let can_config =
                    read_device_can_config(bus.as_ref(), &identity, sdo_timeout).await;
                if let crate::types::DeviceCanConfigStatus::ReadFailed { reason } = &can_config {
                    log::warn!("read CAN config for nid 0x{nid:02X} failed: {reason}");
                }
                if !finish_device_can_config_read(&entry, session_epoch, can_config) {
                    return Ok(None);
                }
                Ok(Some(identity))
            } => Some(r),
        };

        match res {
            Some(Ok(Some(identity))) => {
                let inner = entry.inner.lock().unwrap();
                if inner.session_epoch == session_epoch {
                    let _ = events_tx.send(Cia402Event::Identified { nid, identity });
                }
            }
            Some(Ok(None)) => log::debug!(
                "discarded stale identify result for nid 0x{nid:02X}, epoch {session_epoch}"
            ),
            Some(Err(e)) => {
                let inner = entry.inner.lock().unwrap();
                if inner.session_epoch == session_epoch {
                    log::warn!("identify nid 0x{nid:02X} failed: {e}");
                    let _ = events_tx.send(Cia402Event::IdentifyFailed {
                        nid,
                        reason: e.to_string(),
                    });
                }
            }
            None => log::debug!("identify nid 0x{nid:02X} cancelled"),
        }
    });
    true
}

/// 后台 identify task 独占节点的 owned guard。任务取消、panic 或正常退出都会清理。
struct OwnedInflightGuard {
    set: Arc<StdMutex<HashSet<u8>>>,
    nid: u8,
}

impl Drop for OwnedInflightGuard {
    fn drop(&mut self) {
        self.set.lock().unwrap().remove(&self.nid);
    }
}

/// 读 0x1018 (强制) + 0x1008 (可选)。0x1008 失败只记日志。
pub(crate) async fn identify_once(
    bus: &dyn CanBus,
    nid: u8,
    sdo_timeout: Duration,
) -> Result<MotorIdentity> {
    let timeout = Some(sdo_timeout);
    let vendor_id = sdo::upload_u32(bus, nid, 0x1018, 1, timeout).await?;
    let product_code = sdo::upload_u32(bus, nid, 0x1018, 2, timeout).await?;
    let revision_number = sdo::upload_u32(bus, nid, 0x1018, 3, timeout).await?;
    let serial_number = sdo::upload_u32(bus, nid, 0x1018, 4, timeout).await?;
    let product_name = match sdo::upload_string(bus, nid, 0x1008, 0, timeout).await {
        Ok(s) => Some(s),
        Err(e) => {
            log::debug!("nid 0x{nid:02X}: 0x1008 not readable ({e}); proceeding without product name");
            None
        }
    };
    Ok(MotorIdentity {
        node_id: nid,
        vendor_id,
        product_code,
        revision_number,
        serial_number,
        product_name,
    })
}

/// 常驻 task：周期扫描所有电机，发 `NodeOffline` 边沿事件。
///
/// 阈值按 `lifecycle` 分两段：
/// - **pre-init**（lifecycle != Initialized）：只有电机心跳能用。HEX-MECHA 电机出厂
///   默认 500ms 一发；阈值 = `2.5 × motor_heartbeat_period`，默认约 1.25s。
/// - **post-init**（lifecycle == Initialized）：TPDO 在流（默认 ~20ms 一发），
///   阈值 = `initialized_stale_threshold`，默认 200ms。
///
/// Tick 频率取较小阈值的 1/2，clamp 到 [20ms, 200ms]，保证 offline 检出延迟可控。
pub(crate) async fn run_liveness_monitor(
    motors: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events_tx: broadcast::Sender<Cia402Event>,
    motor_heartbeat_period: Duration,
    initialized_stale_threshold: Duration,
    cancel: CancellationToken,
) {
    let uninit_threshold = (motor_heartbeat_period * 5) / 2;
    let init_threshold = initialized_stale_threshold;
    let tick_period = init_threshold
        .min(uninit_threshold)
        .div_f32(2.0)
        .clamp(Duration::from_millis(20), Duration::from_millis(200));
    log::debug!(
        "liveness monitor: uninit_threshold={uninit_threshold:?}, \
         init_threshold={init_threshold:?}, tick={tick_period:?}"
    );

    let mut tick = tokio::time::interval(tick_period);
    tick.set_missed_tick_behavior(MissedTickBehavior::Delay);
    tick.tick().await;
    loop {
        tokio::select! {
            _ = cancel.cancelled() => {
                log::debug!("Liveness monitor cancelled");
                return;
            }
            _ = tick.tick() => {
                let now = Instant::now();
                // 注意：先 clone Arc 出来再单独锁 entry，避免长持 map 锁。
                let entries: Vec<Arc<MotorEntry>> = {
                    let g = motors.read().unwrap();
                    g.values().cloned().collect()
                };
                for entry in entries {
                    let result = {
                        let mut inner = entry.inner.lock().unwrap();
                        if !inner.online {
                            continue; // 没必要从 offline → offline
                        }
                        let threshold = if matches!(inner.lifecycle, MotorLifecycle::Initialized) {
                            init_threshold
                        } else {
                            uninit_threshold
                        };
                        let last_seen = inner.last_seen();
                        let online_now = last_seen
                            .is_some_and(|t| now.saturating_duration_since(t) <= threshold);
                        let was = inner.online;
                        inner.online = online_now;
                        let live = if was && !online_now {
                            // Node-ID 可以在掉电期间被另一台设备复用。离线边沿必须
                            // 丢弃旧身份/方言；下一次入向心跳会从 Unknown 自动重识别。
                            // 同时清 NMT 状态，防止仅凭一帧 TPDO 就被误认为“本次会话
                            // 已收到心跳”而允许设置写操作。
                            invalidate_for_new_device_session(&mut inner);
                            inner.nmt_revision = inner.nmt_revision.wrapping_add(1);
                            inner.nmt_state = None;
                            Some(inner.build_live_state(now))
                        } else {
                            None
                        };
                        (was, online_now, live)
                    };
                    let (was_online, is_online, live_state) = result;
                    if was_online && !is_online {
                        let _ = events_tx.send(Cia402Event::NodeOffline {
                            nid: entry.node_id,
                        });
                        if let Some(state) = live_state {
                            entry.publish(state);
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    use async_trait::async_trait;
    use can_transport::{CanCapabilities, CanFrame, CanId, CanIoError, CanRx};
    use tokio::sync::{mpsc, Notify};

    #[tokio::test]
    async fn failed_auto_identify_runs_only_once_per_session() {
        let bus_impl = Arc::new(FailingIdentityBus::default());
        let bus: Arc<dyn CanBus> = bus_impl.clone();
        let motors = Arc::new(RwLock::new(HashMap::new()));
        let inflight = Arc::new(StdMutex::new(HashSet::new()));
        let (events_tx, mut events_rx) = broadcast::channel(16);
        let cancel = CancellationToken::new();

        handle_inbound_hb(
            1,
            nmt::NmtState::PreOperational,
            &motors,
            &inflight,
            &events_tx,
            &bus,
            Duration::from_millis(20),
            &cancel,
        );
        loop {
            let event = events_rx.recv().await.unwrap();
            if matches!(event, Cia402Event::IdentifyFailed { nid: 1, .. }) {
                break;
            }
        }
        assert_eq!(bus_impl.send_count.load(Ordering::Acquire), 1);

        for _ in 0..4 {
            handle_inbound_hb(
                1,
                nmt::NmtState::PreOperational,
                &motors,
                &inflight,
                &events_tx,
                &bus,
                Duration::from_millis(20),
                &cancel,
            );
        }
        tokio::task::yield_now().await;
        assert_eq!(bus_impl.send_count.load(Ordering::Acquire), 1);
        let entry = motors.read().unwrap()[&1].clone();
        assert_eq!(
            entry.inner.lock().unwrap().auto_identify_state,
            AutoIdentifyState::Attempted
        );

        // Boot-Up creates a new epoch and therefore one new automatic attempt.
        handle_inbound_hb(
            1,
            nmt::NmtState::BootUp,
            &motors,
            &inflight,
            &events_tx,
            &bus,
            Duration::from_millis(20),
            &cancel,
        );
        loop {
            let event = events_rx.recv().await.unwrap();
            if matches!(event, Cia402Event::IdentifyFailed { nid: 1, .. }) {
                break;
            }
        }
        assert_eq!(bus_impl.send_count.load(Ordering::Acquire), 2);
        assert_eq!(entry.inner.lock().unwrap().session_epoch, 1);
        cancel.cancel();
    }

    #[tokio::test]
    async fn bootup_invalidates_cached_identity_and_retries_identify_after_inflight_clears() {
        let state = Arc::new(BlockingConfigState::default());
        let bus = Arc::new(BlockingConfigBus::new(state.clone()));
        let entry = Arc::new(MotorEntry::new(1));
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.identity = Some(MotorIdentity {
                node_id: 1,
                vendor_id: 0x4859_444C,
                product_code: 0xAAAA_0001,
                revision_number: 0,
                serial_number: 0,
                product_name: None,
            });
            inner.can_config =
                crate::types::DeviceCanConfigStatus::Available(crate::types::DeviceCanConfig {
                    stored_node_id: 1,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(5_000_000),
                    transmit_pdo_brs: Some(true),
                });
            inner.lifecycle = MotorLifecycle::Initialized;
            inner.online = true;
            inner.last_heartbeat = Some(Instant::now());
            inner.last_tpdo = Some(Instant::now());
            inner.nmt_state = Some(nmt::NmtState::Operational);
            inner.measurements.position_rev = Some(0.25);
        }
        let motors = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let inflight = Arc::new(StdMutex::new(HashSet::from([1])));
        let (events_tx, _) = broadcast::channel(8);
        let cancel = CancellationToken::new();
        let bus_dyn: Arc<dyn CanBus> = bus;

        handle_inbound_hb(
            1,
            nmt::NmtState::BootUp,
            &motors,
            &inflight,
            &events_tx,
            &bus_dyn,
            Duration::from_millis(50),
            &cancel,
        );
        {
            let inner = entry.inner.lock().unwrap();
            assert!(inner.identity.is_none());
            assert_eq!(
                inner.can_config,
                crate::types::DeviceCanConfigStatus::Pending
            );
            assert_eq!(inner.lifecycle, MotorLifecycle::Unknown);
            assert_eq!(inner.nmt_state, Some(nmt::NmtState::BootUp));
            assert_eq!(inner.bootup_revision, 1);
            assert!(inner.last_heartbeat.is_some());
            assert!(inner.last_tpdo.is_none());
            assert_eq!(inner.measurements, Default::default());
        }

        // Boot-Up 时设置事务仍占着 inflight，因此没有启动 identify。下一帧心跳
        // 在独占区释放后必须重新尝试，而不是继续信任旧缓存。
        inflight.lock().unwrap().remove(&1);
        handle_inbound_hb(
            1,
            nmt::NmtState::PreOperational,
            &motors,
            &inflight,
            &events_tx,
            &bus_dyn,
            Duration::from_secs(5),
            &cancel,
        );
        tokio::time::timeout(Duration::from_secs(1), state.wait_until_config_requested())
            .await
            .expect("the heartbeat after Boot-Up should retry identity and reach config read");
        {
            let inner = entry.inner.lock().unwrap();
            assert_eq!(
                inner
                    .identity
                    .as_ref()
                    .map(|identity| (identity.vendor_id, identity.product_code)),
                Some((0x4859_444C, 0xAAAA_0001))
            );
            assert_eq!(inner.lifecycle, MotorLifecycle::Identified);
            assert_eq!(inner.session_epoch, 1);
            assert_eq!(inner.auto_identify_state, AutoIdentifyState::Attempted);
        }

        // 旧 identify 正阻塞在配置读时又收到 Boot-Up：epoch 递增并清缓存。
        handle_inbound_hb(
            1,
            nmt::NmtState::BootUp,
            &motors,
            &inflight,
            &events_tx,
            &bus_dyn,
            Duration::from_secs(5),
            &cancel,
        );
        {
            let inner = entry.inner.lock().unwrap();
            assert_eq!(inner.session_epoch, 2);
            assert_eq!(inner.bootup_revision, 2);
            assert!(inner.identity.is_none());
            assert_eq!(
                inner.can_config,
                crate::types::DeviceCanConfigStatus::Pending
            );
            assert_eq!(inner.auto_identify_state, AutoIdentifyState::Ready);
        }

        // 让真实旧 task 完成；epoch guard 必须丢弃它的结果。
        state.release_config();
        tokio::time::timeout(Duration::from_millis(200), async {
            while inflight.lock().unwrap().contains(&1) {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("released stale identify should finish");
        {
            let inner = entry.inner.lock().unwrap();
            assert!(inner.identity.is_none());
            assert_eq!(
                inner.can_config,
                crate::types::DeviceCanConfigStatus::Pending
            );
            assert_eq!(inner.lifecycle, MotorLifecycle::Unknown);
        }

        // 下一帧心跳启动新 epoch 唯一一次真实 SDO identify。
        handle_inbound_hb(
            1,
            nmt::NmtState::PreOperational,
            &motors,
            &inflight,
            &events_tx,
            &bus_dyn,
            Duration::from_secs(5),
            &cancel,
        );
        tokio::time::timeout(Duration::from_millis(200), async {
            loop {
                let ready = {
                    let inner = entry.inner.lock().unwrap();
                    inner.session_epoch == 2
                        && inner.identity.is_some()
                        && matches!(
                            inner.can_config,
                            crate::types::DeviceCanConfigStatus::Available(_)
                        )
                };
                if ready {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("new epoch heartbeat should identify successfully");
        assert_eq!(
            entry.inner.lock().unwrap().auto_identify_state,
            AutoIdentifyState::Attempted
        );

        cancel.cancel();
    }

    #[tokio::test]
    async fn offline_edge_clears_identity_config_and_nmt_for_next_session() {
        let entry = Arc::new(MotorEntry::new(1));
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.identity = Some(MotorIdentity {
                node_id: 1,
                vendor_id: 0x4859_444C,
                product_code: 0xAAAA_0001,
                revision_number: 0,
                serial_number: 0,
                product_name: None,
            });
            inner.can_config =
                crate::types::DeviceCanConfigStatus::Available(crate::types::DeviceCanConfig {
                    stored_node_id: 1,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(5_000_000),
                    transmit_pdo_brs: Some(false),
                });
            inner.lifecycle = MotorLifecycle::Identified;
            inner.online = true;
            inner.last_heartbeat = Some(Instant::now() - Duration::from_secs(1));
            inner.nmt_state = Some(nmt::NmtState::PreOperational);
        }
        let motors = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let (events_tx, mut events_rx) = broadcast::channel(4);
        let cancel = CancellationToken::new();
        let task = tokio::spawn(run_liveness_monitor(
            motors,
            events_tx,
            Duration::from_millis(10),
            Duration::from_millis(10),
            cancel.clone(),
        ));

        let event = tokio::time::timeout(Duration::from_millis(200), events_rx.recv())
            .await
            .expect("liveness monitor should detect the stale node")
            .expect("event channel should stay open");
        assert!(matches!(event, Cia402Event::NodeOffline { nid: 1 }));
        {
            let inner = entry.inner.lock().unwrap();
            assert!(!inner.online);
            assert!(inner.identity.is_none());
            assert_eq!(
                inner.can_config,
                crate::types::DeviceCanConfigStatus::Pending
            );
            assert_eq!(inner.lifecycle, MotorLifecycle::Unknown);
            assert_eq!(inner.nmt_state, None);
            assert_eq!(inner.bootup_revision, 0);
        }

        cancel.cancel();
        task.await.unwrap();
    }

    #[tokio::test]
    async fn old_identify_completion_after_offline_cannot_restore_stale_cache() {
        let state = Arc::new(BlockingConfigState::default());
        let bus: Arc<dyn CanBus> = Arc::new(BlockingConfigBus::new(state.clone()));
        let entry = Arc::new(MotorEntry::new(1));
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.online = true;
            inner.last_heartbeat = Some(Instant::now() - Duration::from_secs(1));
            inner.nmt_state = Some(nmt::NmtState::PreOperational);
        }
        let motors = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let inflight = Arc::new(StdMutex::new(HashSet::new()));
        let (events_tx, mut events_rx) = broadcast::channel(8);
        let identify_cancel = CancellationToken::new();
        assert!(spawn_identify_if_idle(
            1,
            entry.clone(),
            0,
            inflight.clone(),
            events_tx.clone(),
            bus,
            Duration::from_secs(5),
            identify_cancel.clone(),
        ));
        tokio::time::timeout(Duration::from_secs(1), state.wait_until_config_requested())
            .await
            .expect("old identify should block in its configuration read");

        let liveness_cancel = CancellationToken::new();
        let liveness_task = tokio::spawn(run_liveness_monitor(
            motors,
            events_tx,
            Duration::from_millis(10),
            Duration::from_millis(10),
            liveness_cancel.clone(),
        ));
        loop {
            let event = tokio::time::timeout(Duration::from_millis(200), events_rx.recv())
                .await
                .expect("offline edge should be emitted")
                .expect("event channel should stay open");
            if matches!(event, Cia402Event::NodeOffline { nid: 1 }) {
                break;
            }
        }
        assert_eq!(entry.inner.lock().unwrap().session_epoch, 1);

        state.release_config();
        tokio::time::timeout(Duration::from_millis(200), async {
            while inflight.lock().unwrap().contains(&1) {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("stale identify should finish after release");
        {
            let inner = entry.inner.lock().unwrap();
            assert!(!inner.online);
            assert_eq!(inner.session_epoch, 1);
            assert!(inner.identity.is_none());
            assert_eq!(
                inner.can_config,
                crate::types::DeviceCanConfigStatus::Pending
            );
            assert_eq!(inner.lifecycle, MotorLifecycle::Unknown);
        }

        identify_cancel.cancel();
        liveness_cancel.cancel();
        liveness_task.await.unwrap();
    }

    #[tokio::test]
    async fn cancelling_during_config_read_releases_bus_and_inflight_immediately() {
        let state = Arc::new(BlockingConfigState::default());
        let concrete_bus = Arc::new(BlockingConfigBus::new(state.clone()));
        let weak_bus = Arc::downgrade(&concrete_bus);
        let bus: Arc<dyn CanBus> = concrete_bus.clone();
        let entry = Arc::new(MotorEntry::new(1));
        let inflight = Arc::new(StdMutex::new(HashSet::new()));
        let (events_tx, _) = broadcast::channel(4);
        let cancel = CancellationToken::new();

        spawn_identify_if_idle(
            1,
            entry.clone(),
            0,
            inflight.clone(),
            events_tx,
            bus,
            Duration::from_secs(5),
            cancel.clone(),
        );
        drop(concrete_bus);

        tokio::time::timeout(Duration::from_secs(1), state.wait_until_config_requested())
            .await
            .expect("identify should reach the deliberately blocked config SDO");
        assert!(entry.inner.lock().unwrap().identity.is_some());
        assert!(inflight.lock().unwrap().contains(&1));

        cancel.cancel();
        tokio::time::timeout(Duration::from_millis(200), async {
            loop {
                let inflight_cleared = !inflight.lock().unwrap().contains(&1);
                let bus_released = weak_bus.upgrade().is_none();
                if inflight_cleared && bus_released {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("cancelled identify task should promptly release its bus and inflight slot");
    }

    #[derive(Default)]
    struct FailingIdentityBus {
        send_count: AtomicUsize,
    }

    #[async_trait]
    impl CanBus for FailingIdentityBus {
        async fn send(&self, _frame: CanFrame) -> std::result::Result<(), CanIoError> {
            self.send_count.fetch_add(1, Ordering::AcqRel);
            Err(CanIoError::backend(std::io::Error::other(
                "simulated identity transport failure",
            )))
        }

        async fn subscribe(
            &self,
            _filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            let (_tx, rx) = mpsc::unbounded_channel();
            Ok(Box::new(BlockingConfigRx { rx }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    #[derive(Default)]
    struct BlockingConfigState {
        config_requested: AtomicBool,
        config_released: AtomicBool,
        request_notify: Notify,
        release_notify: Notify,
    }

    impl BlockingConfigState {
        fn mark_config_requested(&self) {
            self.config_requested.store(true, Ordering::Release);
            self.request_notify.notify_waiters();
        }

        async fn wait_until_config_requested(&self) {
            loop {
                if self.config_requested.load(Ordering::Acquire) {
                    return;
                }
                self.request_notify.notified().await;
            }
        }

        fn release_config(&self) {
            self.config_released.store(true, Ordering::Release);
            self.release_notify.notify_waiters();
        }

        async fn wait_until_config_released(&self) {
            loop {
                if self.config_released.load(Ordering::Acquire) {
                    return;
                }
                self.release_notify.notified().await;
            }
        }
    }

    struct BlockingConfigBus {
        state: Arc<BlockingConfigState>,
        response_tx: StdMutex<Option<mpsc::UnboundedSender<CanFrame>>>,
    }

    impl BlockingConfigBus {
        fn new(state: Arc<BlockingConfigState>) -> Self {
            Self {
                state,
                response_tx: StdMutex::new(None),
            }
        }
    }

    #[async_trait]
    impl CanBus for BlockingConfigBus {
        async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
            let request = frame.data();
            if request.len() != 8 || request[0] != 0x40 {
                return Err(CanIoError::backend(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "test bus expected an expedited SDO upload request",
                )));
            }
            let nid = match frame.id() {
                CanId::Standard(id) if (0x601..=0x67F).contains(&id) => (id - 0x600) as u8,
                _ => {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "test bus received an invalid RSDO COB-ID",
                    )))
                }
            };
            let index = u16::from_le_bytes([request[1], request[2]]);
            let subindex = request[3];

            if index == 0x2001 {
                self.state.mark_config_requested();
                self.state.wait_until_config_released().await;
            }

            let value = match (index, subindex) {
                (0x1018, 1) => 0x4859_444Cu32.to_le_bytes().to_vec(),
                (0x1018, 2) => 0xAAAA_0001u32.to_le_bytes().to_vec(),
                (0x1018, 3 | 4) => 0u32.to_le_bytes().to_vec(),
                (0x1008, 0) => b"X".to_vec(),
                (0x2001, 0) => vec![4],
                (0x2001, 1) => vec![1],
                (0x2001, 2) => 1_000_000u32.to_le_bytes().to_vec(),
                (0x2001, 3) => 5_000_000u32.to_le_bytes().to_vec(),
                (0x2001, 4) => vec![1],
                _ => {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        format!("unexpected test SDO 0x{index:04X}:{subindex:02X}"),
                    )))
                }
            };

            let mut response = [0u8; 8];
            response[0] = 0x43 | (((4 - value.len()) as u8) << 2);
            response[1] = request[1];
            response[2] = request[2];
            response[3] = subindex;
            response[4..4 + value.len()].copy_from_slice(&value);
            let response = CanFrame::new_data(0x580u16 + nid as u16, &response)?;
            self.response_tx
                .lock()
                .unwrap()
                .as_ref()
                .ok_or(CanIoError::Disconnected)?
                .send(response)
                .map_err(|_| CanIoError::Disconnected)
        }

        async fn subscribe(
            &self,
            _filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            *self.response_tx.lock().unwrap() = Some(tx);
            Ok(Box::new(BlockingConfigRx { rx }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    struct BlockingConfigRx {
        rx: mpsc::UnboundedReceiver<CanFrame>,
    }

    #[async_trait]
    impl CanRx for BlockingConfigRx {
        async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
            self.rx.recv().await.ok_or(CanIoError::Disconnected)
        }

        fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
            match self.rx.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
            }
        }
    }
}
