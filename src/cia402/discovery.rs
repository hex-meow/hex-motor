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
use super::motor_entry::MotorEntry;
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
    let needs_identify;
    let needs_reinit_event;
    let needs_online_event;
    let live_state;
    {
        let mut inner = entry.inner.lock().unwrap();
        inner.last_heartbeat = Some(now);
        inner.nmt_state = Some(state);

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

        // identify 触发条件：lifecycle 还是 Unknown
        needs_identify = matches!(inner.lifecycle, MotorLifecycle::Unknown);

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

    if needs_identify {
        spawn_identify_if_idle(
            nid,
            entry,
            inflight_ops.clone(),
            events_tx.clone(),
            bus.clone(),
            sdo_timeout,
            cancel.clone(),
        );
    }
}

fn spawn_identify_if_idle(
    nid: u8,
    entry: Arc<MotorEntry>,
    inflight: Arc<StdMutex<HashSet<u8>>>,
    events_tx: broadcast::Sender<Cia402Event>,
    bus: Arc<dyn CanBus>,
    sdo_timeout: Duration,
    cancel: CancellationToken,
) {
    {
        let mut g = inflight.lock().unwrap();
        if !g.insert(nid) {
            // 已经有 identify / initialize 等独占操作在跑
            return;
        }
    }
    let inflight_guard = OwnedInflightGuard { set: inflight, nid };

    tokio::spawn(async move {
        let _inflight_guard = inflight_guard;
        let res = tokio::select! {
            _ = cancel.cancelled() => None,
            r = async {
                let identity = identify_once(bus.as_ref(), nid, sdo_timeout).await?;
                begin_device_can_config_read(&entry, &identity);
                let can_config =
                    read_device_can_config(bus.as_ref(), &identity, sdo_timeout).await;
                if let crate::types::DeviceCanConfigStatus::ReadFailed { reason } = &can_config {
                    log::warn!("read CAN config for nid 0x{nid:02X} failed: {reason}");
                }
                finish_device_can_config_read(&entry, can_config);
                Ok::<MotorIdentity, crate::error::Error>(identity)
            } => Some(r),
        };

        match res {
            Some(Ok(identity)) => {
                let _ = events_tx.send(Cia402Event::Identified { nid, identity });
            }
            Some(Err(e)) => {
                log::warn!("identify nid 0x{nid:02X} failed: {e}");
                let _ = events_tx.send(Cia402Event::IdentifyFailed {
                    nid,
                    reason: e.to_string(),
                });
            }
            None => log::debug!("identify nid 0x{nid:02X} cancelled"),
        }
    });
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
    use std::sync::atomic::{AtomicBool, Ordering};

    use async_trait::async_trait;
    use can_transport::{CanCapabilities, CanFrame, CanId, CanIoError, CanRx};
    use tokio::sync::{mpsc, Notify};

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
    struct BlockingConfigState {
        config_requested: AtomicBool,
        notify: Notify,
    }

    impl BlockingConfigState {
        fn mark_config_requested(&self) {
            self.config_requested.store(true, Ordering::Release);
            self.notify.notify_waiters();
        }

        async fn wait_until_config_requested(&self) {
            loop {
                if self.config_requested.load(Ordering::Acquire) {
                    return;
                }
                self.notify.notified().await;
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
                // Deliberately leave this SDO unanswered. Cancellation must drop the read future.
                return Ok(());
            }

            let value = match (index, subindex) {
                (0x1018, 1) => 0x4859_444Cu32.to_le_bytes().to_vec(),
                (0x1018, 2) => 0xAAAA_0001u32.to_le_bytes().to_vec(),
                (0x1018, 3 | 4) => 0u32.to_le_bytes().to_vec(),
                (0x1008, 0) => b"X".to_vec(),
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
