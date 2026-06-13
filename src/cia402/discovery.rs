//! 入向心跳监听 + 自动 identify + 在线状态周期监控。

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex as StdMutex, RwLock};
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFilter};
use tokio::sync::broadcast;
use tokio::time::MissedTickBehavior;
use tokio_util::sync::CancellationToken;

use crate::canopen::{nmt, sdo};
use crate::error::{Error, Result};
use crate::types::MotorIdentity;

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

    tokio::spawn(async move {
        let res = tokio::select! {
            _ = cancel.cancelled() => Err(Error::Internal("cancelled".into())),
            r = identify_once(bus.as_ref(), nid, sdo_timeout) => r,
        };

        match res {
            Ok(identity) => {
                {
                    let mut inner = entry.inner.lock().unwrap();
                    inner.identity = Some(identity.clone());
                    // 仅在还是 Unknown 时升级到 Identified（避免覆盖 M3+ 写入的状态）
                    if matches!(inner.lifecycle, MotorLifecycle::Unknown) {
                        inner.lifecycle = MotorLifecycle::Identified;
                    }
                }
                let _ = events_tx.send(Cia402Event::Identified { nid, identity });
            }
            Err(e) => {
                log::warn!("identify nid 0x{nid:02X} failed: {e}");
                let _ = events_tx.send(Cia402Event::IdentifyFailed {
                    nid,
                    reason: e.to_string(),
                });
            }
        }

        inflight.lock().unwrap().remove(&nid);
    });
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
