use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFilter};
use tokio::sync::broadcast;
use tokio::time::MissedTickBehavior;
use tokio_util::sync::CancellationToken;

use crate::canopen::{nmt, sdo};
use crate::error::{Error, Result};
use crate::types::MotorIdentity;

use super::entry::{MotorEntry, OperationRegistry};
use super::events::MeowMotorEvent;
use super::identity::{model_for_identity, MeowMotorModel};
use super::settings::read_from_device;
use super::types::{MeowMotorCanSettingsStatus, MeowMotorLifecycle, MeowMotorReinitReason};

const HEARTBEAT_BASE: u16 = 0x700;
const HEARTBEAT_MASK: u16 = 0x780;

pub(crate) async fn identify_once(
    bus: &(impl CanBus + ?Sized),
    node_id: u8,
    timeout: Duration,
) -> Result<MotorIdentity> {
    if !(1..=127).contains(&node_id) {
        return Err(Error::InvalidNodeId(node_id));
    }
    let timeout = Some(timeout);
    let vendor_id = sdo::upload_u32(bus, node_id, 0x1018, 1, timeout).await?;
    let product_code = sdo::upload_u32(bus, node_id, 0x1018, 2, timeout).await?;
    let revision_number = sdo::upload_u32(bus, node_id, 0x1018, 3, timeout).await?;
    let serial_number = sdo::upload_u32(bus, node_id, 0x1018, 4, timeout).await?;
    let product_name = sdo::upload_string(bus, node_id, 0x1008, 0, timeout)
        .await
        .ok();
    Ok(MotorIdentity {
        node_id,
        vendor_id,
        product_code,
        revision_number,
        serial_number,
        product_name,
    })
}

pub(crate) async fn identify_and_cache(
    bus: &(impl CanBus + ?Sized),
    entry: &Arc<MotorEntry>,
    expected_epoch: u64,
    timeout: Duration,
    events: &broadcast::Sender<MeowMotorEvent>,
) -> Result<Option<MeowMotorModel>> {
    let node_id = entry.node_id;
    let identity = identify_once(bus, node_id, timeout).await?;
    let model = model_for_identity(&identity);

    {
        let mut inner = entry.inner.lock().unwrap();
        if inner.session_epoch != expected_epoch {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: "device session changed during identity read".into(),
            });
        }
        inner.identity = Some(identity.clone());
        inner.model = model;
        inner.can_settings = MeowMotorCanSettingsStatus::Pending;
        inner.lifecycle = if model.is_some() {
            MeowMotorLifecycle::Identified
        } else {
            MeowMotorLifecycle::UnsupportedIdentity
        };
    }

    let settings_status = if model.is_some() {
        match read_from_device(bus, node_id, timeout).await {
            Ok(settings) => MeowMotorCanSettingsStatus::Available(settings),
            Err(error) => MeowMotorCanSettingsStatus::ReadFailed {
                reason: error.to_string(),
            },
        }
    } else {
        MeowMotorCanSettingsStatus::Pending
    };

    let live = {
        let mut inner = entry.inner.lock().unwrap();
        if inner.session_epoch != expected_epoch
            || inner.identity.as_ref() != Some(&identity)
            || inner.model != model
        {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: "device session changed while caching identity".into(),
            });
        }
        inner.can_settings = settings_status;
        inner.build_live_state(Instant::now())
    };
    entry.publish(live);

    match model {
        Some(model) => {
            let _ = events.send(MeowMotorEvent::Identified {
                node_id,
                model,
                identity,
            });
        }
        None => {
            let _ = events.send(MeowMotorEvent::UnsupportedIdentity { node_id, identity });
        }
    }
    Ok(model)
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn run_discovery(
    bus: Arc<dyn CanBus>,
    host_node_id: u8,
    entries: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    operations: OperationRegistry,
    events: broadcast::Sender<MeowMotorEvent>,
    sdo_timeout: Duration,
    auto_identify: bool,
    cancel: CancellationToken,
) {
    let mut receiver = match bus
        .subscribe(CanFilter::standard(HEARTBEAT_BASE, HEARTBEAT_MASK))
        .await
    {
        Ok(receiver) => receiver,
        Err(error) => {
            log::error!("Meow Motor discovery subscription failed: {error}");
            return;
        }
    };

    loop {
        tokio::select! {
            _ = cancel.cancelled() => return,
            result = receiver.recv() => {
                let frame = match result {
                    Ok(frame) => frame,
                    Err(error) => {
                        log::warn!("Meow Motor heartbeat receive failed: {error}");
                        continue;
                    }
                };
                let Some((node_id, state)) = nmt::parse_heartbeat(&frame) else {
                    continue;
                };
                if node_id == host_node_id {
                    continue;
                }
                handle_heartbeat(
                    node_id,
                    state,
                    &entries,
                    &operations,
                    &events,
                    &bus,
                    sdo_timeout,
                    auto_identify,
                    &cancel,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn handle_heartbeat(
    node_id: u8,
    state: nmt::NmtState,
    entries: &Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    operations: &OperationRegistry,
    events: &broadcast::Sender<MeowMotorEvent>,
    bus: &Arc<dyn CanBus>,
    sdo_timeout: Duration,
    auto_identify: bool,
    cancel: &CancellationToken,
) {
    let (entry, appeared) = {
        let mut map = entries.write().unwrap();
        match map.get(&node_id) {
            Some(entry) => (entry.clone(), false),
            None => {
                let entry = Arc::new(MotorEntry::new(node_id));
                map.insert(node_id, entry.clone());
                (entry, true)
            }
        }
    };
    if appeared {
        let _ = events.send(MeowMotorEvent::NodeAppeared { node_id });
    }

    let now = Instant::now();
    let (became_online, needs_reinit, identify_epoch, live) = {
        let mut inner = entry.inner.lock().unwrap();
        if state == nmt::NmtState::BootUp {
            inner.invalidate_session();
        }
        let became_online = !inner.online;
        inner.online = true;
        inner.last_heartbeat = Some(now);
        inner.heartbeat_generation = inner.heartbeat_generation.wrapping_add(1);
        if inner.nmt_state != Some(state) {
            inner.nmt_revision = inner.nmt_revision.wrapping_add(1);
        }
        inner.nmt_state = Some(state);

        let needs_reinit = if matches!(inner.lifecycle, MeowMotorLifecycle::Initialized)
            && state != nmt::NmtState::Operational
        {
            inner.lifecycle = MeowMotorLifecycle::NeedsReinit {
                reason: MeowMotorReinitReason::LeftOperational,
            };
            true
        } else {
            false
        };

        let identify_epoch = if auto_identify
            && matches!(inner.lifecycle, MeowMotorLifecycle::Unknown)
            && !inner.identify_attempted
        {
            inner.identify_attempted = true;
            Some(inner.session_epoch)
        } else {
            None
        };
        (
            became_online,
            needs_reinit,
            identify_epoch,
            inner.build_live_state(now),
        )
    };
    entry.publish(live);
    if became_online {
        let _ = events.send(MeowMotorEvent::NodeOnline { node_id });
    }
    if needs_reinit {
        let _ = events.send(MeowMotorEvent::NeedsReinit {
            node_id,
            reason: MeowMotorReinitReason::LeftOperational,
        });
    }

    if let Some(epoch) = identify_epoch {
        let Ok(guard) = operations.try_acquire(node_id, "automatic identify") else {
            let mut inner = entry.inner.lock().unwrap();
            if inner.session_epoch == epoch
                && matches!(inner.lifecycle, MeowMotorLifecycle::Unknown)
            {
                inner.identify_attempted = false;
            }
            return;
        };
        let bus = bus.clone();
        let entry = entry.clone();
        let events = events.clone();
        let cancel = cancel.clone();
        tokio::spawn(async move {
            let _guard = guard;
            let result = tokio::select! {
                _ = cancel.cancelled() => return,
                result = identify_and_cache(bus.as_ref(), &entry, epoch, sdo_timeout, &events) => result,
            };
            if let Err(error) = result {
                let mut inner = entry.inner.lock().unwrap();
                if inner.session_epoch == epoch
                    && matches!(inner.lifecycle, MeowMotorLifecycle::Unknown)
                {
                    inner.identify_attempted = false;
                }
                drop(inner);
                let _ = events.send(MeowMotorEvent::IdentifyFailed {
                    node_id,
                    reason: error.to_string(),
                });
            }
        });
    }
}

pub(crate) async fn run_liveness_monitor(
    entries: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events: broadcast::Sender<MeowMotorEvent>,
    motor_heartbeat_period: Duration,
    initialized_stale_threshold: Duration,
    cancel: CancellationToken,
) {
    let offline_threshold = (motor_heartbeat_period * 5) / 2;
    let tick_period = initialized_stale_threshold
        .min(offline_threshold)
        .div_f32(2.0)
        .clamp(Duration::from_millis(20), Duration::from_millis(200));
    let mut interval = tokio::time::interval(tick_period);
    interval.set_missed_tick_behavior(MissedTickBehavior::Delay);
    interval.tick().await;

    loop {
        tokio::select! {
            _ = cancel.cancelled() => return,
            _ = interval.tick() => {
                let now = Instant::now();
                let snapshot: Vec<Arc<MotorEntry>> =
                    entries.read().unwrap().values().cloned().collect();
                for entry in snapshot {
                    let (offline, stale, live) = {
                        let mut inner = entry.inner.lock().unwrap();
                        if !inner.online {
                            continue;
                        }
                        // A continuing PDO stream must not hide loss of the node heartbeat.
                        let offline = inner
                            .last_heartbeat
                            .is_none_or(|last| now.saturating_duration_since(last) > offline_threshold);
                        if offline {
                            inner.invalidate_session();
                            (true, false, inner.build_live_state(now))
                        } else {
                            let stale = matches!(inner.lifecycle, MeowMotorLifecycle::Initialized)
                                && (inner.last_tpdo1.is_none_or(|last| {
                                    now.saturating_duration_since(last) > initialized_stale_threshold
                                }) || inner.last_tpdo2.is_none_or(|last| {
                                    now.saturating_duration_since(last) > initialized_stale_threshold
                                }));
                            if stale {
                                inner.lifecycle = MeowMotorLifecycle::NeedsReinit {
                                    reason: MeowMotorReinitReason::TelemetryStale,
                                };
                            }
                            (false, stale, inner.build_live_state(now))
                        }
                    };
                    if offline {
                        let _ = events.send(MeowMotorEvent::NodeOffline { node_id: entry.node_id });
                    }
                    if stale {
                        let _ = events.send(MeowMotorEvent::NeedsReinit {
                            node_id: entry.node_id,
                            reason: MeowMotorReinitReason::TelemetryStale,
                        });
                    }
                    if offline || stale {
                        entry.publish(live);
                    }
                }
            }
        }
    }
}
