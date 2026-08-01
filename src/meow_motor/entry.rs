use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use tokio::sync::{broadcast, watch, Mutex as AsyncMutex, Notify};

use crate::canopen::nmt::NmtState;
use crate::error::{Error, Result};
use crate::types::MotorIdentity;

use super::codec::{Tpdo1Frame, Tpdo2Frame};
use super::fixed::SignedQ8_24;
use super::identity::MeowMotorModel;
use super::types::{
    MeowMotorCanSettingsStatus, MeowMotorConnection, MeowMotorLifecycle, MeowMotorLiveState,
    MeowMotorLogic, MeowMotorMeasurements,
};

const STATUS_STREAM_CAPACITY: usize = 8192;

pub(crate) struct MotorEntry {
    pub node_id: u8,
    pub inner: Mutex<MotorEntryInner>,
    snapshot: ArcSwap<MeowMotorLiveState>,
    status_tx: watch::Sender<MeowMotorLiveState>,
    status_stream_tx: broadcast::Sender<MeowMotorLiveState>,
    /// Serializes SDO writes to 0x4401 and records when the previous one completed.
    pub mode_command_sdo_last_completed: AsyncMutex<Option<Instant>>,
    pub notify: Notify,
}

impl std::fmt::Debug for MotorEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MotorEntry")
            .field("node_id", &self.node_id)
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
pub(crate) struct MotorEntryInner {
    pub session_epoch: u64,
    pub heartbeat_generation: u64,
    pub nmt_revision: u64,
    pub identify_attempted: bool,
    pub identity: Option<MotorIdentity>,
    pub model: Option<MeowMotorModel>,
    pub can_settings: MeowMotorCanSettingsStatus,
    pub lifecycle: MeowMotorLifecycle,
    pub online: bool,
    pub last_heartbeat: Option<Instant>,
    pub last_tpdo1: Option<Instant>,
    pub last_tpdo2: Option<Instant>,
    pub nmt_state: Option<NmtState>,
    pub logic: Option<MeowMotorLogic>,
    pub measurements: MeowMotorMeasurements,
    pub peak_torque_nm: Option<f32>,
    pub mit_kp_kd_factor: Option<f32>,
    last_position: Option<SignedQ8_24>,
    last_position_timestamp_us: Option<u16>,
    accumulation_needs_baseline: bool,
}

impl MotorEntry {
    pub fn new(node_id: u8) -> Self {
        let now = Instant::now();
        let initial = MeowMotorLiveState::empty(now);
        let (status_tx, _) = watch::channel(initial.clone());
        let (status_stream_tx, _) = broadcast::channel(STATUS_STREAM_CAPACITY);
        Self {
            node_id,
            inner: Mutex::new(MotorEntryInner {
                session_epoch: 0,
                heartbeat_generation: 0,
                nmt_revision: 0,
                identify_attempted: false,
                identity: None,
                model: None,
                can_settings: MeowMotorCanSettingsStatus::Pending,
                lifecycle: MeowMotorLifecycle::Unknown,
                online: false,
                last_heartbeat: None,
                last_tpdo1: None,
                last_tpdo2: None,
                nmt_state: None,
                logic: None,
                measurements: MeowMotorMeasurements::default(),
                peak_torque_nm: None,
                mit_kp_kd_factor: None,
                last_position: None,
                last_position_timestamp_us: None,
                accumulation_needs_baseline: false,
            }),
            snapshot: ArcSwap::from(Arc::new(initial)),
            status_tx,
            status_stream_tx,
            mode_command_sdo_last_completed: AsyncMutex::new(None),
            notify: Notify::new(),
        }
    }

    pub fn status(&self) -> MeowMotorLiveState {
        (*self.snapshot.load_full()).clone()
    }

    pub fn subscribe(&self) -> watch::Receiver<MeowMotorLiveState> {
        self.status_tx.subscribe()
    }

    pub fn subscribe_stream(&self) -> broadcast::Receiver<MeowMotorLiveState> {
        self.status_stream_tx.subscribe()
    }

    pub fn publish(&self, state: MeowMotorLiveState) {
        self.snapshot.store(Arc::new(state.clone()));
        self.status_tx.send_replace(state.clone());
        let _ = self.status_stream_tx.send(state);
        self.notify.notify_waiters();
    }
}

impl MotorEntryInner {
    pub fn build_live_state(&self, now: Instant) -> MeowMotorLiveState {
        MeowMotorLiveState {
            session_epoch: self.session_epoch,
            nmt_revision: self.nmt_revision,
            lifecycle: self.lifecycle.clone(),
            connection: MeowMotorConnection {
                last_heartbeat: self.last_heartbeat,
                last_tpdo1: self.last_tpdo1,
                last_tpdo2: self.last_tpdo2,
                online: self.online,
                nmt_state: self.nmt_state,
            },
            logic: self.logic,
            measurements: self.measurements.clone(),
            timestamp: now,
        }
    }

    pub fn invalidate_session(&mut self) {
        self.session_epoch = self.session_epoch.wrapping_add(1);
        self.identify_attempted = false;
        self.identity = None;
        self.model = None;
        self.can_settings = MeowMotorCanSettingsStatus::Pending;
        self.lifecycle = MeowMotorLifecycle::Unknown;
        self.online = false;
        self.last_heartbeat = None;
        self.last_tpdo1 = None;
        self.last_tpdo2 = None;
        self.nmt_state = None;
        self.logic = None;
        self.measurements = MeowMotorMeasurements::default();
        self.peak_torque_nm = None;
        self.mit_kp_kd_factor = None;
        self.last_position = None;
        self.last_position_timestamp_us = None;
        self.accumulation_needs_baseline = false;
    }

    pub fn reset_telemetry(&mut self) {
        self.last_tpdo1 = None;
        self.last_tpdo2 = None;
        self.logic = None;
        self.measurements = MeowMotorMeasurements::default();
        self.last_position = None;
        self.last_position_timestamp_us = None;
        self.accumulation_needs_baseline = false;
    }

    pub fn apply_tpdo1(&mut self, frame: Tpdo1Frame, now: Instant) {
        let gap_too_large = self
            .last_tpdo1
            .is_some_and(|last| now.saturating_duration_since(last) > Duration::from_millis(50));
        let timestamp_delta = self
            .last_position_timestamp_us
            .map(|previous| frame.timestamp_us.wrapping_sub(previous));

        let invalidates_existing_baseline = self.last_position.is_some()
            && (gap_too_large || !matches!(timestamp_delta, Some(1..=50_000)));
        let (accumulated, velocity, valid) = if invalidates_existing_baseline {
            self.accumulation_needs_baseline = true;
            self.measurements.position_accumulation_segment = self
                .measurements
                .position_accumulation_segment
                .wrapping_add(1);
            (None, None, false)
        } else if self.accumulation_needs_baseline {
            self.accumulation_needs_baseline = false;
            (Some(frame.position.to_revolutions()), None, true)
        } else {
            match (
                self.last_position,
                self.measurements.accumulated_position_rev,
                timestamp_delta,
            ) {
                (Some(previous), Some(accumulated), Some(dt)) if (1..=50_000).contains(&dt) => {
                    let delta = frame.position.wrapping_delta(previous);
                    (
                        Some(accumulated + delta),
                        Some(delta * 1_000_000.0 / dt as f64),
                        true,
                    )
                }
                _ => (Some(frame.position.to_revolutions()), None, true),
            }
        };

        self.last_tpdo1 = Some(now);
        self.last_position = Some(frame.position);
        self.last_position_timestamp_us = Some(frame.timestamp_us);
        self.measurements.position = Some(frame.position);
        self.measurements.accumulated_position_rev = accumulated;
        self.measurements.position_accumulation_valid = valid;
        self.measurements.velocity_rev_per_s = velocity;
        self.measurements.timestamp_us = Some(frame.timestamp_us);
        self.measurements.torque_permille = Some(frame.torque_permille);
        self.measurements.tpdo1_generation = self.measurements.tpdo1_generation.wrapping_add(1);
    }

    pub fn apply_tpdo2(&mut self, frame: Tpdo2Frame, now: Instant) -> Result<()> {
        let logic = frame
            .logic()
            .map_err(|error| Error::Internal(error.to_string()))?;
        self.last_tpdo2 = Some(now);
        self.logic = Some(logic);
        self.measurements.mode_display = Some(frame.mode_display);
        self.measurements.detailed_error = Some(frame.detailed_error);
        self.measurements.driver_temp_c = Some(frame.driver_temp_c());
        self.measurements.motor_temp_c = Some(frame.motor_temp_c());
        self.measurements.tpdo2_generation = self.measurements.tpdo2_generation.wrapping_add(1);
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub(crate) struct OperationRegistry {
    active: Arc<Mutex<HashSet<u8>>>,
}

impl OperationRegistry {
    pub fn new() -> Self {
        Self {
            active: Arc::new(Mutex::new(HashSet::new())),
        }
    }

    pub fn try_acquire(&self, node_id: u8, operation: &'static str) -> Result<OperationGuard> {
        if !self.active.lock().unwrap().insert(node_id) {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: format!("another SDO operation is already active ({operation})"),
            });
        }
        Ok(OperationGuard {
            active: self.active.clone(),
            node_id,
        })
    }
}

#[derive(Debug)]
pub(crate) struct OperationGuard {
    active: Arc<Mutex<HashSet<u8>>>,
    node_id: u8,
}

impl Drop for OperationGuard {
    fn drop(&mut self) {
        self.active.lock().unwrap().remove(&self.node_id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_accumulator_handles_signed_wrap() {
        let mut inner = MotorEntry::new(1).inner.into_inner().unwrap();
        let now = Instant::now();
        inner.apply_tpdo1(
            Tpdo1Frame {
                position: SignedQ8_24::from_raw(i32::MAX - (1 << 22)),
                timestamp_us: 1_000,
                torque_permille: 0,
            },
            now,
        );
        inner.apply_tpdo1(
            Tpdo1Frame {
                position: SignedQ8_24::from_raw(i32::MIN + (1 << 22)),
                timestamp_us: 2_000,
                torque_permille: 0,
            },
            now + Duration::from_millis(1),
        );
        let accumulated = inner.measurements.accumulated_position_rev.unwrap();
        assert!(accumulated > 128.0);
        assert!(inner.measurements.velocity_rev_per_s.unwrap() > 499.0);
    }

    #[test]
    fn operation_registry_is_per_node() {
        let registry = OperationRegistry::new();
        let first = registry.try_acquire(1, "first").unwrap();
        assert!(registry.try_acquire(1, "second").is_err());
        assert!(registry.try_acquire(2, "other node").is_ok());
        drop(first);
        assert!(registry.try_acquire(1, "retry").is_ok());
    }

    #[tokio::test]
    async fn full_status_stream_preserves_each_published_snapshot() {
        let entry = MotorEntry::new(1);
        let mut stream = entry.subscribe_stream();
        for generation in 1..=3 {
            let mut state = entry.status();
            state.measurements.tpdo1_generation = generation;
            entry.publish(state);
        }
        for generation in 1..=3 {
            assert_eq!(
                stream.recv().await.unwrap().measurements.tpdo1_generation,
                generation
            );
        }
    }

    #[test]
    fn telemetry_gap_marks_accumulation_segment_invalid_before_rebaseline() {
        let mut inner = MotorEntry::new(1).inner.into_inner().unwrap();
        let now = Instant::now();
        inner.apply_tpdo1(
            Tpdo1Frame {
                position: SignedQ8_24::ZERO,
                timestamp_us: 1_000,
                torque_permille: 0,
            },
            now,
        );
        assert!(inner.measurements.position_accumulation_valid);
        inner.apply_tpdo1(
            Tpdo1Frame {
                position: SignedQ8_24::from_revolutions(1.0).unwrap(),
                timestamp_us: 2_000,
                torque_permille: 0,
            },
            now + Duration::from_millis(60),
        );
        assert!(!inner.measurements.position_accumulation_valid);
        assert!(inner.measurements.accumulated_position_rev.is_none());
        assert_eq!(inner.measurements.position_accumulation_segment, 1);

        inner.apply_tpdo1(
            Tpdo1Frame {
                position: SignedQ8_24::from_revolutions(1.1).unwrap(),
                timestamp_us: 3_000,
                torque_permille: 0,
            },
            now + Duration::from_millis(61),
        );
        assert!(inner.measurements.position_accumulation_valid);
        assert_eq!(inner.measurements.position_accumulation_segment, 1);
    }
}
