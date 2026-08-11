//! [`MeowMotorManager`] asynchronous facade.
//!
//! A process must have one bus-level heartbeat/discovery/SDO owner. When another manager owns
//! those facilities, construct this manager with both `broadcast_heartbeat` and `auto_identify`
//! disabled, then call [`MeowMotorManager::identify`] only from the shared owner's serialized
//! integration path. The per-node SDO registry is intentionally not a cross-manager mutex.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use can_transport::CanBus;
use canopen_sdo::{asynch::AsyncSdoError, SdoAbortCode, SdoError};
use tokio::sync::{broadcast, watch, MutexGuard};
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

use crate::canopen::heartbeat::{build_heartbeat_frame, encode_consumer_heartbeat_entry};
use crate::canopen::nmt::{self, NmtCommand, NmtState};
use crate::canopen::rpdo_config::{build_rpdo_config_writes, RpdoRecipe};
use crate::canopen::sdo;
use crate::canopen::tpdo_config::{build_tpdo_config_writes, SdoWrite, TpdoRecipe};
use crate::cia402::PdoProfile;
use crate::error::{Error, Result};
use crate::types::MotorIdentity;

use super::calibration::{
    read_from_device as read_calibration_from_device, MeowFactoryCalibration,
};
use super::commands::{
    build_clear_error_rearm_sdo_write, build_clear_error_sdo_write,
    build_compressed_mit_setup_sdo_writes, build_disable_sdo_write, build_max_torque_sdo_write,
    build_mode_command_sdo_write, build_profile_limits_sdo_writes, build_target_sdo_writes,
    MeowMotorTarget, MeowProfileLimits,
};
use super::discovery::{identify_and_cache, run_discovery, run_liveness_monitor};
use super::entry::{MotorEntry, OperationRegistry};
use super::events::MeowMotorEvent;
use super::identity::MeowMotorModel;
use super::listener::run_tpdo_listener;
use super::pdo::{
    compressed_mit_rpdo1_recipe, device_tpdo1_recipe, device_tpdo2_recipe, mode_rpdo2_recipe,
};
use super::settings::{build_settings_update_plan, read_from_device, MeowMotorCanSettings};
use super::types::{
    MeowMotorCanSettingsStatus, MeowMotorInfo, MeowMotorLifecycle, MeowMotorLiveState,
    MeowMotorLogic,
};
use super::CompressedMitMapping;

const SDO_WRITE_GAP: Duration = Duration::from_millis(2);
const MODE_COMMAND_SDO_REWRITE_GAP: Duration = Duration::from_millis(5);

/// 一次 clear error 最多触发几次。
///
/// 观察到的失败有两种，重试对两种都有效：`0x4401` 已经停在 `0xFF` 上，导致这次
/// 写入根本不构成跳变；以及固件确实收到了触发但没能清掉 `0x8130` 一类的错误。
const CLEAR_ERROR_ATTEMPTS: usize = 3;

#[derive(Debug, Clone)]
pub struct MeowMotorManagerOptions {
    /// Host heartbeat node ID used in 0x1016 and, when enabled, outbound heartbeats.
    pub heartbeat_node_id: u8,
    pub heartbeat_period: Duration,
    /// Exactly one component on a bus should set this to true.
    pub broadcast_heartbeat: bool,
    /// Disable when a process has a separate shared discovery/SDO owner; callers may then use
    /// [`MeowMotorManager::identify`] explicitly after inventory classification.
    pub auto_identify: bool,
    pub consumer_heartbeat_timeout: Duration,
    pub sdo_timeout: Duration,
    pub nmt_confirm_timeout: Duration,
    pub mode_confirm_timeout: Duration,
    pub motor_heartbeat_period: Duration,
    pub initialized_stale_threshold: Duration,
    pub events_capacity: usize,
}

impl Default for MeowMotorManagerOptions {
    fn default() -> Self {
        Self {
            heartbeat_node_id: 10,
            heartbeat_period: Duration::from_millis(50),
            broadcast_heartbeat: true,
            auto_identify: true,
            consumer_heartbeat_timeout: Duration::from_millis(250),
            sdo_timeout: Duration::from_millis(200),
            nmt_confirm_timeout: Duration::from_secs(2),
            mode_confirm_timeout: Duration::from_secs(1),
            motor_heartbeat_period: Duration::from_millis(500),
            initialized_stale_threshold: Duration::from_millis(200),
            events_capacity: 128,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SharedHostPdoConfig {
    pub host_node_id: u8,
    pub motor_slot: u8,
    pub total_motors: u8,
    pub compressed_mit_mapping: CompressedMitMapping,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeowMotorInitializeOptions {
    pub pdo_profile: PdoProfile,
    /// Configure 0x1016 to consume the manager/external owner's heartbeat.
    pub configure_consumer_heartbeat: bool,
    /// Controller-only shared Host TPDO1/2 layout. GUI initialization normally leaves this None.
    pub shared_host_pdo: Option<SharedHostPdoConfig>,
}

impl MeowMotorInitializeOptions {
    pub const fn new(pdo_profile: PdoProfile) -> Self {
        Self {
            pdo_profile,
            configure_consumer_heartbeat: true,
            shared_host_pdo: None,
        }
    }
}

impl Default for MeowMotorInitializeOptions {
    fn default() -> Self {
        Self::new(PdoProfile::default())
    }
}

pub struct MeowMotorManager {
    bus: Arc<dyn CanBus>,
    options: MeowMotorManagerOptions,
    entries: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    operations: OperationRegistry,
    events: broadcast::Sender<MeowMotorEvent>,
    cancel: CancellationToken,
    tasks: Vec<JoinHandle<()>>,
}

impl std::fmt::Debug for MeowMotorManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MeowMotorManager")
            .field("options", &self.options)
            .field("known_nodes", &self.entries.read().unwrap().len())
            .field("tasks", &self.tasks.len())
            .finish()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct IdentityToken {
    session_epoch: u64,
    vendor_id: u32,
    product_code: u32,
    revision_number: u32,
    serial_number: u32,
}

/// 一次 clear error 触发之后的确认结果。
///
/// 把"没清掉"和"这次操作出了硬错误"分开，是因为只有前者可以重试：`0x4401` 的值
/// 变化触发意味着重试必须先 re-arm，而身份/会话不匹配一类的失败重试只会更糟。
#[derive(Debug)]
enum ClearOutcome {
    Cleared,
    /// 电机答复了，但确认超时，错误仍在。内含带 mode/detail 码的诊断，供最后
    /// 一次尝试也失败时上报。
    NotConfirmed(Error),
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct OperationalToken {
    identity: IdentityToken,
    nmt_revision: u64,
}

struct ModeCommandSdoCompletionGuard<'a> {
    last_completed: MutexGuard<'a, Option<Instant>>,
}

impl Drop for ModeCommandSdoCompletionGuard<'_> {
    fn drop(&mut self) {
        *self.last_completed = Some(Instant::now());
    }
}

impl MeowMotorManager {
    /// Starts discovery, valid-TPDO listening and liveness tasks. When `broadcast_heartbeat` is
    /// false, the caller must provide the single heartbeat producer for the bus.
    pub fn new(bus: Arc<dyn CanBus>, options: MeowMotorManagerOptions) -> Result<Self> {
        validate_options(&options)?;
        let entries = Arc::new(RwLock::new(HashMap::new()));
        let operations = OperationRegistry::new();
        let (events, _) = broadcast::channel(options.events_capacity);
        let cancel = CancellationToken::new();
        let mut tasks = Vec::new();

        if options.broadcast_heartbeat {
            tasks.push(tokio::spawn(run_heartbeat_broadcaster(
                bus.clone(),
                options.heartbeat_node_id,
                options.heartbeat_period,
                cancel.clone(),
            )));
        }
        tasks.push(tokio::spawn(run_discovery(
            bus.clone(),
            options.heartbeat_node_id,
            entries.clone(),
            operations.clone(),
            events.clone(),
            options.sdo_timeout,
            options.auto_identify,
            cancel.clone(),
        )));
        tasks.push(tokio::spawn(run_tpdo_listener(
            bus.clone(),
            entries.clone(),
            events.clone(),
            cancel.clone(),
        )));
        tasks.push(tokio::spawn(run_liveness_monitor(
            entries.clone(),
            events.clone(),
            options.motor_heartbeat_period,
            options.initialized_stale_threshold,
            cancel.clone(),
        )));

        Ok(Self {
            bus,
            options,
            entries,
            operations,
            events,
            cancel,
            tasks,
        })
    }

    pub fn list(&self) -> Vec<MeowMotorInfo> {
        let mut result: Vec<_> = self
            .entries
            .read()
            .unwrap()
            .values()
            .map(|entry| {
                let inner = entry.inner.lock().unwrap();
                MeowMotorInfo {
                    node_id: entry.node_id,
                    session_epoch: inner.session_epoch,
                    identity: inner.identity.clone(),
                    model: inner.model,
                    can_settings: inner.can_settings.clone(),
                    lifecycle: inner.lifecycle.clone(),
                    online: inner.online,
                    nmt_state: inner.nmt_state,
                    logic: inner.logic,
                    peak_torque_nm: inner.peak_torque_nm,
                    mit_kp_kd_factor: inner.mit_kp_kd_factor,
                }
            })
            .collect();
        result.sort_by_key(|info| info.node_id);
        result
    }

    pub fn status(&self, node_id: u8) -> Result<MeowMotorLiveState> {
        Ok(self.entry(node_id)?.status())
    }

    pub fn subscribe_status(&self, node_id: u8) -> Result<watch::Receiver<MeowMotorLiveState>> {
        Ok(self.entry(node_id)?.subscribe())
    }

    /// Full publication stream for recording. Unlike the watch subscription,
    /// this preserves every queued TPDO-derived snapshot or reports lag through
    /// `tokio::sync::broadcast::error::RecvError::Lagged`.
    pub fn subscribe_status_stream(
        &self,
        node_id: u8,
    ) -> Result<broadcast::Receiver<MeowMotorLiveState>> {
        Ok(self.entry(node_id)?.subscribe_stream())
    }

    pub fn subscribe_events(&self) -> broadcast::Receiver<MeowMotorEvent> {
        self.events.subscribe()
    }

    /// Explicit exact identification, useful when another component owns shared discovery.
    pub async fn identify(&self, node_id: u8) -> Result<MeowMotorModel> {
        validate_node_id(node_id)?;
        let entry = self.get_or_insert_entry(node_id);
        let _guard = self.operations.try_acquire(node_id, "identify")?;
        let epoch = entry.inner.lock().unwrap().session_epoch;
        identify_and_cache(
            self.bus.as_ref(),
            &entry,
            epoch,
            self.options.sdo_timeout,
            &self.events,
        )
        .await?
        .ok_or_else(|| Error::DeviceOperationRejected {
            nid: node_id,
            reason: "exact 0x1018 identity is not a supported Meow Motor".into(),
        })
    }

    pub async fn initialize(&self, node_id: u8, pdo_profile: PdoProfile) -> Result<()> {
        self.initialize_with_options(node_id, MeowMotorInitializeOptions::new(pdo_profile))
            .await
    }

    /// Configures Device TPDO1/2 (and optional controller Host RPDO lanes), enters Operational,
    /// waits for fresh valid feedback, and commits only when mode display is still Disabled.
    pub async fn initialize_with_options(
        &self,
        node_id: u8,
        initialize_options: MeowMotorInitializeOptions,
    ) -> Result<()> {
        let entry = self.require_exact_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "initialize")?;
        let token = self.identity_token(&entry)?;
        {
            let mut inner = entry.inner.lock().unwrap();
            if !matches!(
                inner.lifecycle,
                MeowMotorLifecycle::Identified | MeowMotorLifecycle::NeedsReinit { .. }
            ) {
                return Err(Error::NotReady {
                    nid: node_id,
                    lifecycle: format!("{:?}", inner.lifecycle),
                });
            }
            if !matches!(inner.can_settings, MeowMotorCanSettingsStatus::Available(_)) {
                return Err(Error::NotReady {
                    nid: node_id,
                    lifecycle: "0x2001 settings were not read successfully".into(),
                });
            }
            inner.lifecycle = MeowMotorLifecycle::Initializing;
            inner.reset_telemetry();
            let live = inner.build_live_state(Instant::now());
            drop(inner);
            entry.publish(live);
        }

        let result = self
            .run_initialize(&entry, &token, initialize_options)
            .await;
        if result.is_err() {
            let live = {
                let mut inner = entry.inner.lock().unwrap();
                if self.identity_token_matches(&inner, &token) {
                    inner.lifecycle = MeowMotorLifecycle::Identified;
                }
                inner.build_live_state(Instant::now())
            };
            entry.publish(live);
        }
        result
    }

    async fn run_initialize(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
        initialize_options: MeowMotorInitializeOptions,
    ) -> Result<()> {
        let node_id = entry.node_id;
        self.verify_identity_on_wire(entry, token).await?;
        self.send_nmt_and_wait(
            entry,
            NmtCommand::EnterPreOperational,
            NmtState::PreOperational,
        )
        .await?;
        self.ensure_identity_token(entry, token)?;

        self.clear_existing_error_sdo(entry, token).await?;
        self.write_one(node_id, &build_disable_sdo_write()).await?;
        self.wait_sdo_mode_disabled(entry, token).await?;

        self.ensure_tpdo_recipe(
            entry,
            token,
            &device_tpdo1_recipe(node_id, initialize_options.pdo_profile)?,
        )
        .await?;
        self.ensure_tpdo_recipe(entry, token, &device_tpdo2_recipe(node_id)?)
            .await?;

        if let Some(host) = initialize_options.shared_host_pdo {
            for write in build_compressed_mit_setup_sdo_writes(&host.compressed_mit_mapping)
                .map_err(|error| Error::Internal(error.to_string()))?
            {
                self.ensure_identity_token(entry, token)?;
                self.write_one(node_id, &write).await?;
                tokio::time::sleep(SDO_WRITE_GAP).await;
            }
            self.ensure_rpdo_recipe(
                entry,
                token,
                &compressed_mit_rpdo1_recipe(
                    host.motor_slot,
                    host.total_motors,
                    host.host_node_id,
                )?,
            )
            .await?;
            self.ensure_rpdo_recipe(
                entry,
                token,
                &mode_rpdo2_recipe(host.motor_slot, host.total_motors, host.host_node_id)?,
            )
            .await?;
        }

        if initialize_options.configure_consumer_heartbeat {
            let timeout_ms = self
                .options
                .consumer_heartbeat_timeout
                .as_millis()
                .min(u16::MAX as u128) as u16;
            let consumer =
                encode_consumer_heartbeat_entry(self.options.heartbeat_node_id, timeout_ms);
            self.write_one(node_id, &SdoWrite::u32(0x1016, 1, consumer))
                .await?;
        }

        let peak_torque_nm = sdo::upload_f32(
            self.bus.as_ref(),
            node_id,
            0x4576,
            0,
            Some(self.options.sdo_timeout),
        )
        .await
        .ok();
        let mit_kp_kd_factor = sdo::upload_f32(
            self.bus.as_ref(),
            node_id,
            0x4102,
            7,
            Some(self.options.sdo_timeout),
        )
        .await
        .ok();
        self.ensure_identity_token(entry, token)?;
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.peak_torque_nm = peak_torque_nm;
            inner.mit_kp_kd_factor = mit_kp_kd_factor;
        }

        self.send_nmt_and_wait(entry, NmtCommand::StartRemoteNode, NmtState::Operational)
            .await?;
        // Fence after Operational heartbeat confirmation, so Pre-Operational PDOs cannot
        // satisfy initialization readiness.
        let (tpdo1_baseline, tpdo2_baseline) = {
            let inner = entry.inner.lock().unwrap();
            (
                inner.measurements.tpdo1_generation,
                inner.measurements.tpdo2_generation,
            )
        };
        self.wait_for_initial_feedback(entry, token, tpdo1_baseline, tpdo2_baseline)
            .await?;

        let live = {
            let mut inner = entry.inner.lock().unwrap();
            self.ensure_identity_token_inner(node_id, &inner, token)?;
            if inner.nmt_state != Some(NmtState::Operational) {
                return Err(Error::DeviceOperationRejected {
                    nid: node_id,
                    reason: "node left Operational before initialization commit".into(),
                });
            }
            if inner.logic != Some(MeowMotorLogic::Disabled) {
                return Err(Error::DeviceOperationRejected {
                    nid: node_id,
                    reason: format!(
                        "initialization must finish Disabled, observed {:?}",
                        inner.logic
                    ),
                });
            }
            inner.lifecycle = MeowMotorLifecycle::Initialized;
            inner.build_live_state(Instant::now())
        };
        entry.publish(live);
        let _ = self.events.send(MeowMotorEvent::Initialized { node_id });
        Ok(())
    }

    /// Writes a safe target first, then 0x4401, then fences on a newer valid TPDO2.
    pub async fn set_mode_sdo(&self, node_id: u8, target: MeowMotorTarget) -> Result<()> {
        let entry = self.require_initialized_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "set mode")?;
        let token = self.operational_token(&entry)?;
        if matches!(
            entry.inner.lock().unwrap().logic,
            Some(MeowMotorLogic::Error { .. })
        ) {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: "clear the motor error before changing mode".into(),
            });
        }
        let target_writes =
            build_target_sdo_writes(target).map_err(|error| Error::Internal(error.to_string()))?;
        let result = async {
            for write in target_writes {
                self.ensure_operational_token(&entry, &token)?;
                self.write_one(node_id, &write).await?;
                tokio::time::sleep(SDO_WRITE_GAP).await;
            }
            self.ensure_operational_token(&entry, &token)?;
            let baseline = entry.inner.lock().unwrap().measurements.tpdo2_generation;
            self.write_one(node_id, &build_mode_command_sdo_write(target.mode()))
                .await?;
            let command_completed_at = Instant::now();
            self.ensure_operational_token(&entry, &token)?;
            self.wait_for_logic_after(
                &entry,
                &token,
                baseline,
                command_completed_at,
                MeowMotorLogic::Enabled(target.mode()),
                "set mode confirmation",
            )
            .await
        }
        .await;
        if result.is_err() && self.ensure_identity_token(&entry, &token.identity).is_ok() {
            // Preserve the original error; this is a best-effort safety action only.
            let _ = self.write_one(node_id, &build_disable_sdo_write()).await;
        }
        result
    }

    pub async fn disable(&self, node_id: u8) -> Result<()> {
        let entry = self.require_exact_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "disable")?;
        let identity = self.identity_token(&entry)?;
        let operational = {
            let inner = entry.inner.lock().unwrap();
            (inner.nmt_state == Some(NmtState::Operational)
                && matches!(inner.lifecycle, MeowMotorLifecycle::Initialized))
            .then(|| OperationalToken {
                identity: identity.clone(),
                nmt_revision: inner.nmt_revision,
            })
        };
        let baseline = entry.inner.lock().unwrap().measurements.tpdo2_generation;
        self.write_one(node_id, &build_disable_sdo_write()).await?;
        let command_completed_at = Instant::now();
        if let Some(token) = operational {
            self.ensure_operational_token(&entry, &token)?;
            self.wait_for_logic_after(
                &entry,
                &token,
                baseline,
                command_completed_at,
                MeowMotorLogic::Disabled,
                "disable confirmation",
            )
            .await
        } else {
            self.wait_sdo_mode_disabled(&entry, &identity).await
        }
    }

    /// Sends one 0xFF pulse and confirms Disabled. It never restores the previous mode.
    pub async fn clear_error(&self, node_id: u8) -> Result<()> {
        let entry = self.require_exact_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "clear error")?;
        let identity = self.identity_token(&entry)?;
        let operational = {
            let inner = entry.inner.lock().unwrap();
            (inner.nmt_state == Some(NmtState::Operational)
                && matches!(inner.lifecycle, MeowMotorLifecycle::Initialized))
            .then(|| OperationalToken {
                identity: identity.clone(),
                nmt_revision: inner.nmt_revision,
            })
        };
        let mut not_confirmed = None;
        for attempt in 0..CLEAR_ERROR_ATTEMPTS {
            self.ensure_identity_token(&entry, &identity)?;
            // Attempt 0 keeps the historical single write, so a healthy motor
            // never sees a spurious Disable. Every later attempt must re-arm:
            // 0x4401 is still holding the 0xFF that did not take effect.
            let (baseline, command_completed_at) =
                self.trigger_clear_error(&entry, attempt > 0).await?;
            let outcome = if let Some(token) = operational.as_ref() {
                self.ensure_operational_token(&entry, token)?;
                self.wait_for_clear_after(&entry, token, baseline, command_completed_at)
                    .await?
            } else {
                self.wait_sdo_clear_disabled(&entry, &identity).await?
            };
            match outcome {
                ClearOutcome::Cleared => return Ok(()),
                // The motor answered, it just is not clear yet. Keep the most
                // recent diagnosis; it carries the observed mode/detail codes.
                ClearOutcome::NotConfirmed(error) => not_confirmed = Some(error),
            }
        }
        Err(not_confirmed.expect("the attempt loop always runs at least once"))
    }

    /// Fires one clear-error trigger and reports what to confirm against.
    ///
    /// `0x4401` is change-of-value triggered, so writing `0xFF` while it already
    /// holds `0xFF` does nothing at all. `rearm` writes it back to `0` first so
    /// the next `0xFF` is a real transition — without it, "try clearing again"
    /// silently never reaches the firmware.
    ///
    /// Returns the TPDO2 generation baseline captured after the re-arm and the
    /// instant the trigger write completed.
    async fn trigger_clear_error(
        &self,
        entry: &Arc<MotorEntry>,
        rearm: bool,
    ) -> Result<(u64, Instant)> {
        if rearm {
            // `write_one` already spaces consecutive 0x4401 writes by
            // MODE_COMMAND_SDO_REWRITE_GAP, so the transition is not written
            // faster than the device accepts.
            self.write_one(entry.node_id, &build_clear_error_rearm_sdo_write())
                .await?;
        }
        // After the re-arm, so a TPDO published in response to it cannot be
        // mistaken for the trigger's own confirmation.
        let baseline = entry.inner.lock().unwrap().measurements.tpdo2_generation;
        self.write_one(entry.node_id, &build_clear_error_sdo_write())
            .await?;
        Ok((baseline, Instant::now()))
    }

    /// Updates a target without changing mode. The current fresh logic must be Enabled in the
    /// target's exact mode; this never writes 0x4401.
    pub async fn set_target_sdo(&self, node_id: u8, target: MeowMotorTarget) -> Result<()> {
        let entry = self.require_initialized_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "set target")?;
        let token = self.operational_token(&entry)?;
        let observed = entry.inner.lock().unwrap().logic;
        if observed != Some(MeowMotorLogic::Enabled(target.mode())) {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: format!(
                    "target mode {:?} does not match current logic {observed:?}",
                    target.mode()
                ),
            });
        }
        let writes =
            build_target_sdo_writes(target).map_err(|error| Error::Internal(error.to_string()))?;
        let result = async {
            for write in writes {
                self.ensure_operational_token(&entry, &token)?;
                self.write_one(node_id, &write).await?;
                tokio::time::sleep(SDO_WRITE_GAP).await;
            }
            self.ensure_operational_token(&entry, &token)
        }
        .await;
        if result.is_err() && self.ensure_identity_token(&entry, &token.identity).is_ok() {
            // A multi-write MIT target may otherwise leave an enabled motor with mixed fields.
            // Preserve the original error; this is a best-effort safety action only.
            let _ = self.write_one(node_id, &build_disable_sdo_write()).await;
        }
        result
    }

    pub async fn set_max_torque(&self, node_id: u8, torque_permille: u16) -> Result<()> {
        let entry = self.require_initialized_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "set maximum torque")?;
        let token = self.operational_token(&entry)?;
        let write = build_max_torque_sdo_write(torque_permille)
            .map_err(|error| Error::Internal(error.to_string()))?;
        let result = async {
            self.ensure_operational_token(&entry, &token)?;
            self.write_one(node_id, &write).await?;
            self.ensure_operational_token(&entry, &token)?;
            let observed = sdo::upload_u16(
                self.bus.as_ref(),
                node_id,
                write.index,
                write.subindex,
                Some(self.options.sdo_timeout),
            )
            .await?;
            self.ensure_operational_token(&entry, &token)?;
            if observed != torque_permille {
                return Err(Error::DeviceOperationRejected {
                    nid: node_id,
                    reason: format!(
                        "0x{:04X}:{:02X} maximum torque read-back mismatch: expected \
                         {torque_permille}, got {observed}",
                        write.index, write.subindex,
                    ),
                });
            }
            Ok(())
        }
        .await;
        if result.is_err() && self.ensure_identity_token(&entry, &token.identity).is_ok() {
            // An unknown torque ceiling is unsafe even when the write itself was acknowledged.
            // Preserve the original error; this is a best-effort safety action only.
            let _ = self.write_one(node_id, &build_disable_sdo_write()).await;
        }
        result
    }

    /// Sets the shared PP/PV profile limits and accepts the transaction only when every f32
    /// read-back has exactly the requested IEEE-754 bit pattern.
    pub async fn set_profile_limits(&self, node_id: u8, limits: MeowProfileLimits) -> Result<()> {
        let entry = self.require_initialized_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "set profile limits")?;
        let token = self.operational_token(&entry)?;
        let writes = build_profile_limits_sdo_writes(limits)
            .map_err(|error| Error::Internal(error.to_string()))?;
        let expected = [
            limits.velocity_rev_per_s,
            limits.acceleration_rev_per_s2,
            limits.deceleration_rev_per_s2,
        ];

        let result = async {
            for (write, expected) in writes.iter().zip(expected) {
                self.ensure_operational_token(&entry, &token)?;
                self.write_one(node_id, write).await?;
                self.ensure_operational_token(&entry, &token)?;
                let observed = sdo::upload_f32(
                    self.bus.as_ref(),
                    node_id,
                    write.index,
                    write.subindex,
                    Some(self.options.sdo_timeout),
                )
                .await?;
                self.ensure_operational_token(&entry, &token)?;
                if !profile_limit_readback_matches(expected, observed) {
                    return Err(Error::DeviceOperationRejected {
                        nid: node_id,
                        reason: format!(
                            "0x{:04X}:{:02X} profile limit read-back mismatch: expected \
                             {expected} (0x{:08X}), got {observed} (0x{:08X})",
                            write.index,
                            write.subindex,
                            expected.to_bits(),
                            observed.to_bits(),
                        ),
                    });
                }
                tokio::time::sleep(SDO_WRITE_GAP).await;
            }
            Ok(())
        }
        .await;
        if result.is_err() && self.ensure_identity_token(&entry, &token.identity).is_ok() {
            // A partial profile update can otherwise change the behavior of an enabled mode.
            // Preserve the original error; this is a best-effort safety action only.
            let _ = self.write_one(node_id, &build_disable_sdo_write()).await;
        }
        result
    }

    pub async fn read_can_settings(&self, node_id: u8) -> Result<MeowMotorCanSettings> {
        let entry = self.require_exact_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "read CAN settings")?;
        let token = self.identity_token(&entry)?;
        let settings =
            read_from_device(self.bus.as_ref(), node_id, self.options.sdo_timeout).await?;
        self.ensure_identity_token(&entry, &token)?;
        let live = {
            let mut inner = entry.inner.lock().unwrap();
            inner.can_settings = MeowMotorCanSettingsStatus::Available(settings);
            inner.build_live_state(Instant::now())
        };
        entry.publish(live);
        Ok(settings)
    }

    /// 只读读取 `0x4001` 出厂标定区。
    ///
    /// 只检查 v1 布局与 CRC 自洽性：**不验证 8-byte 签发 token**（那需要联网查签发
    /// 账本），也**不把 torque factor 或摩擦力应用到任何命令路径** —— 解出的物理量
    /// 原样返回给调用方，由调用方决定是否补偿。
    ///
    /// "这台电机没有有效出厂标定"是正常结果
    /// [`MeowFactoryCalibration::Missing`]，只有 SDO/传输失败才返回 `Err`。
    pub async fn read_factory_calibration(&self, node_id: u8) -> Result<MeowFactoryCalibration> {
        let entry = self.require_exact_entry(node_id)?;
        let _guard = self
            .operations
            .try_acquire(node_id, "read factory calibration")?;
        let token = self.identity_token(&entry)?;
        let identity = {
            let inner = entry.inner.lock().unwrap();
            inner.identity.clone().ok_or_else(|| Error::NotReady {
                nid: node_id,
                lifecycle: format!("{:?} without identity", inner.lifecycle),
            })?
        };
        let calibration = read_calibration_from_device(
            self.bus.as_ref(),
            node_id,
            &identity,
            self.options.sdo_timeout,
        )
        .await?;
        // The CRC is bound to 0x1018, so a session change mid-read would make the
        // verdict belong to a different motor.
        self.ensure_identity_token(&entry, &token)?;
        Ok(calibration)
    }

    /// Applies 0x2001 fields, reads all four fields back, then saves only 0x1010:04.
    /// Returns whether any RAM field differed. A successful call always requires power-cycle.
    pub async fn apply_can_settings(
        &self,
        node_id: u8,
        target: MeowMotorCanSettings,
    ) -> Result<bool> {
        target
            .validate()
            .map_err(|error| Error::Internal(error.to_string()))?;
        let entry = self.require_exact_entry(node_id)?;
        let _guard = self.operations.try_acquire(node_id, "apply CAN settings")?;
        let token = self.identity_token(&entry)?;
        self.ensure_configuration_state(&entry, &token)?;
        if target.node_id == self.options.heartbeat_node_id {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: "target node ID conflicts with host heartbeat node ID".into(),
            });
        }
        if target.node_id != node_id
            && self
                .entries
                .read()
                .unwrap()
                .get(&target.node_id)
                .is_some_and(|other| other.inner.lock().unwrap().online)
        {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: format!("target node ID {} is already online", target.node_id),
            });
        }
        self.probe_target_node_id_available(node_id, target.node_id)
            .await?;
        self.verify_identity_on_wire(&entry, &token).await?;
        let current =
            read_from_device(self.bus.as_ref(), node_id, self.options.sdo_timeout).await?;
        let plan = build_settings_update_plan(current, target)
            .map_err(|error| Error::Internal(error.to_string()))?;
        for write in &plan.field_writes {
            self.ensure_configuration_state(&entry, &token)?;
            self.write_one(node_id, write).await?;
            tokio::time::sleep(SDO_WRITE_GAP).await;
        }
        self.ensure_configuration_state(&entry, &token)?;
        let read_back =
            read_from_device(self.bus.as_ref(), node_id, self.options.sdo_timeout).await?;
        if read_back != target {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: format!(
                    "0x2001 read-back mismatch: expected {target:?}, got {read_back:?}"
                ),
            });
        }
        self.ensure_configuration_state(&entry, &token)?;
        let save_capability = sdo::upload_u32(
            self.bus.as_ref(),
            node_id,
            0x1010,
            4,
            Some(self.options.sdo_timeout),
        )
        .await?;
        if save_capability != 1 {
            return Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: format!("0x1010:04 must report save capability 1, got {save_capability}"),
            });
        }
        self.ensure_configuration_state(&entry, &token)?;
        self.write_one(node_id, &plan.save_write).await?;
        self.ensure_configuration_state(&entry, &token)?;

        let live = {
            let mut inner = entry.inner.lock().unwrap();
            inner.can_settings = MeowMotorCanSettingsStatus::Available(target);
            inner.lifecycle = MeowMotorLifecycle::NeedsRestart;
            inner.build_live_state(Instant::now())
        };
        entry.publish(live);
        Ok(plan.changed)
    }

    async fn ensure_tpdo_recipe(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
        recipe: &TpdoRecipe,
    ) -> Result<()> {
        if self.tpdo_recipe_matches(entry.node_id, recipe).await? {
            return Ok(());
        }
        for write in build_tpdo_config_writes(recipe)? {
            self.ensure_identity_token(entry, token)?;
            self.write_one(entry.node_id, &write).await?;
            tokio::time::sleep(SDO_WRITE_GAP).await;
        }
        self.ensure_identity_token(entry, token)?;
        if !self.tpdo_recipe_matches(entry.node_id, recipe).await? {
            return Err(Error::DeviceOperationRejected {
                nid: entry.node_id,
                reason: format!("TPDO{} read-back mismatch", recipe.tpdo_index + 1),
            });
        }
        Ok(())
    }

    async fn ensure_rpdo_recipe(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
        recipe: &RpdoRecipe,
    ) -> Result<()> {
        if self.rpdo_recipe_matches(entry.node_id, recipe).await? {
            return Ok(());
        }
        for write in build_rpdo_config_writes(recipe)? {
            self.ensure_identity_token(entry, token)?;
            self.write_one(entry.node_id, &write).await?;
            tokio::time::sleep(SDO_WRITE_GAP).await;
        }
        self.ensure_identity_token(entry, token)?;
        if !self.rpdo_recipe_matches(entry.node_id, recipe).await? {
            return Err(Error::DeviceOperationRejected {
                nid: entry.node_id,
                reason: format!("RPDO{} read-back mismatch", recipe.rpdo_index + 1),
            });
        }
        Ok(())
    }

    async fn tpdo_recipe_matches(&self, node_id: u8, recipe: &TpdoRecipe) -> Result<bool> {
        let timeout = Some(self.options.sdo_timeout);
        let comm_index = 0x1800 + recipe.tpdo_index as u16;
        let map_index = 0x1A00 + recipe.tpdo_index as u16;
        if sdo::upload_u32(self.bus.as_ref(), node_id, comm_index, 1, timeout).await?
            != 0x4000_0000 | recipe.cob_id as u32
        {
            return Ok(false);
        }
        if sdo::upload_u8(self.bus.as_ref(), node_id, map_index, 0, timeout).await?
            != recipe.entries.len() as u8
        {
            return Ok(false);
        }
        for (offset, expected) in recipe.entries.iter().enumerate() {
            if sdo::upload_u32(
                self.bus.as_ref(),
                node_id,
                map_index,
                (offset + 1) as u8,
                timeout,
            )
            .await?
                != expected.packed()
            {
                return Ok(false);
            }
        }
        Ok(
            sdo::upload_u8(self.bus.as_ref(), node_id, comm_index, 2, timeout).await?
                == recipe.comm.transmission_type
                && sdo::upload_u16(self.bus.as_ref(), node_id, comm_index, 3, timeout).await?
                    == recipe.comm.inhibit_time_x100us
                && sdo::upload_u16(self.bus.as_ref(), node_id, comm_index, 5, timeout).await?
                    == recipe.comm.event_timer_ms,
        )
    }

    async fn rpdo_recipe_matches(&self, node_id: u8, recipe: &RpdoRecipe) -> Result<bool> {
        let timeout = Some(self.options.sdo_timeout);
        let comm_index = 0x1400 + recipe.rpdo_index as u16;
        let map_index = 0x1600 + recipe.rpdo_index as u16;
        if sdo::upload_u32(self.bus.as_ref(), node_id, comm_index, 1, timeout).await?
            != recipe.cob_id as u32
            || sdo::upload_u8(self.bus.as_ref(), node_id, comm_index, 2, timeout).await?
                != recipe.transmission_type
            || sdo::upload_u8(self.bus.as_ref(), node_id, map_index, 0, timeout).await?
                != recipe.entries.len() as u8
        {
            return Ok(false);
        }
        for (offset, expected) in recipe.entries.iter().enumerate() {
            if sdo::upload_u32(
                self.bus.as_ref(),
                node_id,
                map_index,
                (offset + 1) as u8,
                timeout,
            )
            .await?
                != expected.packed()
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    async fn verify_identity_on_wire(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
    ) -> Result<()> {
        let timeout = Some(self.options.sdo_timeout);
        let vendor = sdo::upload_u32(self.bus.as_ref(), entry.node_id, 0x1018, 1, timeout).await?;
        let product = sdo::upload_u32(self.bus.as_ref(), entry.node_id, 0x1018, 2, timeout).await?;
        let revision =
            sdo::upload_u32(self.bus.as_ref(), entry.node_id, 0x1018, 3, timeout).await?;
        let serial = sdo::upload_u32(self.bus.as_ref(), entry.node_id, 0x1018, 4, timeout).await?;
        self.ensure_identity_token(entry, token)?;
        if vendor != token.vendor_id
            || product != token.product_code
            || revision != token.revision_number
            || serial != token.serial_number
        {
            return Err(Error::DeviceOperationRejected {
                nid: entry.node_id,
                reason: format!(
                    "identity changed: expected {:08X}/{:08X}/{:08X}/{:08X}, got \
                     {vendor:08X}/{product:08X}/{revision:08X}/{serial:08X}",
                    token.vendor_id, token.product_code, token.revision_number, token.serial_number,
                ),
            });
        }
        Ok(())
    }

    async fn probe_target_node_id_available(
        &self,
        source_node_id: u8,
        target_node_id: u8,
    ) -> Result<()> {
        if source_node_id == target_node_id {
            return Ok(());
        }
        match sdo::upload_u32(
            self.bus.as_ref(),
            target_node_id,
            0x1018,
            1,
            Some(self.options.sdo_timeout),
        )
        .await
        {
            Ok(_) => Err(Error::DeviceOperationRejected {
                nid: source_node_id,
                reason: format!("target node ID {target_node_id} responded and is occupied"),
            }),
            Err(Error::Sdo(AsyncSdoError::Sdo(SdoError::ClientAborted(
                SdoAbortCode::ProtocolTimeout,
            )))) => Ok(()),
            Err(error) => Err(Error::DeviceOperationRejected {
                nid: source_node_id,
                reason: format!(
                    "could not safely prove target node ID {target_node_id} is free: {error}"
                ),
            }),
        }
    }

    async fn send_nmt_and_wait(
        &self,
        entry: &Arc<MotorEntry>,
        command: NmtCommand,
        expected: NmtState,
    ) -> Result<()> {
        let baseline = entry.inner.lock().unwrap().heartbeat_generation;
        self.bus
            .send(nmt::build_nmt_command(command, entry.node_id)?)
            .await?;
        let deadline = Instant::now() + self.options.nmt_confirm_timeout;
        loop {
            let notified = entry.notify.notified();
            {
                let inner = entry.inner.lock().unwrap();
                if inner.heartbeat_generation != baseline && inner.nmt_state == Some(expected) {
                    return Ok(());
                }
            }
            wait_notified(entry.node_id, "NMT confirmation", deadline, notified).await?;
        }
    }

    async fn wait_for_initial_feedback(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
        tpdo1_baseline: u64,
        tpdo2_baseline: u64,
    ) -> Result<()> {
        let deadline = Instant::now() + self.options.mode_confirm_timeout;
        loop {
            let notified = entry.notify.notified();
            {
                let inner = entry.inner.lock().unwrap();
                self.ensure_identity_token_inner(entry.node_id, &inner, token)?;
                if inner.measurements.tpdo1_generation != tpdo1_baseline
                    && inner.measurements.tpdo2_generation != tpdo2_baseline
                {
                    match inner.logic {
                        Some(MeowMotorLogic::Disabled) => return Ok(()),
                        Some(MeowMotorLogic::Error {
                            mode_display,
                            detailed_error,
                        }) => {
                            return Err(Error::DeviceOperationRejected {
                                nid: entry.node_id,
                                reason: format!(
                                    "initial feedback reports error mode=0x{mode_display:02X}, detail=0x{detailed_error:04X}"
                                ),
                            })
                        }
                        other => {
                            return Err(Error::DeviceOperationRejected {
                                nid: entry.node_id,
                                reason: format!("initial feedback is not Disabled: {other:?}"),
                            })
                        }
                    }
                }
            }
            wait_notified(entry.node_id, "initial TPDO1/TPDO2", deadline, notified).await?;
        }
    }

    async fn wait_for_logic_after(
        &self,
        entry: &Arc<MotorEntry>,
        token: &OperationalToken,
        baseline: u64,
        command_completed_at: Instant,
        expected: MeowMotorLogic,
        operation: &'static str,
    ) -> Result<()> {
        let deadline = Instant::now() + self.options.mode_confirm_timeout;
        loop {
            let notified = entry.notify.notified();
            {
                let inner = entry.inner.lock().unwrap();
                self.ensure_operational_token_inner(entry.node_id, &inner, token)?;
                if inner.measurements.tpdo2_generation != baseline
                    && inner
                        .last_tpdo2
                        .is_some_and(|received| received >= command_completed_at)
                {
                    match inner.logic {
                        Some(logic) if logic == expected => return Ok(()),
                        Some(MeowMotorLogic::Error {
                            mode_display,
                            detailed_error,
                        }) => {
                            return Err(Error::DeviceOperationRejected {
                                nid: entry.node_id,
                                reason: format!(
                                    "{operation} entered error mode=0x{mode_display:02X}, detail=0x{detailed_error:04X}"
                                ),
                            })
                        }
                        _ => {}
                    }
                }
            }
            wait_notified(entry.node_id, operation, deadline, notified).await?;
        }
    }

    async fn wait_for_clear_after(
        &self,
        entry: &Arc<MotorEntry>,
        token: &OperationalToken,
        baseline: u64,
        command_completed_at: Instant,
    ) -> Result<ClearOutcome> {
        let deadline = Instant::now() + self.options.mode_confirm_timeout;
        let mut last_observed = None;
        loop {
            let notified = entry.notify.notified();
            {
                let inner = entry.inner.lock().unwrap();
                self.ensure_operational_token_inner(entry.node_id, &inner, token)?;
                if inner.measurements.tpdo2_generation != baseline
                    && inner
                        .last_tpdo2
                        .is_some_and(|received| received >= command_completed_at)
                {
                    if clear_is_confirmed(inner.logic) {
                        return Ok(ClearOutcome::Cleared);
                    }
                    last_observed = inner.logic;
                }
            }

            let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                return Ok(ClearOutcome::NotConfirmed(clear_confirmation_timeout(
                    entry.node_id,
                    last_observed,
                )));
            };
            if tokio::time::timeout(remaining, notified).await.is_err() {
                return Ok(ClearOutcome::NotConfirmed(clear_confirmation_timeout(
                    entry.node_id,
                    last_observed,
                )));
            }
        }
    }

    async fn clear_existing_error_sdo(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
    ) -> Result<()> {
        self.ensure_identity_token(entry, token)?;
        let display = sdo::upload_u8(
            self.bus.as_ref(),
            entry.node_id,
            0x4402,
            0,
            Some(self.options.sdo_timeout),
        )
        .await?;
        self.ensure_identity_token(entry, token)?;
        if !matches!(display, 0xA1..=0xA6 | 0xAF) {
            return Ok(());
        }
        // Same change-of-value hazard as `clear_error`: a stale 0xFF in 0x4401
        // would make this initialization step a silent no-op.
        let mut not_confirmed = None;
        for attempt in 0..CLEAR_ERROR_ATTEMPTS {
            self.ensure_identity_token(entry, token)?;
            self.trigger_clear_error(entry, attempt > 0).await?;
            match self.wait_sdo_clear_disabled(entry, token).await? {
                ClearOutcome::Cleared => return Ok(()),
                ClearOutcome::NotConfirmed(error) => not_confirmed = Some(error),
            }
        }
        Err(not_confirmed.expect("the attempt loop always runs at least once"))
    }

    async fn wait_sdo_mode_disabled(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
    ) -> Result<()> {
        let deadline = Instant::now() + self.options.mode_confirm_timeout;
        loop {
            self.ensure_identity_token(entry, token)?;
            let display = sdo::upload_u8(
                self.bus.as_ref(),
                entry.node_id,
                0x4402,
                0,
                Some(self.options.sdo_timeout),
            )
            .await?;
            self.ensure_identity_token(entry, token)?;
            if display == 0 {
                return Ok(());
            }
            if matches!(display, 0xA1..=0xA6 | 0xAF) {
                let detailed = sdo::upload_u16(
                    self.bus.as_ref(),
                    entry.node_id,
                    0x453F,
                    0,
                    Some(self.options.sdo_timeout),
                )
                .await?;
                return Err(Error::DeviceOperationRejected {
                    nid: entry.node_id,
                    reason: format!(
                        "motor remains in error mode=0x{display:02X}, detail=0x{detailed:04X}"
                    ),
                });
            }
            if Instant::now() >= deadline {
                return Err(Error::DeviceOperationTimeout {
                    nid: entry.node_id,
                    operation: "SDO Disabled confirmation",
                });
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    async fn wait_sdo_clear_disabled(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
    ) -> Result<ClearOutcome> {
        let deadline = Instant::now() + self.options.mode_confirm_timeout;
        loop {
            self.ensure_identity_token(entry, token)?;
            let display = sdo::upload_u8(
                self.bus.as_ref(),
                entry.node_id,
                0x4402,
                0,
                Some(self.options.sdo_timeout),
            )
            .await?;
            self.ensure_identity_token(entry, token)?;
            if display == 0 {
                return Ok(ClearOutcome::Cleared);
            }
            if Instant::now() >= deadline {
                let detailed = if matches!(display, 0xA1..=0xA6 | 0xAF) {
                    sdo::upload_u16(
                        self.bus.as_ref(),
                        entry.node_id,
                        0x453F,
                        0,
                        Some(self.options.sdo_timeout),
                    )
                    .await
                    .ok()
                } else {
                    None
                };
                return Ok(ClearOutcome::NotConfirmed(clear_sdo_confirmation_timeout(
                    entry.node_id,
                    Some(display),
                    detailed,
                )));
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    async fn write_one(&self, node_id: u8, write: &SdoWrite) -> Result<()> {
        if write.index == 0x4401 && write.subindex == 0 {
            let entry = self.entry(node_id)?;
            let last_completed = entry.mode_command_sdo_last_completed.lock().await;
            if let Some(delay) = mode_command_sdo_rewrite_delay(*last_completed, Instant::now()) {
                tokio::time::sleep(delay).await;
            }
            let _completion_guard = ModeCommandSdoCompletionGuard { last_completed };
            return sdo::download(
                self.bus.as_ref(),
                node_id,
                write.index,
                write.subindex,
                &write.data,
                Some(self.options.sdo_timeout),
            )
            .await;
        }

        sdo::download(
            self.bus.as_ref(),
            node_id,
            write.index,
            write.subindex,
            &write.data,
            Some(self.options.sdo_timeout),
        )
        .await
    }

    fn get_or_insert_entry(&self, node_id: u8) -> Arc<MotorEntry> {
        let mut entries = self.entries.write().unwrap();
        entries
            .entry(node_id)
            .or_insert_with(|| Arc::new(MotorEntry::new(node_id)))
            .clone()
    }

    fn entry(&self, node_id: u8) -> Result<Arc<MotorEntry>> {
        self.entries
            .read()
            .unwrap()
            .get(&node_id)
            .cloned()
            .ok_or(Error::UnknownNode(node_id))
    }

    fn require_exact_entry(&self, node_id: u8) -> Result<Arc<MotorEntry>> {
        let entry = self.entry(node_id)?;
        let inner = entry.inner.lock().unwrap();
        if inner.model.is_none() || inner.identity.is_none() {
            return Err(Error::NotReady {
                nid: node_id,
                lifecycle: format!("{:?}", inner.lifecycle),
            });
        }
        if !inner.online {
            return Err(Error::NotReady {
                nid: node_id,
                lifecycle: "exact identity known but node is offline".into(),
            });
        }
        drop(inner);
        Ok(entry)
    }

    fn require_initialized_entry(&self, node_id: u8) -> Result<Arc<MotorEntry>> {
        let entry = self.require_exact_entry(node_id)?;
        let inner = entry.inner.lock().unwrap();
        if !matches!(inner.lifecycle, MeowMotorLifecycle::Initialized) {
            return Err(Error::NotReady {
                nid: node_id,
                lifecycle: format!("{:?}", inner.lifecycle),
            });
        }
        drop(inner);
        Ok(entry)
    }

    fn identity_token(&self, entry: &Arc<MotorEntry>) -> Result<IdentityToken> {
        let inner = entry.inner.lock().unwrap();
        let identity = inner.identity.as_ref().ok_or_else(|| Error::NotReady {
            nid: entry.node_id,
            lifecycle: format!("{:?} without identity", inner.lifecycle),
        })?;
        if inner.model.is_none() {
            return Err(Error::DeviceOperationRejected {
                nid: entry.node_id,
                reason: "identity is not an exact supported Meow Motor tuple".into(),
            });
        }
        Ok(IdentityToken::from_identity(inner.session_epoch, identity))
    }

    fn operational_token(&self, entry: &Arc<MotorEntry>) -> Result<OperationalToken> {
        let identity = self.identity_token(entry)?;
        let inner = entry.inner.lock().unwrap();
        let now = Instant::now();
        let telemetry_fresh =
            telemetry_is_fresh(&inner, self.options.initialized_stale_threshold, now);
        if !runtime_state_is_current(
            &inner,
            inner.nmt_revision,
            self.options.initialized_stale_threshold,
            now,
        ) {
            return Err(Error::NotReady {
                nid: entry.node_id,
                lifecycle: format!(
                    "{:?}, NMT={:?}, online={}, telemetry_fresh={telemetry_fresh}",
                    inner.lifecycle, inner.nmt_state, inner.online
                ),
            });
        }
        Ok(OperationalToken {
            identity,
            nmt_revision: inner.nmt_revision,
        })
    }

    fn ensure_identity_token(&self, entry: &Arc<MotorEntry>, token: &IdentityToken) -> Result<()> {
        let inner = entry.inner.lock().unwrap();
        self.ensure_identity_token_inner(entry.node_id, &inner, token)
    }

    fn ensure_identity_token_inner(
        &self,
        node_id: u8,
        inner: &super::entry::MotorEntryInner,
        token: &IdentityToken,
    ) -> Result<()> {
        if self.identity_token_matches(inner, token) {
            Ok(())
        } else {
            Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason: "device session or exact identity changed during operation".into(),
            })
        }
    }

    fn identity_token_matches(
        &self,
        inner: &super::entry::MotorEntryInner,
        token: &IdentityToken,
    ) -> bool {
        inner.session_epoch == token.session_epoch
            && inner.model.is_some()
            && inner
                .identity
                .as_ref()
                .is_some_and(|identity| token.matches(identity))
    }

    fn ensure_operational_token(
        &self,
        entry: &Arc<MotorEntry>,
        token: &OperationalToken,
    ) -> Result<()> {
        let inner = entry.inner.lock().unwrap();
        self.ensure_operational_token_inner(entry.node_id, &inner, token)
    }

    fn ensure_operational_token_inner(
        &self,
        node_id: u8,
        inner: &super::entry::MotorEntryInner,
        token: &OperationalToken,
    ) -> Result<()> {
        self.ensure_identity_token_inner(node_id, inner, &token.identity)?;
        let now = Instant::now();
        if runtime_state_is_current(
            inner,
            token.nmt_revision,
            self.options.initialized_stale_threshold,
            now,
        ) {
            Ok(())
        } else {
            Err(Error::DeviceOperationRejected {
                nid: node_id,
                reason:
                    "lifecycle, telemetry freshness, or online/NMT state changed during operation"
                        .into(),
            })
        }
    }

    fn ensure_configuration_state(
        &self,
        entry: &Arc<MotorEntry>,
        token: &IdentityToken,
    ) -> Result<()> {
        let inner = entry.inner.lock().unwrap();
        self.ensure_identity_token_inner(entry.node_id, &inner, token)?;
        if !inner.online
            || !matches!(
                inner.nmt_state,
                Some(NmtState::PreOperational | NmtState::Stopped)
            )
        {
            return Err(Error::DeviceOperationRejected {
                nid: entry.node_id,
                reason: "communication settings require an online Pre-Operational or Stopped node"
                    .into(),
            });
        }
        Ok(())
    }
}

impl Drop for MeowMotorManager {
    fn drop(&mut self) {
        self.cancel.cancel();
        for task in &self.tasks {
            task.abort();
        }
    }
}

impl IdentityToken {
    fn from_identity(session_epoch: u64, identity: &MotorIdentity) -> Self {
        Self {
            session_epoch,
            vendor_id: identity.vendor_id,
            product_code: identity.product_code,
            revision_number: identity.revision_number,
            serial_number: identity.serial_number,
        }
    }

    fn matches(&self, identity: &MotorIdentity) -> bool {
        self.vendor_id == identity.vendor_id
            && self.product_code == identity.product_code
            && self.revision_number == identity.revision_number
            && self.serial_number == identity.serial_number
    }
}

fn validate_options(options: &MeowMotorManagerOptions) -> Result<()> {
    validate_node_id(options.heartbeat_node_id)?;
    if options.heartbeat_period.is_zero()
        || options.consumer_heartbeat_timeout.is_zero()
        || options.sdo_timeout.is_zero()
        || options.nmt_confirm_timeout.is_zero()
        || options.mode_confirm_timeout.is_zero()
        || options.motor_heartbeat_period.is_zero()
        || options.initialized_stale_threshold.is_zero()
    {
        return Err(Error::Internal("manager durations must be non-zero".into()));
    }
    if options.events_capacity == 0 {
        return Err(Error::Internal("events_capacity must be non-zero".into()));
    }
    Ok(())
}

fn validate_node_id(node_id: u8) -> Result<()> {
    if (1..=127).contains(&node_id) {
        Ok(())
    } else {
        Err(Error::InvalidNodeId(node_id))
    }
}

fn profile_limit_readback_matches(expected: f32, observed: f32) -> bool {
    expected.to_bits() == observed.to_bits()
}

fn mode_command_sdo_rewrite_delay(
    last_completed: Option<Instant>,
    now: Instant,
) -> Option<Duration> {
    last_completed
        .and_then(|completed| {
            MODE_COMMAND_SDO_REWRITE_GAP.checked_sub(now.saturating_duration_since(completed))
        })
        .filter(|delay| !delay.is_zero())
}

fn clear_confirmation_timeout(node_id: u8, observed: Option<MeowMotorLogic>) -> Error {
    Error::DeviceOperationRejected {
        nid: node_id,
        reason: match observed {
            Some(MeowMotorLogic::Error {
                mode_display,
                detailed_error,
            }) => format!(
                "clear error confirmation timed out waiting for Disabled; mode=0x{mode_display:02X}, detail=0x{detailed_error:04X}"
            ),
            other => format!(
                "clear error confirmation timed out waiting for Disabled; last observed {other:?}"
            ),
        },
    }
}

fn clear_is_confirmed(observed: Option<MeowMotorLogic>) -> bool {
    observed == Some(MeowMotorLogic::Disabled)
}

fn clear_sdo_confirmation_timeout(
    node_id: u8,
    display: Option<u8>,
    detailed: Option<u16>,
) -> Error {
    Error::DeviceOperationRejected {
        nid: node_id,
        reason: match (display, detailed) {
            (Some(display), Some(detailed)) => format!(
                "clear error confirmation timed out; mode=0x{display:02X}, detail=0x{detailed:04X}"
            ),
            (Some(display), None) => {
                format!("clear error confirmation timed out; mode=0x{display:02X}")
            }
            (None, _) => "clear error confirmation timed out without a mode observation".into(),
        },
    }
}

fn telemetry_is_fresh(
    inner: &super::entry::MotorEntryInner,
    stale_threshold: Duration,
    now: Instant,
) -> bool {
    inner
        .last_tpdo1
        .is_some_and(|last| now.saturating_duration_since(last) <= stale_threshold)
        && inner
            .last_tpdo2
            .is_some_and(|last| now.saturating_duration_since(last) <= stale_threshold)
}

fn runtime_state_is_current(
    inner: &super::entry::MotorEntryInner,
    expected_nmt_revision: u64,
    stale_threshold: Duration,
    now: Instant,
) -> bool {
    inner.online
        && matches!(inner.lifecycle, MeowMotorLifecycle::Initialized)
        && inner.nmt_state == Some(NmtState::Operational)
        && inner.nmt_revision == expected_nmt_revision
        && telemetry_is_fresh(inner, stale_threshold, now)
}

async fn run_heartbeat_broadcaster(
    bus: Arc<dyn CanBus>,
    node_id: u8,
    period: Duration,
    cancel: CancellationToken,
) {
    let mut interval = tokio::time::interval(period);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            _ = cancel.cancelled() => return,
            _ = interval.tick() => {
                let frame = match build_heartbeat_frame(node_id, NmtState::Operational) {
                    Ok(frame) => frame,
                    Err(error) => {
                        log::error!("failed to build host heartbeat: {error}");
                        return;
                    }
                };
                if let Err(error) = bus.send(frame).await {
                    log::warn!("host heartbeat send failed: {error}");
                }
            }
        }
    }
}

async fn wait_notified(
    node_id: u8,
    operation: &'static str,
    deadline: Instant,
    notified: impl std::future::Future<Output = ()>,
) -> Result<()> {
    let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
        return Err(Error::DeviceOperationTimeout {
            nid: node_id,
            operation,
        });
    };
    tokio::time::timeout(remaining, notified)
        .await
        .map_err(|_| Error::DeviceOperationTimeout {
            nid: node_id,
            operation,
        })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meow_motor::identity::{PRODUCT_CODE_4310, VENDOR_ID};

    #[test]
    fn default_initialization_is_gui_safe_and_high_rate() {
        let options = MeowMotorInitializeOptions::default();
        assert_eq!(options.pdo_profile, PdoProfile::default());
        assert!(options.configure_consumer_heartbeat);
        assert!(options.shared_host_pdo.is_none());
    }

    /// `0x4401` 只在**值变化**时触发。这条测试重放 `clear_error` 在全部尝试里发出
    /// 的写序列，逐个检查哪些 `0xFF` 真的构成跳变。
    ///
    /// 被复现的现场故障是：`0x4401` 已经停在上一次没生效的 `0xFF` 上，此时再写
    /// `0xFF` 固件什么都不做，用户看到的是"点了清除错误但完全没反应"。
    #[test]
    fn every_retry_is_a_real_0x4401_transition() {
        let trigger = build_clear_error_sdo_write();
        let rearm = build_clear_error_rearm_sdo_write();

        let mut writes = Vec::new();
        for attempt in 0..CLEAR_ERROR_ATTEMPTS {
            // 与 `clear_error` / `clear_existing_error_sdo` 里的条件一致。
            if attempt > 0 {
                writes.push(rearm.clone());
            }
            writes.push(trigger.clone());
        }

        // 数一遍在给定的 0x4401 初值下，有多少次 0xFF 真的产生了跳变。
        let effective_triggers = |initial: &[u8]| {
            let mut on_wire = initial.to_vec();
            let mut fired = 0;
            for write in &writes {
                assert_eq!((write.index, write.subindex), (0x4401, 0));
                if write.data != on_wire {
                    on_wire = write.data.clone();
                    if on_wire == trigger.data {
                        fired += 1;
                    }
                }
            }
            fired
        };

        // 正常情况：0x4401 停在某个 mode command 上，三次尝试都会触发。
        assert_eq!(effective_triggers(&[3]), CLEAR_ERROR_ATTEMPTS);
        // 故障现场：0x4401 已经是 0xFF，第一次写被固件忽略，但 re-arm 让后面
        // 两次仍然能真的触发。修复前这里会是 0。
        assert_eq!(effective_triggers(&trigger.data), CLEAR_ERROR_ATTEMPTS - 1);
        assert!(
            effective_triggers(&trigger.data) > 0,
            "a stale 0xFF must not swallow every retry"
        );
    }

    #[test]
    fn identity_token_includes_physical_identity_and_epoch() {
        let identity = MotorIdentity {
            node_id: 1,
            vendor_id: VENDOR_ID,
            product_code: PRODUCT_CODE_4310,
            revision_number: 2,
            serial_number: 3,
            product_name: None,
        };
        let token = IdentityToken::from_identity(7, &identity);
        assert!(token.matches(&identity));
        assert_eq!(token.session_epoch, 7);
        let mut replacement = identity;
        replacement.serial_number = 4;
        assert!(!token.matches(&replacement));
    }

    #[test]
    fn options_reject_zero_duration_and_bad_host_id() {
        let mut options = MeowMotorManagerOptions::default();
        options.heartbeat_node_id = 0;
        assert!(validate_options(&options).is_err());
        options = MeowMotorManagerOptions::default();
        options.sdo_timeout = Duration::ZERO;
        assert!(validate_options(&options).is_err());
    }

    #[test]
    fn profile_limit_readback_requires_exact_f32_bits() {
        let expected = 0.5_f32;
        assert!(profile_limit_readback_matches(expected, expected));
        assert!(!profile_limit_readback_matches(
            expected,
            f32::from_bits(expected.to_bits() + 1)
        ));
    }

    #[test]
    fn mode_command_sdo_rewrites_wait_five_ms_after_completion() {
        let completed = Instant::now();
        assert_eq!(
            mode_command_sdo_rewrite_delay(Some(completed), completed + Duration::from_millis(4)),
            Some(Duration::from_millis(1))
        );
        assert_eq!(
            mode_command_sdo_rewrite_delay(Some(completed), completed + Duration::from_millis(5)),
            None
        );
        assert_eq!(mode_command_sdo_rewrite_delay(None, completed), None);
    }

    #[test]
    fn clear_timeout_keeps_transitional_error_for_diagnostics() {
        let transitional = Some(MeowMotorLogic::Error {
            mode_display: 0xA4,
            detailed_error: 0,
        });
        assert!(!clear_is_confirmed(transitional));
        assert!(clear_is_confirmed(Some(MeowMotorLogic::Disabled)));
        let error = clear_confirmation_timeout(1, transitional);
        assert!(error.to_string().contains("A4"));
    }

    #[test]
    fn runtime_fence_requires_initialized_and_fresh_tpdo_streams() {
        let entry = MotorEntry::new(1);
        let now = Instant::now();
        let threshold = Duration::from_millis(100);
        let mut inner = entry.inner.lock().unwrap();
        inner.lifecycle = MeowMotorLifecycle::Initialized;
        inner.online = true;
        inner.nmt_state = Some(NmtState::Operational);
        inner.nmt_revision = 4;
        inner.last_tpdo1 = Some(now);
        inner.last_tpdo2 = Some(now);
        assert!(runtime_state_is_current(&inner, 4, threshold, now));

        inner.last_tpdo1 = now.checked_sub(threshold + Duration::from_millis(1));
        assert!(!runtime_state_is_current(&inner, 4, threshold, now));
        inner.last_tpdo1 = Some(now);
        inner.lifecycle = MeowMotorLifecycle::Identified;
        assert!(!runtime_state_is_current(&inner, 4, threshold, now));
    }
}
