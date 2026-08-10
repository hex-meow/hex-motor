//! Single-node hardware qualification harness for a Meow Motor.
//!
//! `init` never enables a motion mode. The active actions require the exact product, revision,
//! serial number, and an arming phrase so they cannot be started by accidentally reusing the
//! read-only command line.
//!
//! ```bash
//! cargo run --example meow_motor_bench -- can0 0x01 init 500
//! cargo run --example meow_motor_bench -- can0 0x01 mode-switch 500 \
//!   0x6C64BC78 0x65780001 0x25104409 I_CONFIRM_FREE_SPINNING_AND_POWER_CUTOFF_READY
//! cargo run --example meow_motor_bench -- can0 0x01 pp-pv-boundary 500 \
//!   0x6C64BC78 0x65780001 0x25104409 I_CONFIRM_FREE_SPINNING_AND_POWER_CUTOFF_READY
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use can_transport::CanBus;
use hex_motor::canopen::nmt::{self, NmtCommand, NmtState};
use hex_motor::canopen::sdo;
use hex_motor::cia402::PdoProfile;
use hex_motor::meow_motor::{
    MeowMitTarget, MeowMotorCanSettingsStatus, MeowMotorInfo, MeowMotorInitializeOptions,
    MeowMotorLifecycle, MeowMotorLogic, MeowMotorManager, MeowMotorManagerOptions, MeowMotorMode,
    MeowMotorTarget, MeowProfileLimits, SignedQ8_24,
};

mod common;

const ARMING_PHRASE: &str = "I_CONFIRM_FREE_SPINNING_AND_POWER_CUTOFF_READY";
const SDO_TIMEOUT: Duration = Duration::from_millis(250);
const MAX_TEMPERATURE_C: f32 = 70.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Action {
    Init,
    ModeSwitch,
    PpPvBoundary,
}

impl Action {
    fn parse(value: &str) -> anyhow::Result<Self> {
        match value {
            "init" => Ok(Self::Init),
            "mode-switch" => Ok(Self::ModeSwitch),
            "pp-pv-boundary" => Ok(Self::PpPvBoundary),
            _ => {
                anyhow::bail!("action must be init, mode-switch, or pp-pv-boundary; got {value:?}")
            }
        }
    }

    const fn enables_motion(self) -> bool {
        !matches!(self, Self::Init)
    }
}

#[derive(Debug, Clone, Copy)]
struct ExpectedIdentity {
    product_code: u32,
    revision: u32,
    serial: u32,
}

#[derive(Debug, Clone, Copy)]
struct CleanupFence {
    session_epoch: u64,
    vendor_id: u32,
    product_code: u32,
    revision: u32,
    serial: u32,
}

#[derive(Debug, Clone, Copy)]
struct QualificationFence {
    session_epoch: u64,
    nmt_revision: u64,
    accumulation_segment: u64,
    tpdo1_generation: u64,
}

#[derive(Debug, Clone, Copy)]
enum WrapDirection {
    Negative,
    Positive,
}

#[derive(Debug, Clone, Copy)]
struct PositionWait {
    target: SignedQ8_24,
    fence: QualificationFence,
    baseline_accumulated: f64,
    max_travel_rev: f64,
    max_positive_travel_rev: Option<f64>,
    timeout: Duration,
    label: &'static str,
}

#[derive(Debug, Clone, Copy)]
struct WrapWait {
    direction: WrapDirection,
    fence: QualificationFence,
    baseline_accumulated: f64,
    previous: SignedQ8_24,
    timeout: Duration,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args: Vec<String> = std::env::args().collect();
    let iface = args.get(1).map(String::as_str).unwrap_or("vcan0");
    let node_id = args
        .get(2)
        .map(|value| parse_u8(value))
        .transpose()?
        .unwrap_or(1);
    let action = Action::parse(args.get(3).map(String::as_str).unwrap_or("init"))?;
    let tpdo1_rate = parse_tpdo1_rate(args.get(4).map(String::as_str).unwrap_or("500"))?;
    let expected_identity = parse_active_arguments(action, &args)?;
    if node_id == MeowMotorManagerOptions::default().heartbeat_node_id {
        anyhow::bail!("motor node ID {node_id} conflicts with the bench host heartbeat node");
    }

    let bus = common::open_bus(iface).await?;
    let original_consumer =
        sdo::upload_u32(bus.as_ref(), node_id, 0x1016, 1, Some(SDO_TIMEOUT)).await?;
    let producer_heartbeat_ms =
        sdo::upload_u16(bus.as_ref(), node_id, 0x1017, 0, Some(SDO_TIMEOUT)).await?;
    if producer_heartbeat_ms == 0 {
        anyhow::bail!("0x1017 must provide a non-zero producer heartbeat for qualification");
    }
    let initial_mode = sdo::upload_u8(bus.as_ref(), node_id, 0x4402, 0, Some(SDO_TIMEOUT)).await?;
    let initial_error =
        sdo::upload_u16(bus.as_ref(), node_id, 0x453F, 0, Some(SDO_TIMEOUT)).await?;
    println!(
        "preflight: consumer=0x{original_consumer:08X}, producer={producer_heartbeat_ms} ms, mode=0x{initial_mode:02X}, error=0x{initial_error:04X}"
    );
    if action.enables_motion() && (initial_mode != 0 || initial_error != 0) {
        anyhow::bail!(
            "active qualification requires Disabled with no detailed error; observed mode=0x{initial_mode:02X}, error=0x{initial_error:04X}"
        );
    }
    if action.enables_motion() && original_consumer != 0 {
        anyhow::bail!(
            "active qualification requires an initially disabled 0x1016:01, got 0x{original_consumer:08X}"
        );
    }

    let motor_heartbeat_period = Duration::from_millis(u64::from(producer_heartbeat_ms));
    let discovery_timeout = (motor_heartbeat_period * 3).max(Duration::from_secs(5));
    let nmt_confirm_timeout =
        (motor_heartbeat_period * 2 + Duration::from_millis(500)).max(Duration::from_secs(2));
    let options = MeowMotorManagerOptions {
        broadcast_heartbeat: action.enables_motion(),
        auto_identify: true,
        motor_heartbeat_period,
        nmt_confirm_timeout,
        initialized_stale_threshold: Duration::from_millis(100),
        ..MeowMotorManagerOptions::default()
    };
    let manager = MeowMotorManager::new(bus.clone(), options)?;
    let initialization_started = AtomicBool::new(false);
    let cleanup_fence = Mutex::new(None);

    let run = async {
        let info = wait_for_identification(&manager, node_id, discovery_timeout).await?;
        verify_expected_identity(&info, expected_identity)?;
        verify_full_identity_on_wire(bus.as_ref(), node_id, &info).await?;
        *cleanup_fence.lock().unwrap() = Some(cleanup_fence_from_info(&info)?);
        println!(
            "identified {} at node 0x{node_id:02X}: {:?}",
            info.friendly_name(),
            info.can_settings
        );

        let mut initialize = MeowMotorInitializeOptions::new(tpdo1_rate);
        initialize.configure_consumer_heartbeat = action.enables_motion();
        initialization_started.store(true, Ordering::Release);
        manager.initialize_with_options(node_id, initialize).await?;
        ensure_initialized(&manager, node_id)?;

        match action {
            Action::Init => monitor_initialized(&manager, node_id).await,
            Action::ModeSwitch => run_zero_target_mode_switches(&manager, node_id).await,
            Action::PpPvBoundary => run_pp_pv_boundary(&manager, bus.as_ref(), node_id).await,
        }
    };

    let result = tokio::select! {
        result = run => result,
        signal = tokio::signal::ctrl_c() => {
            match signal {
                Ok(()) => Err(anyhow::anyhow!("qualification interrupted; executing cleanup")),
                Err(error) => Err(anyhow::anyhow!(
                    "Ctrl-C listener failed ({error}); executing cleanup"
                )),
            }
        }
    };
    let cleanup_fence_value = *cleanup_fence.lock().unwrap();
    let cleanup = cleanup(
        &manager,
        bus.as_ref(),
        node_id,
        original_consumer,
        action.enables_motion(),
        initialization_started.load(Ordering::Acquire),
        cleanup_fence_value,
    )
    .await;

    match (result, cleanup) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(error), Ok(())) => Err(error),
        (Ok(()), Err(cleanup_error)) => Err(cleanup_error),
        (Err(error), Err(cleanup_error)) => Err(anyhow::anyhow!(
            "{error}; cleanup also failed: {cleanup_error}"
        )),
    }
}

fn parse_active_arguments(
    action: Action,
    args: &[String],
) -> anyhow::Result<Option<ExpectedIdentity>> {
    if !action.enables_motion() {
        return Ok(None);
    }
    let usage = "active actions require: <expected-product> <expected-revision> <expected-serial> \
                 I_CONFIRM_FREE_SPINNING_AND_POWER_CUTOFF_READY";
    let product_code = args.get(5).ok_or_else(|| anyhow::anyhow!(usage))?;
    let revision = args.get(6).ok_or_else(|| anyhow::anyhow!(usage))?;
    let serial = args.get(7).ok_or_else(|| anyhow::anyhow!(usage))?;
    let phrase = args.get(8).ok_or_else(|| anyhow::anyhow!(usage))?;
    if phrase != ARMING_PHRASE {
        anyhow::bail!("arming phrase mismatch; {usage}");
    }
    Ok(Some(ExpectedIdentity {
        product_code: parse_u32(product_code)?,
        revision: parse_u32(revision)?,
        serial: parse_u32(serial)?,
    }))
}

fn verify_expected_identity(
    info: &MeowMotorInfo,
    expected: Option<ExpectedIdentity>,
) -> anyhow::Result<()> {
    let Some(expected) = expected else {
        return Ok(());
    };
    let identity = info
        .identity
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("identified entry has no full 0x1018 identity"))?;
    if identity.product_code != expected.product_code
        || identity.revision_number != expected.revision
        || identity.serial_number != expected.serial
    {
        anyhow::bail!(
            "identity mismatch: expected product/revision/serial {:08X}/{:08X}/{:08X}, got {:08X}/{:08X}/{:08X}",
            expected.product_code,
            expected.revision,
            expected.serial,
            identity.product_code,
            identity.revision_number,
            identity.serial_number,
        );
    }
    Ok(())
}

fn cleanup_fence_from_info(info: &MeowMotorInfo) -> anyhow::Result<CleanupFence> {
    let identity = info
        .identity
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("identified entry has no full 0x1018 identity"))?;
    Ok(CleanupFence {
        session_epoch: info.session_epoch,
        vendor_id: identity.vendor_id,
        product_code: identity.product_code,
        revision: identity.revision_number,
        serial: identity.serial_number,
    })
}

async fn verify_full_identity_on_wire(
    bus: &(dyn CanBus + 'static),
    node_id: u8,
    info: &MeowMotorInfo,
) -> anyhow::Result<()> {
    let expected = info
        .identity
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("identified entry has no full 0x1018 identity"))?;
    let observed = (
        sdo::upload_u32(bus, node_id, 0x1018, 1, Some(SDO_TIMEOUT)).await?,
        sdo::upload_u32(bus, node_id, 0x1018, 2, Some(SDO_TIMEOUT)).await?,
        sdo::upload_u32(bus, node_id, 0x1018, 3, Some(SDO_TIMEOUT)).await?,
        sdo::upload_u32(bus, node_id, 0x1018, 4, Some(SDO_TIMEOUT)).await?,
    );
    let expected_tuple = (
        expected.vendor_id,
        expected.product_code,
        expected.revision_number,
        expected.serial_number,
    );
    if observed != expected_tuple {
        anyhow::bail!(
            "0x1018 changed before initialization: expected {expected_tuple:08X?}, got {observed:08X?}"
        );
    }
    Ok(())
}

async fn wait_for_identification(
    manager: &MeowMotorManager,
    node_id: u8,
    timeout: Duration,
) -> anyhow::Result<MeowMotorInfo> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(info) = manager
            .list()
            .into_iter()
            .find(|info| info.node_id == node_id)
        {
            if matches!(info.lifecycle, MeowMotorLifecycle::UnsupportedIdentity) {
                anyhow::bail!("node 0x{node_id:02X} has an unsupported exact identity");
            }
            if matches!(info.lifecycle, MeowMotorLifecycle::Identified)
                && matches!(info.can_settings, MeowMotorCanSettingsStatus::Available(_))
            {
                return Ok(info);
            }
        }
        if Instant::now() >= deadline {
            anyhow::bail!(
                "timed out waiting for exact identity and 0x2001 at node 0x{node_id:02X}"
            );
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

fn ensure_initialized(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<()> {
    let state = manager.status(node_id)?;
    if !matches!(state.lifecycle, MeowMotorLifecycle::Initialized)
        || !state.connection.online
        || state.connection.nmt_state != Some(NmtState::Operational)
        || state.logic != Some(MeowMotorLogic::Disabled)
    {
        anyhow::bail!(
            "initialization did not commit a fresh Operational/Disabled state: lifecycle={:?}, online={}, NMT={:?}, logic={:?}",
            state.lifecycle,
            state.connection.online,
            state.connection.nmt_state,
            state.logic,
        );
    }
    Ok(())
}

async fn monitor_initialized(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<()> {
    let start = manager.status(node_id)?;
    let deadline = Instant::now() + Duration::from_secs(1);
    while Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(100)).await;
        ensure_initialized(manager, node_id)?;
    }
    let end = manager.status(node_id)?;
    if end.measurements.tpdo1_generation <= start.measurements.tpdo1_generation
        || end.measurements.tpdo2_generation <= start.measurements.tpdo2_generation
    {
        anyhow::bail!("TPDO generations did not advance during the one-second observation");
    }
    println!(
        "non-motion init passed: TPDO1 +{}, TPDO2 +{}, position={:?}, velocity={:?}",
        end.measurements.tpdo1_generation - start.measurements.tpdo1_generation,
        end.measurements.tpdo2_generation - start.measurements.tpdo2_generation,
        end.measurements.position.map(SignedQ8_24::to_revolutions),
        end.measurements.velocity_rev_per_s,
    );
    Ok(())
}

async fn run_zero_target_mode_switches(
    manager: &MeowMotorManager,
    node_id: u8,
) -> anyhow::Result<()> {
    manager.set_max_torque(node_id, 20).await?;
    manager
        .set_profile_limits(
            node_id,
            MeowProfileLimits {
                velocity_rev_per_s: 0.2,
                acceleration_rev_per_s2: 0.2,
                deceleration_rev_per_s2: 0.2,
            },
        )
        .await?;
    let start = manager.status(node_id)?;
    let current = start
        .measurements
        .position
        .ok_or_else(|| anyhow::anyhow!("no position feedback after initialization"))?;
    let start_accumulated = start
        .measurements
        .accumulated_position_rev
        .ok_or_else(|| anyhow::anyhow!("no accumulated position after initialization"))?;

    let sequence = [
        MeowMotorTarget::ProfilePosition { position: current },
        MeowMotorTarget::ProfileVelocity {
            velocity_rev_per_s: 0.0,
        },
        MeowMotorTarget::Torque { torque_permille: 0 },
        MeowMotorTarget::Mit(MeowMitTarget {
            position_rev: 0.0,
            velocity_rev_per_s: 0.0,
            torque_nm: 0.0,
            kp: 0,
            kd: 0,
            kp_kd_limit_permille: 0,
        }),
        MeowMotorTarget::ProfilePosition { position: current },
    ];
    let mode_fence = capture_fence(manager, node_id)?;
    for target in sequence {
        manager.set_mode_sdo(node_id, target).await?;
        tokio::time::sleep(Duration::from_millis(200)).await;
        let state = manager.status(node_id)?;
        validate_live_state(&state, Some(mode_fence), Some(target.mode()))?;
        if state.logic != Some(MeowMotorLogic::Enabled(target.mode())) {
            anyhow::bail!(
                "mode switch did not remain in {:?}: {:?}",
                target.mode(),
                state.logic
            );
        }
        let accumulated = state
            .measurements
            .accumulated_position_rev
            .ok_or_else(|| anyhow::anyhow!("accumulated position disappeared"))?;
        if (accumulated - start_accumulated).abs() > 0.05 {
            anyhow::bail!("zero-target mode sequence moved more than 0.05 Rev");
        }
        if state
            .measurements
            .torque_permille
            .unwrap_or_default()
            .unsigned_abs()
            > 50
        {
            anyhow::bail!("zero-target mode sequence exceeded 50 permille actual torque");
        }
        println!(
            "on-the-fly mode {:?} confirmed with a safe target",
            target.mode()
        );
    }
    Ok(())
}

async fn run_pp_pv_boundary(
    manager: &MeowMotorManager,
    bus: &(dyn CanBus + 'static),
    node_id: u8,
) -> anyhow::Result<()> {
    manager.set_max_torque(node_id, 50).await?;
    let slow = MeowProfileLimits {
        velocity_rev_per_s: 0.2,
        acceleration_rev_per_s2: 0.2,
        deceleration_rev_per_s2: 0.2,
    };
    manager.set_profile_limits(node_id, slow).await?;

    let initial = manager.status(node_id)?;
    validate_live_state(&initial, None, None)?;
    let initial_position = initial
        .measurements
        .position
        .ok_or_else(|| anyhow::anyhow!("no initial position feedback"))?;
    let initial_rev = initial_position.to_revolutions();
    if !(-0.5..0.5).contains(&initial_rev) {
        anyhow::bail!(
            "PP/PV boundary qualification requires the documented power-on position window [-0.5, 0.5), got {initial_rev:.9} Rev"
        );
    }
    manager
        .set_mode_sdo(
            node_id,
            MeowMotorTarget::ProfilePosition {
                position: initial_position,
            },
        )
        .await?;

    let jog_rev = if initial_rev > -127.75 {
        initial_rev - 0.1
    } else {
        initial_rev + 0.1
    };
    let jog = SignedQ8_24::from_revolutions(jog_rev)?;
    let jog_fence = capture_fence(manager, node_id)?;
    let jog_baseline = accumulated_position(manager, node_id)?;
    manager
        .set_target_sdo(node_id, MeowMotorTarget::ProfilePosition { position: jog })
        .await?;
    wait_at_position(
        manager,
        node_id,
        PositionWait {
            target: jog,
            fence: jog_fence,
            baseline_accumulated: jog_baseline,
            max_travel_rev: 0.25,
            max_positive_travel_rev: None,
            timeout: Duration::from_secs(15),
            label: "small PP jog",
        },
    )
    .await?;
    manager.disable(node_id).await?;
    println!("small PP jog and explicit Disabled stop passed");

    let restart_position = manager
        .status(node_id)?
        .measurements
        .position
        .ok_or_else(|| anyhow::anyhow!("position disappeared after jog"))?;
    manager
        .set_profile_limits(
            node_id,
            MeowProfileLimits {
                velocity_rev_per_s: 4.0,
                acceleration_rev_per_s2: 2.0,
                deceleration_rev_per_s2: 2.0,
            },
        )
        .await?;
    manager
        .set_mode_sdo(
            node_id,
            MeowMotorTarget::ProfilePosition {
                position: restart_position,
            },
        )
        .await?;

    let near_boundary = SignedQ8_24::from_revolutions(-127.5)?;
    let approach_fence = capture_fence(manager, node_id)?;
    let approach_baseline = accumulated_position(manager, node_id)?;
    manager
        .set_target_sdo(
            node_id,
            MeowMotorTarget::ProfilePosition {
                position: near_boundary,
            },
        )
        .await?;
    verify_position_target(bus, node_id, near_boundary).await?;
    let observed_approach_delta = wait_at_position(
        manager,
        node_id,
        PositionWait {
            target: near_boundary,
            fence: approach_fence,
            baseline_accumulated: approach_baseline,
            max_travel_rev: 140.0,
            max_positive_travel_rev: Some(0.1),
            timeout: Duration::from_secs(90),
            label: "PP approach to -127.5",
        },
    )
    .await?;
    let expected_approach_delta = near_boundary.wrapping_delta(restart_position);
    if observed_approach_delta >= 0.0
        || (observed_approach_delta - expected_approach_delta).abs() > 0.25
    {
        anyhow::bail!(
            "PP approach took an unexpected route: expected about {expected_approach_delta:.6} Rev, observed {observed_approach_delta:.6} Rev"
        );
    }

    manager.set_profile_limits(node_id, slow).await?;
    let minimum = SignedQ8_24::MIN;
    let boundary_fence = capture_fence(manager, node_id)?;
    let boundary_baseline = accumulated_position(manager, node_id)?;
    manager
        .set_target_sdo(
            node_id,
            MeowMotorTarget::ProfilePosition { position: minimum },
        )
        .await?;
    verify_position_target(bus, node_id, minimum).await?;
    let boundary_delta = wait_at_position(
        manager,
        node_id,
        PositionWait {
            target: minimum,
            fence: boundary_fence,
            baseline_accumulated: boundary_baseline,
            max_travel_rev: 0.75,
            max_positive_travel_rev: Some(0.1),
            timeout: Duration::from_secs(15),
            label: "PP -127.5 to -128",
        },
    )
    .await?;
    if !(-0.75..=-0.25).contains(&boundary_delta) {
        anyhow::bail!(
            "PP endpoint did not take the signed short path: accumulated delta {boundary_delta:.6} Rev"
        );
    }
    println!(
        "PP endpoint PASS: accumulated delta {boundary_delta:.6} Rev, consistent with the short signed path"
    );

    manager
        .set_mode_sdo(
            node_id,
            MeowMotorTarget::ProfileVelocity {
                velocity_rev_per_s: 0.0,
            },
        )
        .await?;
    println!("on-the-fly PP -> PV with zero target confirmed");

    let negative_wrap = capture_wrap_wait(
        manager,
        node_id,
        WrapDirection::Negative,
        Duration::from_secs(10),
    )?;
    manager
        .set_target_sdo(
            node_id,
            MeowMotorTarget::ProfileVelocity {
                velocity_rev_per_s: -0.1,
            },
        )
        .await?;
    wait_for_wrap(manager, node_id, negative_wrap).await?;
    stop_profile_velocity(manager, node_id).await?;

    let positive_wrap = capture_wrap_wait(
        manager,
        node_id,
        WrapDirection::Positive,
        Duration::from_secs(10),
    )?;
    manager
        .set_target_sdo(
            node_id,
            MeowMotorTarget::ProfileVelocity {
                velocity_rev_per_s: 0.1,
            },
        )
        .await?;
    wait_for_wrap(manager, node_id, positive_wrap).await?;
    stop_profile_velocity(manager, node_id).await?;
    println!("PV two-way signed Q8.24 wrap PASS");
    Ok(())
}

async fn verify_position_target(
    bus: &(dyn CanBus + 'static),
    node_id: u8,
    expected: SignedQ8_24,
) -> anyhow::Result<()> {
    let observed = sdo::upload_u32(bus, node_id, 0x457A, 0, Some(SDO_TIMEOUT)).await? as i32;
    if observed != expected.raw() {
        anyhow::bail!(
            "0x457A read-back mismatch: expected 0x{:08X}, got 0x{:08X}",
            expected.raw() as u32,
            observed as u32,
        );
    }
    Ok(())
}

async fn wait_at_position(
    manager: &MeowMotorManager,
    node_id: u8,
    wait: PositionWait,
) -> anyhow::Result<f64> {
    let deadline = Instant::now() + wait.timeout;
    let mut stable_samples = 0u8;
    let mut next_log = Instant::now();
    let mut last_generation = wait.fence.tpdo1_generation;
    loop {
        tokio::time::sleep(Duration::from_millis(20)).await;
        let state = manager.status(node_id)?;
        validate_live_state(
            &state,
            Some(wait.fence),
            Some(MeowMotorMode::ProfilePosition),
        )?;
        if state.measurements.tpdo1_generation <= last_generation {
            if Instant::now() >= deadline {
                anyhow::bail!("{}: timed out after {:?}", wait.label, wait.timeout);
            }
            continue;
        }
        last_generation = state.measurements.tpdo1_generation;
        let position = state
            .measurements
            .position
            .ok_or_else(|| anyhow::anyhow!("{}: position disappeared", wait.label))?;
        let accumulated = state
            .measurements
            .accumulated_position_rev
            .ok_or_else(|| anyhow::anyhow!("{}: accumulated position disappeared", wait.label))?;
        let velocity = state
            .measurements
            .velocity_rev_per_s
            .ok_or_else(|| anyhow::anyhow!("{}: velocity disappeared", wait.label))?;
        let travel = accumulated - wait.baseline_accumulated;
        if travel.abs() > wait.max_travel_rev {
            anyhow::bail!(
                "{}: watchdog exceeded {:.3} Rev, observed {travel:.6} Rev",
                wait.label,
                wait.max_travel_rev,
            );
        }
        if let Some(maximum) = wait.max_positive_travel_rev {
            if travel > maximum {
                anyhow::bail!(
                    "{}: unexpected positive route exceeded {maximum:.3} Rev, observed {travel:.6} Rev",
                    wait.label,
                );
            }
        }
        let error = wait.target.wrapping_delta(position).abs();
        if error <= 0.01 && velocity.abs() <= 0.02 {
            stable_samples = stable_samples.saturating_add(1);
            if stable_samples >= 10 {
                return Ok(travel);
            }
        } else {
            stable_samples = 0;
        }
        if Instant::now() >= next_log {
            println!(
                "{}: raw={:08X}, position={:.6}, velocity={velocity:.6}, travel={travel:.6}, error={error:.6}",
                wait.label,
                position.raw() as u32,
                position.to_revolutions(),
            );
            next_log = Instant::now() + Duration::from_millis(500);
        }
        if Instant::now() >= deadline {
            anyhow::bail!("{}: timed out after {:?}", wait.label, wait.timeout);
        }
    }
}

async fn wait_for_wrap(
    manager: &MeowMotorManager,
    node_id: u8,
    wait: WrapWait,
) -> anyhow::Result<()> {
    let mut previous = wait.previous;
    let mut last_generation = wait.fence.tpdo1_generation;
    let mut crossing = None;
    let mut confirmation_samples = 0u8;
    let deadline = Instant::now() + wait.timeout;
    loop {
        tokio::time::sleep(Duration::from_millis(5)).await;
        let state = manager.status(node_id)?;
        validate_live_state(
            &state,
            Some(wait.fence),
            Some(MeowMotorMode::ProfileVelocity),
        )?;
        if state.measurements.tpdo1_generation <= last_generation {
            if Instant::now() >= deadline {
                anyhow::bail!(
                    "PV {:?} wrap timed out after {:?}",
                    wait.direction,
                    wait.timeout
                );
            }
            continue;
        }
        last_generation = state.measurements.tpdo1_generation;
        let current = state
            .measurements
            .position
            .ok_or_else(|| anyhow::anyhow!("PV wrap position disappeared"))?;
        let accumulated = state
            .measurements
            .accumulated_position_rev
            .ok_or_else(|| anyhow::anyhow!("PV wrap accumulated position disappeared"))?;
        let travel = accumulated - wait.baseline_accumulated;
        if travel.abs() > 0.5 {
            anyhow::bail!("PV wrap watchdog exceeded 0.5 Rev, observed {travel:.6} Rev");
        }
        let expected_travel = current.wrapping_delta(wait.previous);
        if (travel - expected_travel).abs() > 1e-6 {
            anyhow::bail!(
                "PV accumulated position was discontinuous: expected {expected_travel:.9} Rev from raw endpoints, got {travel:.9} Rev"
            );
        }
        let crossed = match wait.direction {
            WrapDirection::Negative => previous.raw() < -0x7000_0000 && current.raw() > 0x7000_0000,
            WrapDirection::Positive => previous.raw() > 0x7000_0000 && current.raw() < -0x7000_0000,
        };
        let step = current.wrapping_delta(previous);
        if let Some((from, to, crossing_travel)) = crossing {
            if !wrap_step_is_plausible(wait.direction, step) {
                anyhow::bail!(
                    "PV {:?} post-wrap step was not a small same-direction delta: {step:.9} Rev",
                    wait.direction,
                );
            }
            confirmation_samples = confirmation_samples.saturating_add(1);
            if confirmation_samples >= 2 {
                println!(
                    "PV {:?} wrap: {:08X} -> {:08X}, continuous delta {crossing_travel:.9} Rev",
                    wait.direction, from, to,
                );
                return Ok(());
            }
        }
        if crossed {
            let sign_is_correct = match wait.direction {
                WrapDirection::Negative => travel < 0.0,
                WrapDirection::Positive => travel > 0.0,
            };
            if !sign_is_correct || !wrap_step_is_plausible(wait.direction, step) {
                anyhow::bail!(
                    "PV {:?} raw wrap had inconsistent accumulated delta {travel:.9} Rev or step {step:.9} Rev",
                    wait.direction,
                );
            }
            crossing = Some((previous.raw() as u32, current.raw() as u32, travel));
        }
        previous = current;
        if Instant::now() >= deadline {
            anyhow::bail!(
                "PV {:?} wrap timed out after {:?}",
                wait.direction,
                wait.timeout
            );
        }
    }
}

fn wrap_step_is_plausible(direction: WrapDirection, step_rev: f64) -> bool {
    step_rev.abs() <= 0.01
        && match direction {
            WrapDirection::Negative => step_rev < 0.0,
            WrapDirection::Positive => step_rev > 0.0,
        }
}

fn capture_wrap_wait(
    manager: &MeowMotorManager,
    node_id: u8,
    direction: WrapDirection,
    timeout: Duration,
) -> anyhow::Result<WrapWait> {
    let state = manager.status(node_id)?;
    validate_live_state(&state, None, Some(MeowMotorMode::ProfileVelocity))?;
    Ok(WrapWait {
        direction,
        fence: QualificationFence {
            session_epoch: state.session_epoch,
            nmt_revision: state.nmt_revision,
            accumulation_segment: state.measurements.position_accumulation_segment,
            tpdo1_generation: state.measurements.tpdo1_generation,
        },
        baseline_accumulated: state
            .measurements
            .accumulated_position_rev
            .ok_or_else(|| anyhow::anyhow!("PV wrap has no accumulated position"))?,
        previous: state
            .measurements
            .position
            .ok_or_else(|| anyhow::anyhow!("PV wrap has no starting position"))?,
        timeout,
    })
}

async fn stop_profile_velocity(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<()> {
    let start = manager.status(node_id)?;
    validate_live_state(&start, None, Some(MeowMotorMode::ProfileVelocity))?;
    let fence = QualificationFence {
        session_epoch: start.session_epoch,
        nmt_revision: start.nmt_revision,
        accumulation_segment: start.measurements.position_accumulation_segment,
        tpdo1_generation: start.measurements.tpdo1_generation,
    };
    let baseline_accumulated = start
        .measurements
        .accumulated_position_rev
        .ok_or_else(|| anyhow::anyhow!("PV stop has no accumulated position"))?;
    manager
        .set_target_sdo(
            node_id,
            MeowMotorTarget::ProfileVelocity {
                velocity_rev_per_s: 0.0,
            },
        )
        .await?;
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut stable_samples = 0u8;
    let mut last_generation = fence.tpdo1_generation;
    loop {
        tokio::time::sleep(Duration::from_millis(20)).await;
        let state = manager.status(node_id)?;
        validate_live_state(&state, Some(fence), Some(MeowMotorMode::ProfileVelocity))?;
        if state.measurements.tpdo1_generation <= last_generation {
            if Instant::now() >= deadline {
                anyhow::bail!("PV zero target did not settle within 10 seconds");
            }
            continue;
        }
        last_generation = state.measurements.tpdo1_generation;
        let accumulated = state
            .measurements
            .accumulated_position_rev
            .ok_or_else(|| anyhow::anyhow!("PV stop accumulated position disappeared"))?;
        if (accumulated - baseline_accumulated).abs() > 0.1 {
            anyhow::bail!("PV zero-target stopping distance exceeded 0.1 Rev");
        }
        if state
            .measurements
            .velocity_rev_per_s
            .is_some_and(|velocity| velocity.abs() <= 0.02)
        {
            stable_samples = stable_samples.saturating_add(1);
            if stable_samples >= 10 {
                return Ok(());
            }
        } else {
            stable_samples = 0;
        }
        if Instant::now() >= deadline {
            anyhow::bail!("PV zero target did not settle within 10 seconds");
        }
    }
}

fn capture_fence(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<QualificationFence> {
    let state = manager.status(node_id)?;
    validate_live_state(&state, None, None)?;
    Ok(QualificationFence {
        session_epoch: state.session_epoch,
        nmt_revision: state.nmt_revision,
        accumulation_segment: state.measurements.position_accumulation_segment,
        tpdo1_generation: state.measurements.tpdo1_generation,
    })
}

fn validate_live_state(
    state: &hex_motor::meow_motor::MeowMotorLiveState,
    fence: Option<QualificationFence>,
    expected_mode: Option<MeowMotorMode>,
) -> anyhow::Result<()> {
    if !matches!(state.lifecycle, MeowMotorLifecycle::Initialized)
        || !state.connection.online
        || state.connection.nmt_state != Some(NmtState::Operational)
    {
        anyhow::bail!(
            "qualification fence lost: lifecycle={:?}, online={}, NMT={:?}",
            state.lifecycle,
            state.connection.online,
            state.connection.nmt_state,
        );
    }
    if let Some(MeowMotorLogic::Error {
        mode_display,
        detailed_error,
    }) = state.logic
    {
        anyhow::bail!(
            "motor entered error mode 0x{mode_display:02X}, detail 0x{detailed_error:04X}"
        );
    }
    if let Some(expected_mode) = expected_mode {
        if state.logic != Some(MeowMotorLogic::Enabled(expected_mode)) {
            anyhow::bail!(
                "expected enabled mode {expected_mode:?}, observed {:?}",
                state.logic
            );
        }
    }
    if !state.measurements.position_accumulation_valid {
        anyhow::bail!("wide position accumulation became invalid");
    }
    if let Some(fence) = fence {
        if state.session_epoch != fence.session_epoch
            || state.nmt_revision != fence.nmt_revision
            || state.measurements.position_accumulation_segment != fence.accumulation_segment
        {
            anyhow::bail!(
                "session/NMT/position segment changed during qualification: expected {fence:?}, got epoch={}, NMT revision={}, segment={}",
                state.session_epoch,
                state.nmt_revision,
                state.measurements.position_accumulation_segment,
            );
        }
        if state.measurements.tpdo1_generation < fence.tpdo1_generation {
            anyhow::bail!("TPDO1 generation moved backwards");
        }
    }
    for (label, temperature) in [
        ("driver", state.measurements.driver_temp_c),
        ("motor", state.measurements.motor_temp_c),
    ] {
        let temperature = temperature
            .ok_or_else(|| anyhow::anyhow!("{label} temperature feedback is unavailable"))?;
        if temperature >= MAX_TEMPERATURE_C {
            anyhow::bail!("{label} temperature reached {temperature} C");
        }
    }
    let torque = state
        .measurements
        .torque_permille
        .ok_or_else(|| anyhow::anyhow!("actual torque feedback is unavailable"))?;
    if torque.unsigned_abs() > 100 {
        anyhow::bail!("actual torque exceeded the 100 permille qualification watchdog");
    }
    Ok(())
}

fn accumulated_position(manager: &MeowMotorManager, node_id: u8) -> anyhow::Result<f64> {
    manager
        .status(node_id)?
        .measurements
        .accumulated_position_rev
        .ok_or_else(|| anyhow::anyhow!("accumulated position is unavailable"))
}

async fn cleanup(
    manager: &MeowMotorManager,
    bus: &(dyn CanBus + 'static),
    node_id: u8,
    original_consumer: u32,
    restore_consumer: bool,
    initialization_started: bool,
    cleanup_fence: Option<CleanupFence>,
) -> anyhow::Result<()> {
    if !initialization_started {
        println!(
            "cleanup sent no frames because exact identity was not verified for initialization"
        );
        return Ok(());
    }
    let cleanup_fence = cleanup_fence.ok_or_else(|| {
        anyhow::anyhow!(
            "initialization started without a verified cleanup identity; sent no cleanup writes"
        )
    })?;
    verify_cleanup_fence(manager, bus, node_id, cleanup_fence).await?;

    let mut failures = Vec::new();
    if let Err(error) = manager.disable(node_id).await {
        anyhow::bail!(
            "Disabled was not confirmed ({error}); NMT and 0x1016 were deliberately left unchanged so heartbeat loss remains a fail-safe. Cut motor power immediately"
        );
    }
    verify_cleanup_fence(manager, bus, node_id, cleanup_fence)
        .await
        .map_err(|error| {
            anyhow::anyhow!(
                "cleanup session changed after disable; NMT and 0x1016 were deliberately left unchanged: {error}"
            )
        })?;
    match nmt::build_nmt_command(NmtCommand::EnterPreOperational, node_id) {
        Ok(frame) => {
            if let Err(error) = bus.send(frame).await {
                failures.push(format!("NMT Pre-operational: {error}"));
            }
        }
        Err(error) => failures.push(format!("build NMT Pre-operational: {error}")),
    }
    tokio::time::sleep(Duration::from_millis(100)).await;

    if restore_consumer {
        verify_cleanup_fence(manager, bus, node_id, cleanup_fence)
            .await
            .map_err(|error| {
                anyhow::anyhow!(
                    "cleanup session changed after disable; 0x1016 was deliberately left unchanged: {error}"
                )
            })?;
        if let Err(error) = sdo::download_u32(
            bus,
            node_id,
            0x1016,
            1,
            original_consumer,
            Some(SDO_TIMEOUT),
        )
        .await
        {
            failures.push(format!("restore 0x1016:01: {error}"));
        } else {
            match sdo::upload_u32(bus, node_id, 0x1016, 1, Some(SDO_TIMEOUT)).await {
                Ok(observed) if observed == original_consumer => {}
                Ok(observed) => failures.push(format!(
                    "restore 0x1016:01 read-back: expected 0x{original_consumer:08X}, got 0x{observed:08X}"
                )),
                Err(error) => failures.push(format!("read back restored 0x1016:01: {error}")),
            }
        }
    }

    if failures.is_empty() {
        println!("cleanup confirmed Disabled, requested NMT Pre-operational, and restored heartbeat settings");
        Ok(())
    } else {
        anyhow::bail!(failures.join("; "))
    }
}

async fn verify_cleanup_fence(
    manager: &MeowMotorManager,
    bus: &(dyn CanBus + 'static),
    node_id: u8,
    expected: CleanupFence,
) -> anyhow::Result<()> {
    let state = manager.status(node_id)?;
    if state.session_epoch != expected.session_epoch {
        anyhow::bail!(
            "session epoch changed: expected {}, got {}",
            expected.session_epoch,
            state.session_epoch
        );
    }
    let observed = (
        sdo::upload_u32(bus, node_id, 0x1018, 1, Some(SDO_TIMEOUT)).await?,
        sdo::upload_u32(bus, node_id, 0x1018, 2, Some(SDO_TIMEOUT)).await?,
        sdo::upload_u32(bus, node_id, 0x1018, 3, Some(SDO_TIMEOUT)).await?,
        sdo::upload_u32(bus, node_id, 0x1018, 4, Some(SDO_TIMEOUT)).await?,
    );
    let expected_tuple = (
        expected.vendor_id,
        expected.product_code,
        expected.revision,
        expected.serial,
    );
    if observed != expected_tuple {
        anyhow::bail!(
            "cleanup 0x1018 mismatch: expected {expected_tuple:08X?}, got {observed:08X?}"
        );
    }
    Ok(())
}

fn parse_tpdo1_rate(value: &str) -> anyhow::Result<PdoProfile> {
    // 速率是 1000/N Hz —— OD 里的 event timer 就是整毫秒，只有这些值可表达。
    let hz: u32 = value.parse().map_err(|_| anyhow::anyhow!("TPDO1 rate must be a number, got {value}"))?;
    if hz == 0 || 1000 % hz != 0 {
        anyhow::bail!("TPDO1 rate must divide 1000 (1000/500/250/200/125/100/…), got {value}");
    }
    Ok(PdoProfile::from_event_timer_ms((1000 / hz) as u16)?)
}

fn parse_u8(value: &str) -> anyhow::Result<u8> {
    let parsed = if let Some(hex) = value.strip_prefix("0x") {
        u8::from_str_radix(hex, 16)
    } else {
        value.parse()
    }
    .map_err(|error| anyhow::anyhow!("invalid node ID '{value}': {error}"))?;
    if !(1..=127).contains(&parsed) {
        anyhow::bail!("node ID must be 1..=127, got {parsed}");
    }
    Ok(parsed)
}

fn parse_u32(value: &str) -> anyhow::Result<u32> {
    if let Some(hex) = value.strip_prefix("0x") {
        u32::from_str_radix(hex, 16)
            .map_err(|error| anyhow::anyhow!("invalid hexadecimal u32 '{value}': {error}"))
    } else {
        value
            .parse()
            .map_err(|error| anyhow::anyhow!("invalid decimal u32 '{value}': {error}"))
    }
}
