//! Meow Motor 专属状态、生命周期和测量类型。

use std::time::Instant;

use crate::canopen::nmt::NmtState;
use crate::types::MotorIdentity;

use super::fixed::SignedQ8_24;
use super::identity::{model_for_identity, MeowMotorModel};
use super::settings::MeowMotorCanSettings;

/// 0x4401/0x4402 的运行模式编号。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum MeowMotorMode {
    ProfilePosition = 1,
    ProfileVelocity = 2,
    Torque = 3,
    Mit = 4,
}

impl MeowMotorMode {
    pub const fn command_code(self) -> u8 {
        self as u8
    }

    pub const fn from_display_code(code: u8) -> Option<Self> {
        match code {
            1 => Some(Self::ProfilePosition),
            2 => Some(Self::ProfileVelocity),
            3 => Some(Self::Torque),
            4 => Some(Self::Mit),
            _ => None,
        }
    }
}

/// 由 0x4402 和 0x453F 得出的逻辑状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeowMotorLogic {
    Disabled,
    Enabled(MeowMotorMode),
    Error {
        mode_display: u8,
        detailed_error: u16,
    },
}

/// 两路 Device TPDO 合并后的测量快照。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeowMotorMeasurements {
    pub position: Option<SignedQ8_24>,
    /// 从本次在线 session 的第一帧起，以 modular delta 累计的宽位置。
    pub accumulated_position_rev: Option<f64>,
    /// false 表示 telemetry gap 破坏了连续累计基线。
    pub position_accumulation_valid: bool,
    /// 每次 gap 使该代数递增；即使调用方错过短暂的 invalid 帧也能发现断段。
    pub position_accumulation_segment: u64,
    pub velocity_rev_per_s: Option<f64>,
    pub timestamp_us: Option<u16>,
    pub torque_permille: Option<i16>,
    pub driver_temp_c: Option<f32>,
    pub motor_temp_c: Option<f32>,
    pub mode_display: Option<u8>,
    pub detailed_error: Option<u16>,
    /// 只在 TPDO1 成功解码后递增。
    pub tpdo1_generation: u64,
    /// 只在 TPDO2 成功解码后递增；模式事务以它作 freshness fence。
    pub tpdo2_generation: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum MeowMotorCanSettingsStatus {
    #[default]
    Pending,
    Available(MeowMotorCanSettings),
    ReadFailed {
        reason: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeowMotorReinitReason {
    LeftOperational,
    TelemetryStale,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MeowMotorLifecycle {
    Unknown,
    UnsupportedIdentity,
    Identified,
    Initializing,
    Initialized,
    NeedsReinit {
        reason: MeowMotorReinitReason,
    },
    /// 0x2001 已保存，必须受控重新上电后再识别和初始化。
    NeedsRestart,
}

impl MeowMotorLifecycle {
    pub const fn is_ready(&self) -> bool {
        matches!(self, Self::Initialized)
    }
}

#[derive(Debug, Clone, Default)]
pub struct MeowMotorConnection {
    pub last_heartbeat: Option<Instant>,
    pub last_tpdo1: Option<Instant>,
    pub last_tpdo2: Option<Instant>,
    pub online: bool,
    pub nmt_state: Option<NmtState>,
}

#[derive(Debug, Clone)]
pub struct MeowMotorLiveState {
    /// Changes on Boot-Up/offline invalidation; external Host-PDO transactions must fence on it.
    pub session_epoch: u64,
    /// Changes whenever the observed NMT state changes.
    pub nmt_revision: u64,
    pub lifecycle: MeowMotorLifecycle,
    pub connection: MeowMotorConnection,
    pub logic: Option<MeowMotorLogic>,
    pub measurements: MeowMotorMeasurements,
    pub timestamp: Instant,
}

impl MeowMotorLiveState {
    pub fn empty(now: Instant) -> Self {
        Self {
            session_epoch: 0,
            nmt_revision: 0,
            lifecycle: MeowMotorLifecycle::Unknown,
            connection: MeowMotorConnection::default(),
            logic: None,
            measurements: MeowMotorMeasurements::default(),
            timestamp: now,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MeowMotorInfo {
    pub node_id: u8,
    pub session_epoch: u64,
    pub identity: Option<MotorIdentity>,
    pub model: Option<MeowMotorModel>,
    pub can_settings: MeowMotorCanSettingsStatus,
    pub lifecycle: MeowMotorLifecycle,
    pub online: bool,
    pub nmt_state: Option<NmtState>,
    pub logic: Option<MeowMotorLogic>,
    pub peak_torque_nm: Option<f32>,
    pub mit_kp_kd_factor: Option<f32>,
}

impl MeowMotorInfo {
    pub fn is_ready(&self) -> bool {
        self.lifecycle.is_ready() && self.online
    }

    pub fn friendly_name(&self) -> String {
        if let Some(model) = self.model {
            return model.display_name().to_string();
        }
        if let Some(identity) = &self.identity {
            if let Some(model) = model_for_identity(identity) {
                return model.display_name().to_string();
            }
            return format!(
                "Unsupported CANopen device (vendor 0x{:08X}, product 0x{:08X})",
                identity.vendor_id, identity.product_code
            );
        }
        format!("Node 0x{:02X}", self.node_id)
    }
}

/// 16-bit 微秒时间戳的 modular delta。
pub const fn timestamp_delta_us(current: u16, previous: u16) -> u16 {
    current.wrapping_sub(previous)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timestamp_delta_wraps_at_u16_boundary() {
        assert_eq!(timestamp_delta_us(0x0004, 0xFFFC), 8);
    }

    #[test]
    fn mode_codes_are_protocol_specific() {
        assert_eq!(MeowMotorMode::ProfilePosition.command_code(), 1);
        assert_eq!(MeowMotorMode::ProfileVelocity.command_code(), 2);
        assert_eq!(MeowMotorMode::Torque.command_code(), 3);
        assert_eq!(MeowMotorMode::Mit.command_code(), 4);
        assert_eq!(MeowMotorMode::from_display_code(5), None);
    }

    #[test]
    fn only_initialized_lifecycle_is_ready() {
        assert!(MeowMotorLifecycle::Initialized.is_ready());
        assert!(!MeowMotorLifecycle::Identified.is_ready());
        assert!(!MeowMotorLifecycle::NeedsRestart.is_ready());
    }
}
