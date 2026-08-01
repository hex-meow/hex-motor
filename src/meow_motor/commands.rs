//! 新对象字典的 SDO command builders。

use std::fmt;

use crate::canopen::tpdo_config::SdoWrite;
use crate::cia402::compressed_mit::{CompressedMitMapping, CompressedMitTarget};

use super::fixed::SignedQ8_24;
use super::types::MeowMotorMode;

/// 非压缩 MIT 目标（0x4102）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeowMitTarget {
    pub position_rev: f32,
    pub velocity_rev_per_s: f32,
    pub torque_nm: f32,
    pub kp: u16,
    pub kd: u16,
    pub kp_kd_limit_permille: u16,
}

/// Profile Position / Profile Velocity 共用的运动轮廓限制。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeowProfileLimits {
    pub velocity_rev_per_s: f32,
    pub acceleration_rev_per_s2: f32,
    pub deceleration_rev_per_s2: f32,
}

/// 与 mode 一一对应的安全/运行 target。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MeowMotorTarget {
    ProfilePosition { position: SignedQ8_24 },
    ProfileVelocity { velocity_rev_per_s: f32 },
    Torque { torque_permille: i16 },
    Mit(MeowMitTarget),
}

impl MeowMotorTarget {
    pub const fn mode(self) -> MeowMotorMode {
        match self {
            Self::ProfilePosition { .. } => MeowMotorMode::ProfilePosition,
            Self::ProfileVelocity { .. } => MeowMotorMode::ProfileVelocity,
            Self::Torque { .. } => MeowMotorMode::Torque,
            Self::Mit(_) => MeowMotorMode::Mit,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CommandBuildError {
    NonFinite { field: &'static str },
    NonPositive { field: &'static str },
    TorquePermilleOutOfRange(i16),
    KpKdLimitOutOfRange(u16),
    MaxTorquePermilleOutOfRange(u16),
    InvalidCompressedMitMapping { field: &'static str },
}

impl fmt::Display for CommandBuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite { field } => write!(f, "{field} must be finite"),
            Self::NonPositive { field } => write!(f, "{field} must be greater than zero"),
            Self::TorquePermilleOutOfRange(value) => {
                write!(
                    f,
                    "torque target must be -1000..=1000 permille, got {value}"
                )
            }
            Self::KpKdLimitOutOfRange(value) => {
                write!(f, "MIT Kp/Kd limit must be 0..=1000 permille, got {value}")
            }
            Self::MaxTorquePermilleOutOfRange(value) => {
                write!(f, "maximum torque must be 0..=1000 permille, got {value}")
            }
            Self::InvalidCompressedMitMapping { field } => {
                write!(f, "compressed MIT mapping has an invalid {field} range")
            }
        }
    }
}

impl std::error::Error for CommandBuildError {}

fn require_finite(field: &'static str, value: f32) -> Result<(), CommandBuildError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(CommandBuildError::NonFinite { field })
    }
}

impl MeowProfileLimits {
    pub fn validate(self) -> Result<(), CommandBuildError> {
        for (field, value) in [
            ("velocity_rev_per_s", self.velocity_rev_per_s),
            ("acceleration_rev_per_s2", self.acceleration_rev_per_s2),
            ("deceleration_rev_per_s2", self.deceleration_rev_per_s2),
        ] {
            require_finite(field, value)?;
            if value <= 0.0 {
                return Err(CommandBuildError::NonPositive { field });
            }
        }
        Ok(())
    }
}

/// 为 target 构造新对象字典写入，不含 mode command。
pub fn build_target_sdo_writes(
    target: MeowMotorTarget,
) -> Result<Vec<SdoWrite>, CommandBuildError> {
    match target {
        MeowMotorTarget::ProfilePosition { position } => {
            Ok(vec![SdoWrite::i32(0x457A, 0, position.raw())])
        }
        MeowMotorTarget::ProfileVelocity { velocity_rev_per_s } => {
            require_finite("velocity_rev_per_s", velocity_rev_per_s)?;
            Ok(vec![SdoWrite::f32(0x45FF, 0, velocity_rev_per_s)])
        }
        MeowMotorTarget::Torque { torque_permille } => {
            if !(-1000..=1000).contains(&torque_permille) {
                return Err(CommandBuildError::TorquePermilleOutOfRange(torque_permille));
            }
            Ok(vec![SdoWrite::i16(0x4571, 0, torque_permille)])
        }
        MeowMotorTarget::Mit(target) => {
            require_finite("position_rev", target.position_rev)?;
            require_finite("velocity_rev_per_s", target.velocity_rev_per_s)?;
            require_finite("torque_nm", target.torque_nm)?;
            if target.kp_kd_limit_permille > 1000 {
                return Err(CommandBuildError::KpKdLimitOutOfRange(
                    target.kp_kd_limit_permille,
                ));
            }
            Ok(vec![
                SdoWrite::f32(0x4102, 1, target.position_rev),
                SdoWrite::f32(0x4102, 2, target.velocity_rev_per_s),
                SdoWrite::f32(0x4102, 3, target.torque_nm),
                SdoWrite::u16(0x4102, 4, target.kp),
                SdoWrite::u16(0x4102, 5, target.kd),
                SdoWrite::u16(0x4102, 6, target.kp_kd_limit_permille),
            ])
        }
    }
}

/// 单独的 0x4401 mode command。运行中切换时不插入 Disabled。
pub fn build_mode_command_sdo_write(mode: MeowMotorMode) -> SdoWrite {
    SdoWrite::u8(0x4401, 0, mode.command_code())
}

/// 先写与目标 mode 匹配的安全 target，再写一次 0x4401。
pub fn build_set_mode_sdo_writes(
    target: MeowMotorTarget,
) -> Result<Vec<SdoWrite>, CommandBuildError> {
    let mode = target.mode();
    let mut writes = build_target_sdo_writes(target)?;
    writes.push(build_mode_command_sdo_write(mode));
    Ok(writes)
}

pub fn build_disable_sdo_write() -> SdoWrite {
    SdoWrite::u8(0x4401, 0, 0)
}

pub fn build_clear_error_sdo_write() -> SdoWrite {
    SdoWrite::u8(0x4401, 0, 0xFF)
}

pub fn build_max_torque_sdo_write(torque_permille: u16) -> Result<SdoWrite, CommandBuildError> {
    if torque_permille > 1000 {
        return Err(CommandBuildError::MaxTorquePermilleOutOfRange(
            torque_permille,
        ));
    }
    Ok(SdoWrite::u16(0x4572, 0, torque_permille))
}

/// 依次设置 PP/PV 的轮廓速度、加速度和减速度。
pub fn build_profile_limits_sdo_writes(
    limits: MeowProfileLimits,
) -> Result<[SdoWrite; 3], CommandBuildError> {
    limits.validate()?;
    Ok([
        SdoWrite::f32(0x4581, 0, limits.velocity_rev_per_s),
        SdoWrite::f32(0x4583, 0, limits.acceleration_rev_per_s2),
        SdoWrite::f32(0x4584, 0, limits.deceleration_rev_per_s2),
    ])
}

/// 配置 0x4103 的量化范围、零初值与 compressed-MIT selector。
///
/// 0x4103:0E 是只读项，本序列不会写它。电机应保持 Disabled，直到 Host TPDO1
/// 已开始发送安全 lane 且上层显式切到 MIT mode。
pub fn build_compressed_mit_setup_sdo_writes(
    mapping: &CompressedMitMapping,
) -> Result<Vec<SdoWrite>, CommandBuildError> {
    let ranges = [
        ("position", mapping.position_min, mapping.position_max),
        ("velocity", mapping.velocity_min, mapping.velocity_max),
        ("torque", mapping.torque_min, mapping.torque_max),
        ("kp", mapping.kp_min, mapping.kp_max),
        ("kd", mapping.kd_min, mapping.kd_max),
    ];
    for (field, min, max) in ranges {
        if !min.is_finite() || !max.is_finite() || min >= max || min > 0.0 || max < 0.0 {
            return Err(CommandBuildError::InvalidCompressedMitMapping { field });
        }
    }

    let mut writes = vec![
        SdoWrite::f32(0x4103, 0x04, mapping.position_min),
        SdoWrite::f32(0x4103, 0x05, mapping.position_max),
        SdoWrite::f32(0x4103, 0x06, mapping.velocity_min),
        SdoWrite::f32(0x4103, 0x07, mapping.velocity_max),
        SdoWrite::f32(0x4103, 0x08, mapping.kp_min),
        SdoWrite::f32(0x4103, 0x09, mapping.kp_max),
        SdoWrite::f32(0x4103, 0x0A, mapping.kd_min),
        SdoWrite::f32(0x4103, 0x0B, mapping.kd_max),
        SdoWrite::f32(0x4103, 0x0C, mapping.torque_min),
        SdoWrite::f32(0x4103, 0x0D, mapping.torque_max),
    ];
    let zero = CompressedMitTarget::ZERO.to_le_bytes(mapping);
    writes.push(SdoWrite::u32(
        0x4103,
        0x02,
        u32::from_le_bytes(zero[0..4].try_into().unwrap()),
    ));
    writes.push(SdoWrite::u32(
        0x4103,
        0x03,
        u32::from_le_bytes(zero[4..8].try_into().unwrap()),
    ));
    writes.push(SdoWrite::u8(0x4103, 0x01, 1));
    Ok(writes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pp_target_is_signed_q8_24_on_wire() {
        let position = SignedQ8_24::from_revolutions(-127.5).unwrap();
        let writes =
            build_target_sdo_writes(MeowMotorTarget::ProfilePosition { position }).unwrap();
        assert_eq!(writes, [SdoWrite::i32(0x457A, 0, -0x7F80_0000)]);
    }

    #[test]
    fn live_mode_switch_writes_target_then_mode_without_disable() {
        let writes = build_set_mode_sdo_writes(MeowMotorTarget::ProfileVelocity {
            velocity_rev_per_s: 0.0,
        })
        .unwrap();
        assert_eq!(writes[0], SdoWrite::f32(0x45FF, 0, 0.0));
        assert_eq!(writes[1], SdoWrite::u8(0x4401, 0, 2));
        assert_eq!(writes.len(), 2);
    }

    #[test]
    fn each_target_uses_only_new_object_dictionary() {
        let target_sets = [
            build_target_sdo_writes(MeowMotorTarget::ProfilePosition {
                position: SignedQ8_24::ZERO,
            })
            .unwrap(),
            build_target_sdo_writes(MeowMotorTarget::ProfileVelocity {
                velocity_rev_per_s: 0.0,
            })
            .unwrap(),
            build_target_sdo_writes(MeowMotorTarget::Torque { torque_permille: 0 }).unwrap(),
            build_target_sdo_writes(MeowMotorTarget::Mit(MeowMitTarget {
                position_rev: 0.0,
                velocity_rev_per_s: 0.0,
                torque_nm: 0.0,
                kp: 0,
                kd: 0,
                kp_kd_limit_permille: 1000,
            }))
            .unwrap(),
        ];
        for writes in target_sets {
            assert!(writes
                .iter()
                .all(|write| matches!(write.index, 0x4102 | 0x4571 | 0x457A | 0x45FF)));
        }
    }

    #[test]
    fn range_validation_is_fail_closed() {
        assert!(build_target_sdo_writes(MeowMotorTarget::Torque {
            torque_permille: 1001
        })
        .is_err());
        assert!(build_max_torque_sdo_write(1001).is_err());
        assert!(build_target_sdo_writes(MeowMotorTarget::ProfileVelocity {
            velocity_rev_per_s: f32::NAN
        })
        .is_err());
    }

    #[test]
    fn profile_limits_are_positive_finite_and_use_exact_objects() {
        let limits = MeowProfileLimits {
            velocity_rev_per_s: 0.5,
            acceleration_rev_per_s2: 1.25,
            deceleration_rev_per_s2: 2.0,
        };
        assert_eq!(
            build_profile_limits_sdo_writes(limits).unwrap(),
            [
                SdoWrite::f32(0x4581, 0, 0.5),
                SdoWrite::f32(0x4583, 0, 1.25),
                SdoWrite::f32(0x4584, 0, 2.0),
            ]
        );

        for invalid in [
            MeowProfileLimits {
                velocity_rev_per_s: 0.0,
                ..limits
            },
            MeowProfileLimits {
                acceleration_rev_per_s2: -1.0,
                ..limits
            },
            MeowProfileLimits {
                deceleration_rev_per_s2: f32::INFINITY,
                ..limits
            },
            MeowProfileLimits {
                velocity_rev_per_s: f32::NAN,
                ..limits
            },
        ] {
            assert!(build_profile_limits_sdo_writes(invalid).is_err());
        }
    }

    #[test]
    fn disable_and_clear_are_independent_one_shots() {
        assert_eq!(build_disable_sdo_write(), SdoWrite::u8(0x4401, 0, 0));
        assert_eq!(build_clear_error_sdo_write(), SdoWrite::u8(0x4401, 0, 0xFF));
    }

    #[test]
    fn compressed_mit_setup_uses_new_object_and_never_writes_read_only_limit() {
        let writes =
            build_compressed_mit_setup_sdo_writes(&CompressedMitMapping::default()).unwrap();
        assert_eq!(writes.len(), 13);
        assert!(writes.iter().all(|write| write.index == 0x4103));
        assert!(writes.iter().all(|write| write.subindex != 0x0E));
        assert_eq!(writes[10].subindex, 0x02);
        assert_eq!(writes[11].subindex, 0x03);
        assert_eq!(writes[12], SdoWrite::u8(0x4103, 0x01, 1));
    }

    #[test]
    fn compressed_mit_setup_rejects_invalid_ranges() {
        let mapping = CompressedMitMapping {
            position_min: 1.0,
            position_max: 1.0,
            ..CompressedMitMapping::default()
        };
        assert!(build_compressed_mit_setup_sdo_writes(&mapping).is_err());

        let excludes_zero = CompressedMitMapping {
            torque_min: 1.0,
            torque_max: 2.0,
            ..CompressedMitMapping::default()
        };
        assert!(build_compressed_mit_setup_sdo_writes(&excludes_zero).is_err());
    }
}
