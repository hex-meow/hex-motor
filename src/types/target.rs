//! 电机控制目标 + 配置参数。

use super::mode::MotorMode;

/// 单台电机的控制目标。
///
/// enum variant 必须与当前电机所处模式匹配，否则 `set_target` 会返回
/// `Error::TargetModeMismatch`。`Disable` 是通用的，在任何模式下都合法。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorTarget {
    /// 失能（短刹车）。任何模式下都合法。
    Disable,
    /// 位置目标，单位 Rev。
    Position { rev: f32 },
    /// 速度目标，单位 Rev/s。
    Velocity { rev_per_s: f32 },
    /// 力矩目标，单位 Nm。
    Torque { nm: f32 },
    /// MIT 复合目标。
    Mit {
        /// 位置目标，单位 Rev。
        pos: f32,
        /// 速度目标，单位 Rev/s。
        vel: f32,
        /// 前馈力矩，单位 Nm。
        tor: f32,
        /// 比例增益，单位 Nm/Rev。
        kp: f32,
        /// 微分增益，单位 Nm·s/Rev。
        kd: f32,
    },
}

impl MotorTarget {
    /// 推断该 target 对应的模式。`Disable` 通用，返回 `None`。
    pub fn mode_hint(&self) -> Option<MotorMode> {
        match self {
            MotorTarget::Disable => None,
            MotorTarget::Position { .. } => Some(MotorMode::ProfilePosition),
            MotorTarget::Velocity { .. } => Some(MotorMode::ProfileVelocity),
            MotorTarget::Torque { .. } => Some(MotorMode::Torque),
            MotorTarget::Mit { .. } => Some(MotorMode::Mit),
        }
    }

    /// 该 target 是否能在指定模式下使用。
    pub fn matches_mode(&self, mode: MotorMode) -> bool {
        match self.mode_hint() {
            Some(m) => m == mode,
            None => true,
        }
    }

    /// 用于错误信息和日志的简短描述。
    pub fn variant_name(&self) -> &'static str {
        match self {
            MotorTarget::Disable => "Disable",
            MotorTarget::Position { .. } => "Position",
            MotorTarget::Velocity { .. } => "Velocity",
            MotorTarget::Torque { .. } => "Torque",
            MotorTarget::Mit { .. } => "Mit",
        }
    }
}

/// 轮廓模式（位置/速度）通用参数。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProfileParams {
    /// 巡航速度，单位 Rev/s。
    pub velocity_rev_per_s: f32,
    /// 加速段加速度，单位 Rev/s^2。
    pub acceleration_rev_per_s2: f32,
    /// 减速段减速度，单位 Rev/s^2。
    pub deceleration_rev_per_s2: f32,
}

/// MIT 模式各分量的硬件映射上下限。写到电机 SDO 用。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MitMapping {
    pub position_min: f32,
    pub position_max: f32,
    pub velocity_min: f32,
    pub velocity_max: f32,
    pub torque_min: f32,
    pub torque_max: f32,
    pub kp_min: f32,
    pub kp_max: f32,
    pub kd_min: f32,
    pub kd_max: f32,
}

impl Default for MitMapping {
    fn default() -> Self {
        Self {
            position_min: -0.5,
            position_max: 0.5,
            velocity_min: -10.0,
            velocity_max: 10.0,
            torque_min: -10.0,
            torque_max: 10.0,
            kp_min: 0.0,
            kp_max: 100.0,
            kd_min: 0.0,
            kd_max: 20.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_mode_match() {
        assert!(MotorTarget::Position { rev: 1.0 }.matches_mode(MotorMode::ProfilePosition));
        assert!(!MotorTarget::Position { rev: 1.0 }.matches_mode(MotorMode::ProfileVelocity));
        assert!(MotorTarget::Disable.matches_mode(MotorMode::Torque));
        assert!(MotorTarget::Mit {
            pos: 0.0,
            vel: 0.0,
            tor: 0.0,
            kp: 0.0,
            kd: 0.0,
        }
        .matches_mode(MotorMode::Mit));
    }

    #[test]
    fn target_mode_hint() {
        assert_eq!(MotorTarget::Disable.mode_hint(), None);
        assert_eq!(
            MotorTarget::Position { rev: 1.0 }.mode_hint(),
            Some(MotorMode::ProfilePosition)
        );
        assert_eq!(
            MotorTarget::Velocity { rev_per_s: 1.0 }.mode_hint(),
            Some(MotorMode::ProfileVelocity)
        );
        assert_eq!(
            MotorTarget::Torque { nm: 1.0 }.mode_hint(),
            Some(MotorMode::Torque)
        );
    }

    #[test]
    fn target_variant_name() {
        assert_eq!(MotorTarget::Disable.variant_name(), "Disable");
        assert_eq!(MotorTarget::Position { rev: 0.0 }.variant_name(), "Position");
    }

    #[test]
    fn mit_mapping_default_consistent() {
        let m = MitMapping::default();
        assert!(m.position_min < m.position_max);
        assert!(m.velocity_min < m.velocity_max);
        assert!(m.kp_min <= m.kp_max);
        assert!(m.kd_min <= m.kd_max);
    }
}
