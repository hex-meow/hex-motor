//! 用户视角的电机控制模式。

/// 用户视角的电机控制模式（语义级，跨协议复用）。
///
/// 各协议各自的"实际模式编号"由该协议的 driver 内部映射。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotorMode {
    /// 轮廓位置模式。CiA402 中对应 mode 1 (PP)。
    ProfilePosition,
    /// 轮廓速度模式。CiA402 中对应 mode 3 (PV)。
    ProfileVelocity,
    /// 力矩模式。CiA402 中对应 mode 4 (PT) / 10 (CST)。
    Torque,
    /// MIT (位置/速度/力矩/Kp/Kd) 复合模式。
    Mit,
}

impl MotorMode {
    /// 简短名称，用于日志和错误信息。
    pub fn name(&self) -> &'static str {
        match self {
            MotorMode::ProfilePosition => "ProfilePosition",
            MotorMode::ProfileVelocity => "ProfileVelocity",
            MotorMode::Torque => "Torque",
            MotorMode::Mit => "Mit",
        }
    }
}
