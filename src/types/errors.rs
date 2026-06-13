//! 跨协议的电机故障语义类别。

/// 电机故障语义类别。各协议的原始错误码由该协议的 codec 映射到此枚举。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotorErrorKind {
    OverCurrent,
    OverVoltage,
    UnderVoltage,
    DriverOverTemp,
    MotorOverTemp,
    HeartbeatLost,
    EncoderError,
    HallError,
    MotorStall,
    StartupDifficult,
    VelocityError,
    PositionError,
    /// 其他未识别错误。`raw_code` 字段在 `LiveState` 里仍保留。
    Other,
}
