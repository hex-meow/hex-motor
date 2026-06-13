//! 纯函数：TPDO 解码 + status_word 解析 + CiA402 mode 编号映射 + 错误码归类。
//!
//! 所有函数都是纯 sans-IO，方便单测和复用。`tpdo_listener` 调用它们把收到
//! 的 CAN 帧翻译成 [`crate::cia402::types`] 里的高层概念。
//!
//! ## 字节序约定
//!
//! 全部小端。HexMeow CiA402 电机的几处约定：
//! - TPDO1 position 4 字节按 **f32 单圈 Rev `[-0.5, 0.5)`** 解；标准 CiA402
//!   的 `0x6064` 是 i32 encoder pulse，本库默认 recipe 把这 32 位重用成 f32。
//! - TPDO1 torque 是 i16 `‰ of peak_torque`；要换算成 Nm 需要 `0x6076`
//!   读出来缓存（v0.1 不做，直接暴露 raw permille）。
//! - TPDO2 driver_temp / motor_temp 是 i16 `×0.1 ℃`（vendor-specific
//!   `0x2204:01/02`，标准 CiA402 不保证有，其他厂家走自己的 recipe）。

use crate::types::{MotorErrorKind, MotorMode};

use super::types::Logic;

/// TPDO1 = position(4) + timestamp(4) + torque(2) + error_code(2)。共 12 字节。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tpdo1Frame {
    /// HexMeow 解读：单圈 f32 Rev `[-0.5, 0.5)`。标准 CiA402 `0x6064`
    /// 是 i32 encoder pulse，按 i32 解读的电机请自定义 recipe + 自行解码。
    pub position_rev: f32,
    /// `0x1013` 高分辨率时间戳，单位 μs（CiA301 可选，HexMeow 实现）。
    pub timestamp_us: u32,
    /// raw i16 torque permille of peak。Nm = raw × peak_torque / 1000。
    pub torque_permille: i16,
    /// CiA402 `0x603F`。0 表示无故障。
    pub error_code: u16,
}

/// TPDO2 = status(2) + drv_temp(2) + motor_temp(2) + ctrl(2) + error_code(2)。共 10 字节。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tpdo2Frame {
    pub status_word: u16,
    /// vendor-specific `0x2204:01/02`：i16 `×0.1 ℃`。`degrees_c = raw * 0.1`。
    pub driver_temp_x10: i16,
    pub motor_temp_x10: i16,
    /// 回读自电机的控制字（不是我们写下去那次的回执，是当前内部值）。
    pub control_word_readback: u16,
    pub error_code: u16,
}

impl Tpdo2Frame {
    pub fn driver_temp_c(&self) -> f32 {
        self.driver_temp_x10 as f32 * 0.1
    }
    pub fn motor_temp_c(&self) -> f32 {
        self.motor_temp_x10 as f32 * 0.1
    }
}

/// `None` 表示长度对不上 / 解码失败。
pub fn decode_tpdo1(data: &[u8]) -> Option<Tpdo1Frame> {
    if data.len() < 12 {
        return None;
    }
    Some(Tpdo1Frame {
        position_rev: f32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        timestamp_us: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        torque_permille: i16::from_le_bytes([data[8], data[9]]),
        error_code: u16::from_le_bytes([data[10], data[11]]),
    })
}

/// `None` 表示长度对不上 / 解码失败。
pub fn decode_tpdo2(data: &[u8]) -> Option<Tpdo2Frame> {
    if data.len() < 10 {
        return None;
    }
    Some(Tpdo2Frame {
        status_word: u16::from_le_bytes([data[0], data[1]]),
        driver_temp_x10: i16::from_le_bytes([data[2], data[3]]),
        motor_temp_x10: i16::from_le_bytes([data[4], data[5]]),
        control_word_readback: u16::from_le_bytes([data[6], data[7]]),
        error_code: u16::from_le_bytes([data[8], data[9]]),
    })
}

// =====================================================================
// status_word 解析
// =====================================================================
//
// CiA402 §6.3.1 status word，低 7 位是 CiA402 状态机状态：
//
//   bit 0  Ready To Switch On     (RTSO)
//   bit 1  Switched On            (SO)
//   bit 2  Operation Enabled      (OE)
//   bit 3  Fault                  (F)
//   bit 5  Quick Stop             (active-low)
//   bit 6  Switch On Disabled     (SOD)
//
// 状态由低 4 位 + bit 5,6 联合判定。我们这里**只关心 3 件事**：
//
//   1. Fault 置位（bit 3） → Logic::Error
//   2. Operation Enabled（bits 0..2 == 0b111） → Logic::Enabled
//   3. 其他全部塌缩为 Logic::Disabled
//
// 这样上层 UI 只需要"能/不能/坏了"，足够 v0.1 上位机使用。

/// CiA402 status_word bit 3 = Fault。
pub fn status_word_has_fault(sw: u16) -> bool {
    (sw & 0x0008) != 0
}

/// CiA402 status_word 低 3 位都置 1 = Operation Enabled state。
pub fn status_word_is_operation_enabled(sw: u16) -> bool {
    (sw & 0x0007) == 0x0007
}

/// 把 status_word + 当前目标模式 + 错误码 综合判定为 [`Logic`]。
///
/// - `current_target_mode`：上一次 `set_mode` 设置的模式。`None` 表示从未
///   调用过 `set_mode`（此时即使电机处于 Operation Enabled，我们也不知道是
///   什么模式，退化为 `Disabled`，避免向上层撒谎）。
pub fn status_word_to_logic(
    sw: u16,
    current_target_mode: Option<MotorMode>,
    error_code: u16,
) -> Logic {
    if status_word_has_fault(sw) {
        return Logic::Error {
            kind: error_code_to_kind(error_code),
            raw_code: error_code,
        };
    }
    if status_word_is_operation_enabled(sw) {
        if let Some(m) = current_target_mode {
            return Logic::Enabled(m);
        }
    }
    Logic::Disabled
}

// =====================================================================
// error_code → MotorErrorKind
// =====================================================================
//
// 参考 CiA-402 §A.1 emergency error codes。HexMeow CiA402 电机实际只
// 用到少数几个，其他一律归 `Other`。`raw_code` 在 `Logic::Error` 里保留
// 给上层做更精细的解读。

pub fn error_code_to_kind(code: u16) -> MotorErrorKind {
    match code {
        0x2310 => MotorErrorKind::OverCurrent,
        0x3210 => MotorErrorKind::OverVoltage,
        0x3220 => MotorErrorKind::UnderVoltage,
        0x4210 => MotorErrorKind::DriverOverTemp,
        0x4310 => MotorErrorKind::MotorOverTemp,
        0x7305 => MotorErrorKind::EncoderError,
        0x8130 => MotorErrorKind::HeartbeatLost,
        _ => MotorErrorKind::Other,
    }
}

// =====================================================================
// MotorMode ↔ CiA402 0x6060 mode_of_operation 编号
// =====================================================================

/// 把库的高层模式映射到 CiA402 `0x6060` 写值。
pub fn mode_to_cia402_code(mode: MotorMode) -> i8 {
    match mode {
        MotorMode::ProfilePosition => 1,
        MotorMode::ProfileVelocity => 3,
        // 4 = Profile Torque (PT)。CiA402 也定义了 10 = CST，HexMeow CiA402 电机用 4。
        MotorMode::Torque => 4,
        // 5 = vendor-specific MIT 模式（标准 CiA402 未定义）。
        MotorMode::Mit => 5,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_tpdo1_too_short_returns_none() {
        assert!(decode_tpdo1(&[]).is_none());
        assert!(decode_tpdo1(&[0; 11]).is_none());
    }

    #[test]
    fn decode_tpdo1_roundtrip() {
        let mut buf = [0u8; 12];
        buf[..4].copy_from_slice(&0.25f32.to_le_bytes());
        buf[4..8].copy_from_slice(&12345u32.to_le_bytes());
        buf[8..10].copy_from_slice(&(-500i16).to_le_bytes());
        buf[10..12].copy_from_slice(&0x2310u16.to_le_bytes());
        let f = decode_tpdo1(&buf).unwrap();
        assert!((f.position_rev - 0.25).abs() < f32::EPSILON);
        assert_eq!(f.timestamp_us, 12345);
        assert_eq!(f.torque_permille, -500);
        assert_eq!(f.error_code, 0x2310);
    }

    #[test]
    fn decode_tpdo2_too_short_returns_none() {
        assert!(decode_tpdo2(&[0; 9]).is_none());
    }

    #[test]
    fn decode_tpdo2_roundtrip() {
        let mut buf = [0u8; 10];
        buf[..2].copy_from_slice(&0x0237u16.to_le_bytes()); // OperationEnabled
        buf[2..4].copy_from_slice(&425i16.to_le_bytes()); // 42.5 ℃
        buf[4..6].copy_from_slice(&510i16.to_le_bytes()); // 51.0 ℃
        buf[6..8].copy_from_slice(&0x000Fu16.to_le_bytes());
        buf[8..10].copy_from_slice(&0u16.to_le_bytes());
        let f = decode_tpdo2(&buf).unwrap();
        assert_eq!(f.status_word, 0x0237);
        assert!((f.driver_temp_c() - 42.5).abs() < 0.01);
        assert!((f.motor_temp_c() - 51.0).abs() < 0.01);
        assert_eq!(f.control_word_readback, 0x000F);
        assert_eq!(f.error_code, 0);
    }

    #[test]
    fn status_fault_detection() {
        assert!(!status_word_has_fault(0x0237));
        assert!(status_word_has_fault(0x0008));
        assert!(status_word_has_fault(0x00FF));
    }

    #[test]
    fn status_op_enabled_detection() {
        assert!(status_word_is_operation_enabled(0x0237));
        assert!(status_word_is_operation_enabled(0x0007));
        assert!(!status_word_is_operation_enabled(0x0003)); // Switched On but not OE
        assert!(!status_word_is_operation_enabled(0x0040)); // Switch On Disabled
    }

    #[test]
    fn logic_from_sw_fault_overrides_enabled_bits() {
        // Fault + OE bits both set: should still be Error
        let l = status_word_to_logic(0x000F, Some(MotorMode::ProfileVelocity), 0x2310);
        assert!(matches!(
            l,
            Logic::Error {
                kind: MotorErrorKind::OverCurrent,
                raw_code: 0x2310,
            }
        ));
    }

    #[test]
    fn logic_from_sw_enabled_uses_target_mode() {
        let l = status_word_to_logic(0x0237, Some(MotorMode::ProfileVelocity), 0);
        assert_eq!(l, Logic::Enabled(MotorMode::ProfileVelocity));
    }

    #[test]
    fn logic_from_sw_enabled_without_target_mode_is_disabled() {
        // No prior set_mode → unknown mode → don't lie, say Disabled
        let l = status_word_to_logic(0x0237, None, 0);
        assert_eq!(l, Logic::Disabled);
    }

    #[test]
    fn logic_from_sw_not_enabled_is_disabled() {
        // Switch On Disabled state, no fault
        let l = status_word_to_logic(0x0040, Some(MotorMode::ProfileVelocity), 0);
        assert_eq!(l, Logic::Disabled);
    }

    #[test]
    fn error_code_known_codes_map_correctly() {
        assert_eq!(error_code_to_kind(0x2310), MotorErrorKind::OverCurrent);
        assert_eq!(error_code_to_kind(0x3210), MotorErrorKind::OverVoltage);
        assert_eq!(error_code_to_kind(0x3220), MotorErrorKind::UnderVoltage);
        assert_eq!(error_code_to_kind(0x4210), MotorErrorKind::DriverOverTemp);
        assert_eq!(error_code_to_kind(0x8130), MotorErrorKind::HeartbeatLost);
        assert_eq!(error_code_to_kind(0x9999), MotorErrorKind::Other);
    }

    #[test]
    fn mode_codes_match_cia402_spec() {
        assert_eq!(mode_to_cia402_code(MotorMode::ProfilePosition), 1);
        assert_eq!(mode_to_cia402_code(MotorMode::ProfileVelocity), 3);
        assert_eq!(mode_to_cia402_code(MotorMode::Torque), 4);
        assert_eq!(mode_to_cia402_code(MotorMode::Mit), 5);
    }
}
