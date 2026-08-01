//! Device TPDO1/TPDO2 的严格小端 codec。

use std::fmt;

use super::fixed::SignedQ8_24;
use super::types::{MeowMotorLogic, MeowMotorMode};

pub const TPDO1_LEN: usize = 8;
pub const TPDO2_LEN: usize = 7;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecError {
    InvalidLength { expected: usize, actual: usize },
    InvalidModeDisplay(u8),
}

impl fmt::Display for CodecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLength { expected, actual } => {
                write!(f, "invalid PDO length: expected {expected}, got {actual}")
            }
            Self::InvalidModeDisplay(code) => {
                write!(f, "invalid mode display 0x{code:02X}")
            }
        }
    }
}

impl std::error::Error for CodecError {}

/// TPDO1 = signed Q8.24 position + u16 timestamp + i16 torque permille。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tpdo1Frame {
    pub position: SignedQ8_24,
    pub timestamp_us: u16,
    pub torque_permille: i16,
}

/// TPDO2 = u8 mode + u16 detailed error + two i16 temperatures (0.1 °C)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tpdo2Frame {
    pub mode_display: u8,
    pub detailed_error: u16,
    pub driver_temp_x10: i16,
    pub motor_temp_x10: i16,
}

impl Tpdo2Frame {
    pub fn driver_temp_c(self) -> f32 {
        self.driver_temp_x10 as f32 * 0.1
    }

    pub fn motor_temp_c(self) -> f32 {
        self.motor_temp_x10 as f32 * 0.1
    }

    pub fn logic(self) -> Result<MeowMotorLogic, CodecError> {
        match self.mode_display {
            0 => Ok(MeowMotorLogic::Disabled),
            1..=4 => Ok(MeowMotorLogic::Enabled(
                MeowMotorMode::from_display_code(self.mode_display).unwrap(),
            )),
            0xA1..=0xA6 | 0xAF => Ok(MeowMotorLogic::Error {
                mode_display: self.mode_display,
                detailed_error: self.detailed_error,
            }),
            code => Err(CodecError::InvalidModeDisplay(code)),
        }
    }
}

pub fn decode_tpdo1(data: &[u8]) -> Result<Tpdo1Frame, CodecError> {
    if data.len() != TPDO1_LEN {
        return Err(CodecError::InvalidLength {
            expected: TPDO1_LEN,
            actual: data.len(),
        });
    }
    Ok(Tpdo1Frame {
        position: SignedQ8_24::from_le_bytes(data[0..4].try_into().unwrap()),
        timestamp_us: u16::from_le_bytes(data[4..6].try_into().unwrap()),
        torque_permille: i16::from_le_bytes(data[6..8].try_into().unwrap()),
    })
}

pub fn decode_tpdo2(data: &[u8]) -> Result<Tpdo2Frame, CodecError> {
    if data.len() != TPDO2_LEN {
        return Err(CodecError::InvalidLength {
            expected: TPDO2_LEN,
            actual: data.len(),
        });
    }
    Ok(Tpdo2Frame {
        mode_display: data[0],
        detailed_error: u16::from_le_bytes(data[1..3].try_into().unwrap()),
        driver_temp_x10: i16::from_le_bytes(data[3..5].try_into().unwrap()),
        motor_temp_x10: i16::from_le_bytes(data[5..7].try_into().unwrap()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tpdo1_decodes_exact_eight_bytes() {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&(-0x0180_0000i32).to_le_bytes());
        data[4..6].copy_from_slice(&0xFFF0u16.to_le_bytes());
        data[6..8].copy_from_slice(&(-375i16).to_le_bytes());
        let frame = decode_tpdo1(&data).unwrap();
        assert_eq!(frame.position.raw(), -0x0180_0000);
        assert_eq!(frame.timestamp_us, 0xFFF0);
        assert_eq!(frame.torque_permille, -375);
        assert!(decode_tpdo1(&[0; 7]).is_err());
        assert!(decode_tpdo1(&[0; 9]).is_err());
    }

    #[test]
    fn tpdo2_decodes_exact_seven_bytes_and_logic() {
        let mut data = [0u8; 7];
        data[0] = 2;
        data[1..3].copy_from_slice(&0u16.to_le_bytes());
        data[3..5].copy_from_slice(&425i16.to_le_bytes());
        data[5..7].copy_from_slice(&(-55i16).to_le_bytes());
        let frame = decode_tpdo2(&data).unwrap();
        assert_eq!(
            frame.logic().unwrap(),
            MeowMotorLogic::Enabled(MeowMotorMode::ProfileVelocity)
        );
        assert!((frame.driver_temp_c() - 42.5).abs() < f32::EPSILON);
        assert!((frame.motor_temp_c() + 5.5).abs() < f32::EPSILON);
    }

    #[test]
    fn error_display_preserves_detailed_code() {
        let frame = Tpdo2Frame {
            mode_display: 0xAF,
            detailed_error: 0x8140,
            driver_temp_x10: 0,
            motor_temp_x10: 0,
        };
        assert_eq!(
            frame.logic().unwrap(),
            MeowMotorLogic::Error {
                mode_display: 0xAF,
                detailed_error: 0x8140,
            }
        );
        assert!(matches!(
            Tpdo2Frame {
                mode_display: 5,
                ..frame
            }
            .logic(),
            Err(CodecError::InvalidModeDisplay(5))
        ));
    }
}
