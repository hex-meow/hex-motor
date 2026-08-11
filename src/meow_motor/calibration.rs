//! `0x4001` v1 出厂标定区的只读解码。
//!
//! 本模块的边界是有意收窄的：
//!
//! - 只做 wire 层解码 + CRC16 自洽性检查。`:02..03` 的 8-byte 签发 token 只原样读出，
//!   **本库不验证它**；判断该 token 是否由我方签发需要联网查签发账本，那是上位机和
//!   服务端的职责，不属于电机驱动库。
//! - 解出的 torque factor 与摩擦力只是原始物理量。**本库不会把它们应用到任何命令
//!   路径**：`raw_command = desired_torque * torque_factor + friction_raw` 这一步
//!   由调用方自行决定是否做、怎么做。摩擦值本身不得再乘或除 `torque_factor`。
//! - 没有标定区、区域不是完整 v1、CRC 不符，都如实返回
//!   [`MeowFactoryCalibration::Missing`] 并带上原因和（读到时的）原始字，交给调用方
//!   判断。这些都不是 `Err`；只有 SDO/传输失败才是 `Err`。
//!
//! v1 布局（七个 U32，`0x4001:01..07`，每项精确 4 字节）：
//!
//! | 子索引 | 低半字 | 高半字 |
//! |---:|---|---|
//! | `:01` | CRC16 | `format_version` + `payload_words`，固定 [`MANIFEST_UPPER_V1`] |
//! | `:02` | 签发 token 的低 32 位（little-endian 的前四字节） | |
//! | `:03` | 签发 token 的高 32 位 | |
//! | `:04` | torque factor（`e4m11`） | 拟合 RMSE（`e4m11`） |
//! | `:05` | 正向静摩擦（`e4m11`，修正前原始命令域 Nm） | 反向静摩擦 |
//! | `:06` | 正向动摩擦 | 反向动摩擦 |
//! | `:07` | 动摩擦参考速度（v1 固定 1 rad/s） | 标定温度（°C） |
//!
//! `:05..07` 三个 U32 全零是"出厂未标定摩擦"的规范缺省值，返回
//! [`MeowFactoryCalibrationV1::friction`] `= None`；否则整组必须完整有效。
//!
//! 注意 `0x4001` 是 RW 自由存储区：`0x1011` 恢复默认值或旧版"自由用户存储"工具会
//! 清除或破坏这片数据。本库只读，不写。

use std::time::Duration;

use can_transport::CanBus;
use canopen_sdo::{asynch::AsyncSdoError, SdoAbortCode, SdoError};

use crate::canopen::sdo;
use crate::error::{Error, Result as MotorResult};
use crate::types::MotorIdentity;

/// 出厂标定与来源数据所在的厂商对象。
pub const FACTORY_DATA_INDEX: u16 = 0x4001;
/// v1 使用的子索引数量：`:01..:07`。
pub const FACTORY_DATA_WORD_COUNT: usize = 7;
/// `:01` 的高半字：`format_version = 1`、`payload_words = 6`。
///
/// v1 没有单独的 magic：版本、长度和 CRC 共同承担格式判别。
pub const MANIFEST_UPPER_V1: u16 = 0x0106;

/// CRC transcript 的固定域分隔前缀（含结尾 NUL）。
const CRC_DOMAIN: &[u8; 23] = b"hex-meow/0x4001/crc/v1\0";

/// 为什么这台电机没有可用的 v1 出厂标定。
///
/// 这些都不是通信故障，而是"如实报告"的判定结果。
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum FactoryDataError {
    /// 对象字典里根本没有 `0x4001`。
    #[error("0x4001 does not exist on this device")]
    ObjectAbsent,

    /// `0x4001:00` 报告的最高子索引不足 7。
    #[error("0x4001:00 reports {highest} sub-indices; v1 requires at least 7")]
    TooFewSubIndices { highest: u8 },

    /// `:00` 声称有该子索引，实际上传却是"子索引不存在"。
    #[error("0x4001:{subindex:02X} does not exist although 0x4001:00 promised it")]
    SubIndexAbsent { subindex: u8 },

    /// 长度不是精确 1（`:00`）或精确 4（`:01..07`）字节。
    #[error("0x4001:{subindex:02X} is {len} bytes, expected exactly {expected}")]
    WrongLength {
        subindex: u8,
        len: usize,
        expected: usize,
    },

    /// manifest 高半字不是 v1。
    #[error("0x4001:01 manifest upper half is 0x{found:04X}, not v1 0x{expected:04X}")]
    NotV1 { found: u16, expected: u16 },

    /// CRC 与按本机 identity 重算的结果不符。
    ///
    /// 数据被改写、被旧工具破坏，或这片字节属于另一台电机时都会命中这一条。
    #[error("0x4001 v1 CRC16 mismatch: manifest carries 0x{found:04X}, computed 0x{computed:04X}")]
    CrcMismatch { found: u16, computed: u16 },

    /// 签发 token 为零。零不是合法 token，本库只做这一项非零检查，不做任何验证。
    #[error("0x4001:02..03 issuance token is zero")]
    ZeroToken,

    /// 某个半字不是规范的 `e4m11`（指数全 1，或负零）。
    #[error("0x4001:{subindex:02X} half-word 0x{raw:04X} is not a canonical e4m11 value")]
    NonCanonicalE4m11 { subindex: u8, raw: u16 },

    /// torque factor 必须为正，RMSE 不得为负。
    #[error("0x4001:04 torque factor {factor} / fit RMSE {rmse} is out of range")]
    TorqueGroupInvalid { factor: f64, rmse: f64 },

    /// `:05..07` 不是"整组全零"也不是"整组有效正值"。
    #[error("0x4001:05..07 friction group is incomplete or non-positive")]
    FrictionGroupInvalid,
}

/// `:04` 的力矩系数组。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeowTorqueCalibration {
    /// 出厂标定的力矩系数。
    ///
    /// 定义为 `raw_command_nm = desired_physical_torque_nm * factor`。本库只解出它，
    /// 不做这一步换算。
    pub factor: f64,
    /// 出厂力矩拟合残差 RMSE（Nm）。
    pub fit_rmse_nm: f64,
}

/// `:05..07` 的摩擦力组。四个力矩值都在**修正前的原始命令域**，单位 Nm。
///
/// 组合顺序固定为 `raw_command = desired_torque * torque_factor + friction_raw`；
/// 摩擦值本身不得再乘或除 `torque_factor`。本库不做这一步组合。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeowFrictionCalibration {
    pub static_pos_raw_nm: f64,
    pub static_neg_raw_nm: f64,
    pub kinetic_pos_raw_nm: f64,
    pub kinetic_neg_raw_nm: f64,
    /// 动摩擦参考速度，v1 固定 1 rad/s。
    pub kinetic_reference_speed_rad_per_s: f64,
    /// 出厂标定时的温度（°C）。
    pub calibration_temperature_c: f64,
}

/// 一份完整、CRC 自洽的 v1 出厂标定。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeowFactoryCalibrationV1 {
    /// `0x4001:01..07` 的原始 U32，按子索引升序。写回前备份、上报服务端时使用。
    pub words: [u32; FACTORY_DATA_WORD_COUNT],
    /// `:02..03` 的 8-byte 签发 token。
    ///
    /// 本库只把它原样读出，**不做任何验证**，也不认为它是客户秘密；它在电机上可读。
    /// 需要判断来源时由联网上位机把它连同完整 identity 提交给签发账本。
    pub issuance_token: u64,
    pub torque: MeowTorqueCalibration,
    /// `None` 表示 `:05..07` 全零，即出厂未标定摩擦。
    pub friction: Option<MeowFrictionCalibration>,
}

/// 一次 `0x4001` 读取的结论。
#[derive(Debug, Clone, PartialEq)]
pub enum MeowFactoryCalibration {
    /// 没有可用的出厂标定。按规范，格式、长度、数值或 CRC 任一不符都等同"没有标定"，
    /// 不得只应用其中一部分字段。
    Missing {
        reason: FactoryDataError,
        /// 完整读到七个字时保留原始值，便于诊断和备份；连区域都没读全时为 `None`。
        words: Option<[u32; FACTORY_DATA_WORD_COUNT]>,
    },
    /// 完整 v1，CRC 与本机 identity 自洽。
    Valid(MeowFactoryCalibrationV1),
}

impl MeowFactoryCalibration {
    pub fn valid(&self) -> Option<&MeowFactoryCalibrationV1> {
        match self {
            Self::Valid(v1) => Some(v1),
            Self::Missing { .. } => None,
        }
    }

    pub fn is_valid(&self) -> bool {
        matches!(self, Self::Valid(_))
    }

    /// 读到的原始七个字，无论是否有效。
    pub fn words(&self) -> Option<&[u32; FACTORY_DATA_WORD_COUNT]> {
        match self {
            Self::Valid(v1) => Some(&v1.words),
            Self::Missing { words, .. } => words.as_ref(),
        }
    }
}

/// 解码已经读到的七个字。CRC 绑定 identity，所以必须传入该电机自己的 `0x1018`。
///
/// `identity` 中只有 `vendor_id`、`product_code` 和 `serial_number` 参与 CRC；
/// `revision_number` 会随固件升级变化，不参与。
pub fn decode_factory_words(
    identity: &MotorIdentity,
    words: [u32; FACTORY_DATA_WORD_COUNT],
) -> MeowFactoryCalibration {
    match decode_v1(identity, &words) {
        Ok(v1) => MeowFactoryCalibration::Valid(v1),
        Err(reason) => MeowFactoryCalibration::Missing {
            reason,
            words: Some(words),
        },
    }
}

fn decode_v1(
    identity: &MotorIdentity,
    words: &[u32; FACTORY_DATA_WORD_COUNT],
) -> Result<MeowFactoryCalibrationV1, FactoryDataError> {
    let manifest_upper = (words[0] >> 16) as u16;
    if manifest_upper != MANIFEST_UPPER_V1 {
        return Err(FactoryDataError::NotV1 {
            found: manifest_upper,
            expected: MANIFEST_UPPER_V1,
        });
    }
    let found = words[0] as u16;
    let computed = factory_crc16(identity, words);
    if found != computed {
        return Err(FactoryDataError::CrcMismatch { found, computed });
    }

    let issuance_token = u64::from(words[1]) | (u64::from(words[2]) << 32);
    if issuance_token == 0 {
        return Err(FactoryDataError::ZeroToken);
    }

    let factor = e4m11(4, words[3] as u16)?;
    let fit_rmse_nm = e4m11(4, (words[3] >> 16) as u16)?;
    if factor <= 0.0 || fit_rmse_nm < 0.0 {
        return Err(FactoryDataError::TorqueGroupInvalid {
            factor,
            rmse: fit_rmse_nm,
        });
    }

    // 规范缺省：三个字全零 = 出厂未标定摩擦。
    let friction = if words[4] == 0 && words[5] == 0 && words[6] == 0 {
        None
    } else {
        let static_pos_raw_nm = e4m11(5, words[4] as u16)?;
        let static_neg_raw_nm = e4m11(5, (words[4] >> 16) as u16)?;
        let kinetic_pos_raw_nm = e4m11(6, words[5] as u16)?;
        let kinetic_neg_raw_nm = e4m11(6, (words[5] >> 16) as u16)?;
        let kinetic_reference_speed_rad_per_s = e4m11(7, words[6] as u16)?;
        // 四个摩擦力矩记录的是幅值，参考速度也必须为正；只有温度可以是零或负。
        if [
            static_pos_raw_nm,
            static_neg_raw_nm,
            kinetic_pos_raw_nm,
            kinetic_neg_raw_nm,
            kinetic_reference_speed_rad_per_s,
        ]
        .iter()
        .any(|value| *value <= 0.0)
        {
            return Err(FactoryDataError::FrictionGroupInvalid);
        }
        Some(MeowFrictionCalibration {
            static_pos_raw_nm,
            static_neg_raw_nm,
            kinetic_pos_raw_nm,
            kinetic_neg_raw_nm,
            kinetic_reference_speed_rad_per_s,
            calibration_temperature_c: e4m11(7, (words[6] >> 16) as u16)?,
        })
    };

    Ok(MeowFactoryCalibrationV1 {
        words: *words,
        issuance_token,
        torque: MeowTorqueCalibration {
            factor,
            fit_rmse_nm,
        },
        friction,
    })
}

fn e4m11(subindex: u8, raw: u16) -> Result<f64, FactoryDataError> {
    decode_e4m11(raw).ok_or(FactoryDataError::NonCanonicalE4m11 { subindex, raw })
}

/// 按 v1 transcript 重算 CRC16/CCITT-FALSE。
///
/// transcript 精确为 61 字节：域前缀 || `vendor_id` || `product_code` ||
/// `serial_number` || `words[1..7]` || [`MANIFEST_UPPER_V1`]，全部 little-endian。
/// `words[0]` 自身不参与。
pub fn factory_crc16(identity: &MotorIdentity, words: &[u32; FACTORY_DATA_WORD_COUNT]) -> u16 {
    let mut transcript = Vec::with_capacity(61);
    transcript.extend_from_slice(CRC_DOMAIN);
    transcript.extend_from_slice(&identity.vendor_id.to_le_bytes());
    transcript.extend_from_slice(&identity.product_code.to_le_bytes());
    transcript.extend_from_slice(&identity.serial_number.to_le_bytes());
    for word in &words[1..] {
        transcript.extend_from_slice(&word.to_le_bytes());
    }
    transcript.extend_from_slice(&MANIFEST_UPPER_V1.to_le_bytes());
    crc16_ccitt_false(&transcript)
}

fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for byte in bytes {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// 解码定制的 `e4m11` 半精度：1 符号位 + 4 位指数（bias 7）+ 11 位尾数。
///
/// 指数全 1 保留（没有 Inf/NaN 编码），负零同样非规范；两者都返回 `None`。
/// 指数为 0 是次正规数，步长 `2^-6 / 2048`。
pub fn decode_e4m11(raw: u16) -> Option<f64> {
    let negative = raw & 0x8000 != 0;
    let exponent = (raw >> 11) & 0x0F;
    let fraction = raw & 0x07FF;
    if exponent == 0x0F || (exponent == 0 && fraction == 0 && negative) {
        return None;
    }
    let magnitude = if exponent == 0 {
        2f64.powi(-6) * (f64::from(fraction) / 2048.0)
    } else {
        2f64.powi(i32::from(exponent) - 7) * (1.0 + f64::from(fraction) / 2048.0)
    };
    Some(if negative { -magnitude } else { magnitude })
}

/// 从设备读取整片 `0x4001` 并解码。
///
/// 逐个子索引严格要求精确长度（`:00` 一字节，`:01..07` 各四字节）。对象或子索引
/// 不存在会被翻译成 [`FactoryDataError`] 而不是 `Err`；其余 SDO/传输失败原样上抛。
///
/// `identity` 必须是这台电机自己刚读到的 `0x1018` —— CRC 绑定 identity。用
/// [`MeowMotorManager::read_factory_calibration`] 时这一步由 manager 负责。
///
/// [`MeowMotorManager::read_factory_calibration`]: super::manager::MeowMotorManager::read_factory_calibration
pub async fn read_from_device(
    bus: &(impl CanBus + ?Sized),
    node_id: u8,
    identity: &MotorIdentity,
    timeout: Duration,
) -> MotorResult<MeowFactoryCalibration> {
    let timeout = Some(timeout);

    let Some(highest) = upload_optional(bus, node_id, 0, timeout).await? else {
        return Ok(missing(FactoryDataError::ObjectAbsent));
    };
    if highest.len() != 1 {
        return Ok(missing(FactoryDataError::WrongLength {
            subindex: 0,
            len: highest.len(),
            expected: 1,
        }));
    }
    let highest = highest[0];
    if usize::from(highest) < FACTORY_DATA_WORD_COUNT {
        return Ok(missing(FactoryDataError::TooFewSubIndices { highest }));
    }

    let mut words = [0u32; FACTORY_DATA_WORD_COUNT];
    for (offset, word) in words.iter_mut().enumerate() {
        let subindex = (offset + 1) as u8;
        let Some(bytes) = upload_optional(bus, node_id, subindex, timeout).await? else {
            return Ok(missing(FactoryDataError::SubIndexAbsent { subindex }));
        };
        let Ok(bytes) = <[u8; 4]>::try_from(bytes.as_slice()) else {
            return Ok(missing(FactoryDataError::WrongLength {
                subindex,
                len: bytes.len(),
                expected: 4,
            }));
        };
        *word = u32::from_le_bytes(bytes);
    }

    Ok(decode_factory_words(identity, words))
}

fn missing(reason: FactoryDataError) -> MeowFactoryCalibration {
    MeowFactoryCalibration::Missing {
        reason,
        words: None,
    }
}

/// `Ok(None)` = 服务器明确回答"对象/子索引不存在"。
async fn upload_optional(
    bus: &(impl CanBus + ?Sized),
    node_id: u8,
    subindex: u8,
    timeout: Option<Duration>,
) -> MotorResult<Option<Vec<u8>>> {
    match sdo::upload(bus, node_id, FACTORY_DATA_INDEX, subindex, timeout).await {
        Ok(bytes) => Ok(Some(bytes)),
        Err(Error::Sdo(AsyncSdoError::Sdo(SdoError::ServerAborted(
            SdoAbortCode::ObjectDoesNotExist | SdoAbortCode::SubindexDoesNotExist,
        )))) => Ok(None),
        Err(other) => Err(other),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meow_motor::identity::{PRODUCT_CODE_4310, VENDOR_ID};

    const SERIAL: u32 = 0x0111_B49A;
    const TOKEN: u64 = 0x0123_4567_89AB_CDEF;

    // e4m11: 1.0 = 0x3800, 0.1 ≈ 0x1CCD, 0.5 = 0x3000, 0.25 = 0x2800, 25.0 = 0x5C80。
    const WORDS_WITH_FRICTION: [u32; 7] = [
        0x0106_2E25,
        0x89AB_CDEF,
        0x0123_4567,
        0x1CCD_3800,
        0x3000_3000,
        0x2800_2800,
        0x5C80_3800,
    ];
    const WORDS_NO_FRICTION: [u32; 7] =
        [0x0106_61B2, 0x89AB_CDEF, 0x0123_4567, 0x1CCD_3800, 0, 0, 0];

    fn identity() -> MotorIdentity {
        MotorIdentity {
            node_id: 1,
            vendor_id: VENDOR_ID,
            product_code: PRODUCT_CODE_4310,
            // revision 不参与 CRC，随便取一个值。
            revision_number: 0x0002_0000,
            serial_number: SERIAL,
            product_name: None,
        }
    }

    fn reason(words: [u32; 7]) -> FactoryDataError {
        match decode_factory_words(&identity(), words) {
            MeowFactoryCalibration::Missing {
                reason,
                words: kept,
            } => {
                assert_eq!(kept, Some(words), "原始字必须保留下来供诊断");
                reason
            }
            MeowFactoryCalibration::Valid(_) => panic!("expected a rejection"),
        }
    }

    /// 重算 manifest 字，让被改过的 payload 重新 CRC 自洽 —— 这样后面的断言测的是
    /// 数值/结构检查，而不是又一次 CRC 失败。
    fn resealed(mut words: [u32; 7]) -> [u32; 7] {
        let crc = u32::from(factory_crc16(&identity(), &words));
        words[0] = (u32::from(MANIFEST_UPPER_V1) << 16) | crc;
        words
    }

    #[test]
    fn e4m11_matches_frozen_values() {
        for (raw, expected) in [
            (0x0000u16, 0.0),
            (0x3000, 0.5),
            (0x3800, 1.0),
            (0x4000, 2.0),
            (0x2800, 0.25),
            (0x5C80, 25.0),
            (0xB800, -1.0),
            (0x77FF, 255.9375),
        ] {
            assert_eq!(decode_e4m11(raw), Some(expected), "raw 0x{raw:04X}");
        }
        // 次正规：0x0001 = 2^-6 / 2048。
        assert_eq!(decode_e4m11(0x0001), Some(2f64.powi(-6) / 2048.0));
        let tenth = decode_e4m11(0x1CCD).unwrap();
        assert!((tenth - 0.1).abs() < 1e-4, "0x1CCD decoded as {tenth}");
    }

    #[test]
    fn e4m11_rejects_reserved_exponent_and_negative_zero() {
        assert_eq!(decode_e4m11(0x7800), None);
        assert_eq!(decode_e4m11(0xF800), None);
        assert_eq!(decode_e4m11(0x8000), None);
        assert_eq!(decode_e4m11(0x0000), Some(0.0));
    }

    #[test]
    fn crc_transcript_is_frozen() {
        // 独立算出的 golden 值：域前缀 + vendor/product/serial + words[1..7] + 0x0106。
        assert_eq!(factory_crc16(&identity(), &WORDS_WITH_FRICTION), 0x2E25);
        assert_eq!(factory_crc16(&identity(), &WORDS_NO_FRICTION), 0x61B2);
    }

    #[test]
    fn complete_v1_decodes_without_applying_anything() {
        let v1 = match decode_factory_words(&identity(), WORDS_WITH_FRICTION) {
            MeowFactoryCalibration::Valid(v1) => v1,
            other => panic!("expected valid, got {other:?}"),
        };
        assert_eq!(v1.words, WORDS_WITH_FRICTION);
        assert_eq!(v1.issuance_token, TOKEN);
        assert_eq!(v1.torque.factor, 1.0);
        assert!((v1.torque.fit_rmse_nm - 0.1).abs() < 1e-4);
        let friction = v1.friction.expect("friction group is present");
        assert_eq!(friction.static_pos_raw_nm, 0.5);
        assert_eq!(friction.static_neg_raw_nm, 0.5);
        assert_eq!(friction.kinetic_pos_raw_nm, 0.25);
        assert_eq!(friction.kinetic_neg_raw_nm, 0.25);
        assert_eq!(friction.kinetic_reference_speed_rad_per_s, 1.0);
        assert_eq!(friction.calibration_temperature_c, 25.0);
    }

    #[test]
    fn all_zero_friction_group_is_the_uncalibrated_default() {
        let v1 = decode_factory_words(&identity(), WORDS_NO_FRICTION)
            .valid()
            .copied()
            .expect("torque-only v1 is still valid");
        assert_eq!(v1.torque.factor, 1.0);
        assert_eq!(v1.friction, None);
    }

    #[test]
    fn crc_binds_the_payload_to_this_exact_motor() {
        let mut other_serial = identity();
        other_serial.serial_number = SERIAL + 1;
        assert!(matches!(
            decode_factory_words(&other_serial, WORDS_WITH_FRICTION),
            MeowFactoryCalibration::Missing {
                reason: FactoryDataError::CrcMismatch { .. },
                ..
            }
        ));

        let mut tampered = WORDS_WITH_FRICTION;
        tampered[3] = 0x1CCD_4000; // factor 1.0 -> 2.0，CRC 未同步更新
        assert!(matches!(
            reason(tampered),
            FactoryDataError::CrcMismatch { .. }
        ));
    }

    #[test]
    fn non_v1_manifest_is_rejected_before_the_crc() {
        let mut words = WORDS_WITH_FRICTION;
        words[0] = 0x0107_2E25;
        assert_eq!(
            reason(words),
            FactoryDataError::NotV1 {
                found: 0x0107,
                expected: MANIFEST_UPPER_V1,
            }
        );
        assert_eq!(
            reason([0xFFFF_FFFF; 7]),
            FactoryDataError::NotV1 {
                found: 0xFFFF,
                expected: MANIFEST_UPPER_V1,
            }
        );
        assert_eq!(
            reason([0; 7]),
            FactoryDataError::NotV1 {
                found: 0,
                expected: MANIFEST_UPPER_V1,
            }
        );
    }

    #[test]
    fn partial_friction_group_never_degrades_to_a_partial_apply() {
        // 只有 :05 被清零，:06/:07 仍有值：整组作废，不允许只应用其中一半。
        let mut words = WORDS_WITH_FRICTION;
        words[4] = 0;
        assert_eq!(
            reason(resealed(words)),
            FactoryDataError::FrictionGroupInvalid
        );
    }

    #[test]
    fn zero_token_and_non_positive_torque_factor_are_rejected() {
        let mut words = WORDS_WITH_FRICTION;
        words[1] = 0;
        words[2] = 0;
        assert_eq!(reason(resealed(words)), FactoryDataError::ZeroToken);

        let mut words = WORDS_WITH_FRICTION;
        words[3] = 0x1CCD_0000; // factor = +0.0
        assert!(matches!(
            reason(resealed(words)),
            FactoryDataError::TorqueGroupInvalid { .. }
        ));
    }

    #[test]
    fn non_canonical_half_word_is_reported_with_its_subindex() {
        let mut words = WORDS_WITH_FRICTION;
        words[5] = 0x2800_7800; // :06 低半字使用保留指数
        assert_eq!(
            reason(resealed(words)),
            FactoryDataError::NonCanonicalE4m11 {
                subindex: 6,
                raw: 0x7800,
            }
        );
    }
}
