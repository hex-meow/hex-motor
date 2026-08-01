//! 标准 0x2001 通信设置及 0x1010 保存计划。

use std::fmt;
use std::time::Duration;

use can_transport::CanBus;

use crate::canopen::sdo;
use crate::canopen::tpdo_config::SdoWrite;
use crate::error::{Error, Result as MotorResult};

pub const NOMINAL_BITRATES: &[u32] = &[125_000, 250_000, 500_000, 800_000, 1_000_000];
pub const DATA_BITRATES: &[u32] = &[1_000_000, 2_000_000, 4_000_000, 5_000_000];

const STORE_SIGNATURE: u32 = 0x6576_6173;

/// 0x2001:01..04 的完整快照/目标。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeowMotorCanSettings {
    pub node_id: u8,
    pub nominal_bitrate: u32,
    pub data_bitrate: u32,
    pub transmit_pdo_brs: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettingsError {
    InvalidNodeId(u8),
    UnsupportedNominalBitrate(u32),
    UnsupportedDataBitrate(u32),
    InvalidHighestSubindex(u8),
    InvalidBrsValue(u8),
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNodeId(id) => write!(f, "node id must be 1..=127, got {id}"),
            Self::UnsupportedNominalBitrate(rate) => {
                write!(f, "unsupported nominal bitrate {rate}")
            }
            Self::UnsupportedDataBitrate(rate) => write!(f, "unsupported data bitrate {rate}"),
            Self::InvalidHighestSubindex(value) => {
                write!(f, "0x2001:00 must report 4, got {value}")
            }
            Self::InvalidBrsValue(value) => write!(f, "0x2001:04 must be 0 or 1, got {value}"),
        }
    }
}

impl std::error::Error for SettingsError {}

impl MeowMotorCanSettings {
    pub fn validate(self) -> Result<(), SettingsError> {
        if !(1..=127).contains(&self.node_id) {
            return Err(SettingsError::InvalidNodeId(self.node_id));
        }
        if !NOMINAL_BITRATES.contains(&self.nominal_bitrate) {
            return Err(SettingsError::UnsupportedNominalBitrate(
                self.nominal_bitrate,
            ));
        }
        if !DATA_BITRATES.contains(&self.data_bitrate) {
            return Err(SettingsError::UnsupportedDataBitrate(self.data_bitrate));
        }
        Ok(())
    }

    /// 读取 0x2001:00 后必须用此检查对象形状，不能猜测缺失的子索引。
    pub fn validate_highest_subindex(value: u8) -> Result<(), SettingsError> {
        if value == 4 {
            Ok(())
        } else {
            Err(SettingsError::InvalidHighestSubindex(value))
        }
    }
}

/// 一次显式设置事务的纯计划。
///
/// 调用方必须依次执行 `field_writes`，read-back 0x2001:01..04 并与 `target`
/// 完全比较，最后才执行 `save_write`。保存成功仍须受控重新上电。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeowMotorCanSettingsPlan {
    pub target: MeowMotorCanSettings,
    pub field_writes: Vec<SdoWrite>,
    pub save_write: SdoWrite,
    pub changed: bool,
}

pub fn build_settings_update_plan(
    current: MeowMotorCanSettings,
    target: MeowMotorCanSettings,
) -> Result<MeowMotorCanSettingsPlan, SettingsError> {
    current.validate()?;
    target.validate()?;

    let mut field_writes = Vec::new();
    if current.nominal_bitrate != target.nominal_bitrate {
        field_writes.push(SdoWrite::u32(0x2001, 2, target.nominal_bitrate));
    }
    if current.data_bitrate != target.data_bitrate {
        field_writes.push(SdoWrite::u32(0x2001, 3, target.data_bitrate));
    }
    if current.transmit_pdo_brs != target.transmit_pdo_brs {
        field_writes.push(SdoWrite::u8(0x2001, 4, u8::from(target.transmit_pdo_brs)));
    }
    if current.node_id != target.node_id {
        // Node-ID is deliberately the final 0x2001 field write.
        field_writes.push(SdoWrite::u8(0x2001, 1, target.node_id));
    }
    let changed = !field_writes.is_empty();

    Ok(MeowMotorCanSettingsPlan {
        target,
        field_writes,
        save_write: SdoWrite::u32(0x1010, 4, STORE_SIGNATURE),
        changed,
    })
}

/// 显式 commissioning 时持久化 1000h communication-profile 对象。
pub fn pdo_communication_save_write() -> SdoWrite {
    SdoWrite::u32(0x1010, 2, STORE_SIGNATURE)
}

pub(crate) async fn read_from_device(
    bus: &(impl CanBus + ?Sized),
    node_id: u8,
    timeout: Duration,
) -> MotorResult<MeowMotorCanSettings> {
    let timeout = Some(timeout);
    let highest = sdo::upload_u8(bus, node_id, 0x2001, 0, timeout).await?;
    MeowMotorCanSettings::validate_highest_subindex(highest)
        .map_err(|error| Error::Internal(error.to_string()))?;
    let settings = MeowMotorCanSettings {
        node_id: sdo::upload_u8(bus, node_id, 0x2001, 1, timeout).await?,
        nominal_bitrate: sdo::upload_u32(bus, node_id, 0x2001, 2, timeout).await?,
        data_bitrate: sdo::upload_u32(bus, node_id, 0x2001, 3, timeout).await?,
        transmit_pdo_brs: match sdo::upload_u8(bus, node_id, 0x2001, 4, timeout).await? {
            0 => false,
            1 => true,
            value => {
                return Err(Error::Internal(
                    SettingsError::InvalidBrsValue(value).to_string(),
                ))
            }
        },
    };
    settings
        .validate()
        .map_err(|error| Error::Internal(error.to_string()))?;
    Ok(settings)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings() -> MeowMotorCanSettings {
        MeowMotorCanSettings {
            node_id: 10,
            nominal_bitrate: 1_000_000,
            data_bitrate: 5_000_000,
            transmit_pdo_brs: true,
        }
    }

    #[test]
    fn complete_standard_rate_sets_are_accepted() {
        for nominal_bitrate in NOMINAL_BITRATES {
            for data_bitrate in DATA_BITRATES {
                MeowMotorCanSettings {
                    nominal_bitrate: *nominal_bitrate,
                    data_bitrate: *data_bitrate,
                    ..settings()
                }
                .validate()
                .unwrap();
            }
        }
        assert!(MeowMotorCanSettings {
            nominal_bitrate: 2_000_000,
            ..settings()
        }
        .validate()
        .is_err());
        assert!(MeowMotorCanSettings {
            data_bitrate: 3_000_000,
            ..settings()
        }
        .validate()
        .is_err());
    }

    #[test]
    fn update_writes_fields_then_node_id_and_separates_save() {
        let target = MeowMotorCanSettings {
            node_id: 42,
            nominal_bitrate: 500_000,
            data_bitrate: 2_000_000,
            transmit_pdo_brs: false,
        };
        let plan = build_settings_update_plan(settings(), target).unwrap();
        assert!(plan.changed);
        assert_eq!(plan.field_writes[0], SdoWrite::u32(0x2001, 2, 500_000));
        assert_eq!(plan.field_writes[1], SdoWrite::u32(0x2001, 3, 2_000_000));
        assert_eq!(plan.field_writes[2], SdoWrite::u8(0x2001, 4, 0));
        assert_eq!(plan.field_writes[3], SdoWrite::u8(0x2001, 1, 42));
        assert_eq!(plan.save_write, SdoWrite::u32(0x1010, 4, STORE_SIGNATURE));
        assert_eq!(plan.save_write.data, [0x73, 0x61, 0x76, 0x65]);
    }

    #[test]
    fn unchanged_retry_still_provides_explicit_save() {
        let plan = build_settings_update_plan(settings(), settings()).unwrap();
        assert!(!plan.changed);
        assert!(plan.field_writes.is_empty());
        assert_eq!(plan.save_write.subindex, 4);
    }

    #[test]
    fn pdo_mapping_uses_communication_group_save_only() {
        let write = pdo_communication_save_write();
        assert_eq!((write.index, write.subindex), (0x1010, 2));
    }
}
