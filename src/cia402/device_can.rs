//! 按精确设备身份读取 CAN 配置对象。
//!
//! 这里的分派键始终是完整的 `(vendor_id, product_code)`。即使两个产品共用
//! vendor ID，也不能据此推断它们使用相同对象字典。

use std::time::Duration;

use can_transport::CanBus;

use crate::canopen::sdo;
use crate::canopen::tpdo_config::SdoWrite;
use crate::types::{
    DeviceCanConfig, DeviceCanConfigStatus, DeviceSettingsResult, DeviceSettingsUpdate,
    MotorIdentity,
};

use super::motor_entry::MotorEntry;
use super::types::MotorLifecycle;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CanConfigDialect {
    Object2001,
    Object2100,
}

const OBJECT_2001_IDENTITIES: &[(u32, u32)] = &[
    (0x4859_444C, 0xAAAA_0001),
    (0x4859_444C, 0xAAAA_0002),
    (0x4859_444C, 0xAAAA_0005),
    (0x0068_6578, 0x6C64_AE8D),
    (0x0068_6578, 0x6C64_BC78),
    (0x0068_6578, 0x6C64_BCAA),
];

const OBJECT_2100_IDENTITIES: &[(u32, u32)] = &[
    (0x6865_786D, 0x0069_6D75),
    (0x6865_786D, 0x6169_6D75),
    (0x6865_786D, 0x006C_6674),
];

const OBJECT_2100_IMU_IDENTITIES: &[(u32, u32)] = &[
    (0x6865_786D, 0x0069_6D75),
    (0x6865_786D, 0x6169_6D75),
];

const OBJECT_2100_LIFT_IDENTITY: (u32, u32) = (0x6865_786D, 0x006C_6674);

pub(crate) fn dialect_for_tuple(
    vendor_id: u32,
    product_code: u32,
) -> Option<CanConfigDialect> {
    let key = (vendor_id, product_code);
    if OBJECT_2001_IDENTITIES.contains(&key) {
        Some(CanConfigDialect::Object2001)
    } else if OBJECT_2100_IDENTITIES.contains(&key) {
        Some(CanConfigDialect::Object2100)
    } else {
        None
    }
}

fn dialect_for(identity: &MotorIdentity) -> Option<CanConfigDialect> {
    dialect_for_tuple(identity.vendor_id, identity.product_code)
}

pub(crate) fn is_object_2001_tuple(vendor_id: u32, product_code: u32) -> bool {
    OBJECT_2001_IDENTITIES.contains(&(vendor_id, product_code))
}

pub(crate) fn validate_role_node_id(
    vendor_id: u32,
    product_code: u32,
    new_node_id: u8,
) -> std::result::Result<(), String> {
    let key = (vendor_id, product_code);
    if OBJECT_2100_IMU_IDENTITIES.contains(&key) && !(31..=40).contains(&new_node_id) {
        return Err(format!(
            "node ID {new_node_id} is outside the allowed IMU range 31..=40"
        ));
    }
    if key == OBJECT_2100_LIFT_IDENTITY && !(16..=20).contains(&new_node_id) {
        return Err(format!(
            "node ID {new_node_id} is outside the allowed lift range 16..=20"
        ));
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct DeviceSettingsSnapshot {
    pub dialect: CanConfigDialect,
    pub config: DeviceCanConfig,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct DeviceSettingsPlan {
    pub writes: Vec<SdoWrite>,
    pub result: DeviceSettingsResult,
}

/// 身份成功后立即缓存，并把设备配置标成正在读取。
///
/// 返回 `false` 表示异步读取期间 session 已替换，旧结果被丢弃。
pub(crate) fn begin_device_can_config_read(
    entry: &MotorEntry,
    expected_epoch: u64,
    identity: &MotorIdentity,
) -> bool {
    let mut inner = entry.inner.lock().unwrap();
    if inner.session_epoch != expected_epoch {
        return false;
    }
    inner.identity = Some(identity.clone());
    inner.can_config = DeviceCanConfigStatus::Pending;
    if matches!(inner.lifecycle, MotorLifecycle::Unknown) {
        inner.lifecycle = MotorLifecycle::Identified;
    }
    true
}

/// 缓存最终配置状态。即使读取失败，也不修改已成功读取的 identity/lifecycle。
pub(crate) fn finish_device_can_config_read(
    entry: &MotorEntry,
    expected_epoch: u64,
    status: DeviceCanConfigStatus,
) -> bool {
    let mut inner = entry.inner.lock().unwrap();
    if inner.session_epoch != expected_epoch {
        return false;
    }
    inner.can_config = status;
    true
}

pub(crate) async fn read_device_can_config(
    bus: &dyn CanBus,
    identity: &MotorIdentity,
    sdo_timeout: Duration,
) -> DeviceCanConfigStatus {
    let Some(dialect) = dialect_for(identity) else {
        return DeviceCanConfigStatus::Unsupported;
    };

    let timeout = Some(sdo_timeout);
    let result = match dialect {
        CanConfigDialect::Object2001 => read_object_2001(bus, identity.node_id, timeout).await,
        CanConfigDialect::Object2100 => read_object_2100(bus, identity.node_id, timeout).await,
    };

    match result {
        Ok(config) => DeviceCanConfigStatus::Available(config),
        Err(reason) => DeviceCanConfigStatus::ReadFailed { reason },
    }
}

/// 设置按钮使用的强制重读：除了通信配置，也读取对象字典里保存的 Node-ID。
///
/// 调用方必须已经用本次会话重新读取的 0x1018 选择了精确方言。
pub(crate) async fn read_device_settings_snapshot(
    bus: &dyn CanBus,
    nid: u8,
    vendor_id: u32,
    product_code: u32,
    sdo_timeout: Duration,
) -> std::result::Result<DeviceSettingsSnapshot, String> {
    let dialect = dialect_for_tuple(vendor_id, product_code)
        .ok_or_else(|| "the exact vendor/product tuple has no supported settings dialect".to_string())?;
    let timeout = Some(sdo_timeout);
    let config = match dialect {
        CanConfigDialect::Object2001 => read_object_2001(bus, nid, timeout).await?,
        CanConfigDialect::Object2100 => read_object_2100(bus, nid, timeout).await?,
    };

    Ok(DeviceSettingsSnapshot { dialect, config })
}

/// 纯函数：比较对象字典当前值和完整目标值，生成顺序敏感的写事务。
pub(crate) fn build_device_settings_plan(
    current: &DeviceSettingsSnapshot,
    update: DeviceSettingsUpdate,
) -> std::result::Result<DeviceSettingsPlan, String> {
    if update.nominal_bitrate != 1_000_000 {
        return Err(format!(
            "nominal bitrate must be 1000000, got {}",
            update.nominal_bitrate
        ));
    }
    if let Some(data_bitrate) = update.data_bitrate {
        if !matches!(data_bitrate, 1_000_000 | 2_000_000 | 4_000_000 | 5_000_000) {
            return Err(format!(
                "data bitrate must be 1000000, 2000000, 4000000, or 5000000, got {data_bitrate}"
            ));
        }
    }

    match current.dialect {
        CanConfigDialect::Object2001 => build_object_2001_settings_plan(current, update),
        CanConfigDialect::Object2100 => build_object_2100_settings_plan(current, update),
    }
}

fn build_object_2001_settings_plan(
    current: &DeviceSettingsSnapshot,
    update: DeviceSettingsUpdate,
) -> std::result::Result<DeviceSettingsPlan, String> {
    let data_bitrate = update
        .data_bitrate
        .ok_or_else(|| "0x2001 devices require a data bitrate".to_string())?;
    let transmit_pdo_brs = update
        .transmit_pdo_brs
        .ok_or_else(|| "0x2001 devices require a TPDO BRS value".to_string())?;
    let current_data_bitrate = current
        .config
        .data_bitrate
        .ok_or_else(|| "0x2001 snapshot unexpectedly lacks a data bitrate".to_string())?;
    let current_brs = current
        .config
        .transmit_pdo_brs
        .ok_or_else(|| "0x2001 snapshot unexpectedly lacks a TPDO BRS value".to_string())?;

    let mut writes = Vec::new();
    if current.config.nominal_bitrate != update.nominal_bitrate {
        writes.push(SdoWrite::u32(0x2001, 2, update.nominal_bitrate));
    }
    if current_data_bitrate != data_bitrate {
        writes.push(SdoWrite::u32(0x2001, 3, data_bitrate));
    }
    if current_brs != transmit_pdo_brs {
        writes.push(SdoWrite::u8(0x2001, 4, u8::from(transmit_pdo_brs)));
    }
    if current.config.stored_node_id != update.new_node_id {
        // Node-ID 必须是设备字段中的最后一写。
        writes.push(SdoWrite::u8(0x2001, 1, update.new_node_id));
    }

    let changed = !writes.is_empty();
    // 只有前面的全部字段写成功后，才触发唯一一次持久化。即使字段已经等于目标
    // 也显式 store：上一次事务可能在字段写成功、store 失败后中断，重试必须仍
    // 能完成持久化，不能被 RAM 中看似相等的值短路。
    writes.push(SdoWrite::u32(0x1010, 1, 0x6576_6173));
    Ok(DeviceSettingsPlan {
        writes,
        result: DeviceSettingsResult {
            changed,
            // 即使当前 RAM 值相同，也可能是在“字段写成功、上次 store 失败”
            // 后重试。store 成功后仍保守要求重新上电，避免把不确定运行态报告为
            // 已经生效。
            restart_required: true,
            persistence_pending: false,
            brs_applied_immediately: false,
        },
    })
}

fn build_object_2100_settings_plan(
    current: &DeviceSettingsSnapshot,
    update: DeviceSettingsUpdate,
) -> std::result::Result<DeviceSettingsPlan, String> {
    // 0x2100 是 write-through。显式 Apply 总是完整重写目标字段，既能重新触发
    // WT 持久化，也能从“上次部分写成功后中断”的不确定状态可靠恢复。
    let mut writes = vec![SdoWrite::u32(0x2100, 1, update.nominal_bitrate)];
    let (data_changed, brs_changed) = match (
        current.config.data_bitrate,
        current.config.transmit_pdo_brs,
        update.data_bitrate,
        update.transmit_pdo_brs,
    ) {
        (None, None, None, None) => (false, false),
        (Some(current_data), Some(current_brs), Some(target_data), Some(target_brs)) => {
            let data_changed = current_data != target_data;
            let brs_changed = current_brs != target_brs;
            writes.push(SdoWrite::u32(0x2100, 2, target_data));
            writes.push(SdoWrite::u8(0x2100, 3, u8::from(target_brs)));
            (data_changed, brs_changed)
        }
        (None, None, Some(_), _) | (None, None, _, Some(_)) => {
            return Err(
                "Classic-only 0x2100 shape does not permit data bitrate or BRS writes".to_string(),
            )
        }
        (Some(_), Some(_), None, _) | (Some(_), Some(_), _, None) => {
            return Err("CAN FD 0x2100 shape requires data bitrate and BRS values".to_string())
        }
        _ => {
            return Err("inconsistent 0x2100 configuration snapshot shape".to_string());
        }
    };

    // Node-ID 即使相同也最后重写，重新 kick write-through persistence。
    let node_changed = current.config.stored_node_id != update.new_node_id;
    writes.push(SdoWrite::u8(0x2101, 0, update.new_node_id));

    let nominal_changed = current.config.nominal_bitrate != update.nominal_bitrate;
    let changed = nominal_changed || data_changed || brs_changed || node_changed;
    Ok(DeviceSettingsPlan {
        writes,
        result: DeviceSettingsResult {
            changed,
            // 显式完整重写也可能是在恢复前次 partial retry；保守要求重启。
            restart_required: true,
            persistence_pending: true,
            brs_applied_immediately: brs_changed,
        },
    })
}

async fn read_object_2001(
    bus: &dyn CanBus,
    nid: u8,
    timeout: Option<Duration>,
) -> std::result::Result<DeviceCanConfig, String> {
    let max_subindex = sdo::upload_u8(bus, nid, 0x2001, 0, timeout)
        .await
        .map_err(|e| format!("read 0x2001:00 failed: {e}"))?;
    if max_subindex != 4 {
        return Err(format!(
            "0x2001:00 expected max sub-index 4, got {max_subindex}"
        ));
    }

    let stored_node_id = sdo::upload_u8(bus, nid, 0x2001, 1, timeout)
        .await
        .map_err(|e| format!("read 0x2001:01 failed: {e}"))?;
    let nominal_bitrate = sdo::upload_u32(bus, nid, 0x2001, 2, timeout)
        .await
        .map_err(|e| format!("read 0x2001:02 failed: {e}"))?;
    let data_bitrate = sdo::upload_u32(bus, nid, 0x2001, 3, timeout)
        .await
        .map_err(|e| format!("read 0x2001:03 failed: {e}"))?;
    let transmit_pdo_brs = sdo::upload_u8(bus, nid, 0x2001, 4, timeout)
        .await
        .map_err(|e| format!("read 0x2001:04 failed: {e}"))?;

    Ok(DeviceCanConfig {
        stored_node_id,
        nominal_bitrate,
        data_bitrate: Some(data_bitrate),
        transmit_pdo_brs: Some(parse_bool(0x2001, 4, transmit_pdo_brs)?),
    })
}

async fn read_object_2100(
    bus: &dyn CanBus,
    nid: u8,
    timeout: Option<Duration>,
) -> std::result::Result<DeviceCanConfig, String> {
    let max_subindex = sdo::upload_u8(bus, nid, 0x2100, 0, timeout)
        .await
        .map_err(|e| format!("read 0x2100:00 failed: {e}"))?;
    if !matches!(max_subindex, 1 | 3) {
        return Err(format!(
            "0x2100:00 expected max sub-index 1 or 3, got {max_subindex}"
        ));
    }
    let stored_node_id = sdo::upload_u8(bus, nid, 0x2101, 0, timeout)
        .await
        .map_err(|e| format!("read 0x2101:00 failed: {e}"))?;

    match max_subindex {
        1 => {
            let nominal_bitrate = sdo::upload_u32(bus, nid, 0x2100, 1, timeout)
                .await
                .map_err(|e| format!("read 0x2100:01 failed: {e}"))?;
            Ok(DeviceCanConfig {
                stored_node_id,
                nominal_bitrate,
                data_bitrate: None,
                transmit_pdo_brs: None,
            })
        }
        3 => {
            let nominal_bitrate = sdo::upload_u32(bus, nid, 0x2100, 1, timeout)
                .await
                .map_err(|e| format!("read 0x2100:01 failed: {e}"))?;
            let data_bitrate = sdo::upload_u32(bus, nid, 0x2100, 2, timeout)
                .await
                .map_err(|e| format!("read 0x2100:02 failed: {e}"))?;
            let transmit_pdo_brs = sdo::upload_u8(bus, nid, 0x2100, 3, timeout)
                .await
                .map_err(|e| format!("read 0x2100:03 failed: {e}"))?;
            Ok(DeviceCanConfig {
                stored_node_id,
                nominal_bitrate,
                data_bitrate: Some(data_bitrate),
                transmit_pdo_brs: Some(parse_bool(0x2100, 3, transmit_pdo_brs)?),
            })
        }
        _ => unreachable!("0x2100 shape validated above"),
    }
}

fn parse_bool(index: u16, subindex: u8, raw: u8) -> std::result::Result<bool, String> {
    match raw {
        0 => Ok(false),
        1 => Ok(true),
        other => Err(format!(
            "0x{index:04X}:{subindex:02X} expected 0 or 1, got {other}"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashMap, HashSet};
    use std::sync::Mutex;

    use async_trait::async_trait;
    use can_transport::{CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanRx};
    use tokio::sync::mpsc;

    fn identity(vendor_id: u32, product_code: u32) -> MotorIdentity {
        MotorIdentity {
            node_id: 1,
            vendor_id,
            product_code,
            revision_number: 0,
            serial_number: 0,
            product_name: None,
        }
    }

    #[test]
    fn object_2001_dialect_requires_an_exact_tuple() {
        for product_code in [0xAAAA_0001, 0xAAAA_0002, 0xAAAA_0005] {
            assert_eq!(
                dialect_for(&identity(0x4859_444C, product_code)),
                Some(CanConfigDialect::Object2001)
            );
        }
        for product_code in [0x6C64_AE8D, 0x6C64_BC78, 0x6C64_BCAA] {
            assert_eq!(
                dialect_for(&identity(0x0068_6578, product_code)),
                Some(CanConfigDialect::Object2001)
            );
        }
        assert_eq!(dialect_for(&identity(0x4859_444C, 0x6C64_AE8D)), None);
        assert_eq!(dialect_for(&identity(0x6865_786D, 0x6C64_AE8D)), None);
        assert_eq!(dialect_for(&identity(0x4859_444C, 0xAAAA_9999)), None);
        assert_eq!(dialect_for(&identity(0x0068_6578, 0xAAAA_0001)), None);
        assert_eq!(dialect_for(&identity(0x0068_6578, 0x6C64_BC79)), None);
    }

    #[test]
    fn object_2100_dialect_requires_an_exact_tuple() {
        for product_code in [0x0069_6D75, 0x6169_6D75, 0x006C_6674] {
            assert_eq!(
                dialect_for(&identity(0x6865_786D, product_code)),
                Some(CanConfigDialect::Object2100)
            );
        }
        assert_eq!(dialect_for(&identity(0x6865_786D, 0xDEAD_BEEF)), None);
        assert_eq!(dialect_for(&identity(0x0068_6578, 0x006C_6674)), None);
    }

    #[test]
    fn schema_tuples_are_unique_disjoint_and_registered_as_known_devices() {
        let object_2001: HashSet<_> = OBJECT_2001_IDENTITIES.iter().copied().collect();
        let object_2100: HashSet<_> = OBJECT_2100_IDENTITIES.iter().copied().collect();

        assert_eq!(
            object_2001.len(),
            OBJECT_2001_IDENTITIES.len(),
            "0x2001 identity table contains a duplicate tuple"
        );
        assert_eq!(
            object_2100.len(),
            OBJECT_2100_IDENTITIES.len(),
            "0x2100 identity table contains a duplicate tuple"
        );
        assert!(
            object_2001.is_disjoint(&object_2100),
            "a tuple cannot select both 0x2001 and 0x2100"
        );

        for &(vendor_id, product_code) in
            OBJECT_2001_IDENTITIES.iter().chain(OBJECT_2100_IDENTITIES)
        {
            let known = crate::cia402::known_devices::lookup_known_device(vendor_id, product_code);
            assert!(
                known.is_some(),
                "schema tuple 0x{vendor_id:08X}/0x{product_code:08X} lacks an exact KNOWN_DEVICES entry"
            );
        }
    }

    #[test]
    fn bool_fields_only_accept_zero_or_one() {
        assert_eq!(parse_bool(0x2100, 3, 0), Ok(false));
        assert_eq!(parse_bool(0x2100, 3, 1), Ok(true));
        assert!(parse_bool(0x2100, 3, 2).is_err());
    }

    fn settings_snapshot(
        dialect: CanConfigDialect,
        stored_node_id: u8,
        data_bitrate: Option<u32>,
        transmit_pdo_brs: Option<bool>,
    ) -> DeviceSettingsSnapshot {
        DeviceSettingsSnapshot {
            dialect,
            config: DeviceCanConfig {
                stored_node_id,
                nominal_bitrate: 1_000_000,
                data_bitrate,
                transmit_pdo_brs,
            },
        }
    }

    #[test]
    fn object_2001_write_plan_orders_node_last_and_save_after_every_field() {
        let current = settings_snapshot(
            CanConfigDialect::Object2001,
            1,
            Some(5_000_000),
            Some(false),
        );
        let plan = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 42,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(2_000_000),
                transmit_pdo_brs: Some(true),
            },
        )
        .unwrap();

        assert_eq!(
            plan.writes,
            vec![
                SdoWrite::u32(0x2001, 3, 2_000_000),
                SdoWrite::u8(0x2001, 4, 1),
                SdoWrite::u8(0x2001, 1, 42),
                SdoWrite::u32(0x1010, 1, 0x6576_6173),
            ]
        );
        assert_eq!(
            plan.result,
            DeviceSettingsResult {
                changed: true,
                restart_required: true,
                persistence_pending: false,
                brs_applied_immediately: false,
            }
        );
    }

    #[test]
    fn unchanged_object_2001_plan_retries_store_without_reporting_field_changes() {
        let current = settings_snapshot(
            CanConfigDialect::Object2001,
            8,
            Some(4_000_000),
            Some(true),
        );
        let plan = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 8,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(4_000_000),
                transmit_pdo_brs: Some(true),
            },
        )
        .unwrap();

        assert_eq!(
            plan.writes,
            vec![SdoWrite::u32(0x1010, 1, 0x6576_6173)]
        );
        assert!(!plan.result.changed);
        assert!(plan.result.restart_required);
    }

    #[test]
    fn object_2100_brs_only_plan_is_immediate_and_persistence_pending() {
        let current = settings_snapshot(
            CanConfigDialect::Object2100,
            31,
            Some(5_000_000),
            Some(false),
        );
        let plan = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 31,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(5_000_000),
                transmit_pdo_brs: Some(true),
            },
        )
        .unwrap();

        assert_eq!(
            plan.writes,
            vec![
                SdoWrite::u32(0x2100, 1, 1_000_000),
                SdoWrite::u32(0x2100, 2, 5_000_000),
                SdoWrite::u8(0x2100, 3, 1),
                SdoWrite::u8(0x2101, 0, 31),
            ]
        );
        assert_eq!(
            plan.result,
            DeviceSettingsResult {
                changed: true,
                restart_required: true,
                persistence_pending: true,
                brs_applied_immediately: true,
            }
        );
    }

    #[test]
    fn unchanged_object_2100_plan_rewrites_full_target_and_kicks_persistence() {
        let current = settings_snapshot(
            CanConfigDialect::Object2100,
            31,
            Some(4_000_000),
            Some(true),
        );
        let plan = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 31,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(4_000_000),
                transmit_pdo_brs: Some(true),
            },
        )
        .unwrap();

        assert_eq!(
            plan.writes,
            vec![
                SdoWrite::u32(0x2100, 1, 1_000_000),
                SdoWrite::u32(0x2100, 2, 4_000_000),
                SdoWrite::u8(0x2100, 3, 1),
                SdoWrite::u8(0x2101, 0, 31),
            ]
        );
        assert_eq!(
            plan.result,
            DeviceSettingsResult {
                changed: false,
                restart_required: true,
                persistence_pending: true,
                brs_applied_immediately: false,
            }
        );
    }

    #[test]
    fn unchanged_classic_2100_plan_rewrites_nominal_then_node() {
        let current = settings_snapshot(CanConfigDialect::Object2100, 16, None, None);
        let plan = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 16,
                nominal_bitrate: 1_000_000,
                data_bitrate: None,
                transmit_pdo_brs: None,
            },
        )
        .unwrap();

        assert_eq!(
            plan.writes,
            vec![
                SdoWrite::u32(0x2100, 1, 1_000_000),
                SdoWrite::u8(0x2101, 0, 16),
            ]
        );
        assert!(!plan.result.changed);
        assert!(plan.result.restart_required);
        assert!(plan.result.persistence_pending);
    }

    #[test]
    fn object_2100_classic_shape_rejects_fd_fields() {
        let current = settings_snapshot(CanConfigDialect::Object2100, 16, None, None);
        let error = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 16,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(1_000_000),
                transmit_pdo_brs: Some(false),
            },
        )
        .unwrap_err();
        assert!(error.contains("Classic-only"));
    }

    #[test]
    fn settings_plan_rejects_nonstandard_bitrates() {
        let current = settings_snapshot(
            CanConfigDialect::Object2100,
            31,
            Some(5_000_000),
            Some(false),
        );
        let bad_nominal = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 31,
                nominal_bitrate: 500_000,
                data_bitrate: Some(5_000_000),
                transmit_pdo_brs: Some(false),
            },
        )
        .unwrap_err();
        assert!(bad_nominal.contains("nominal bitrate"));

        let bad_data = build_device_settings_plan(
            &current,
            DeviceSettingsUpdate {
                new_node_id: 31,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(3_000_000),
                transmit_pdo_brs: Some(false),
            },
        )
        .unwrap_err();
        assert!(bad_data.contains("data bitrate"));
    }

    #[test]
    fn role_node_id_ranges_are_exact_tuple_scoped() {
        assert!(validate_role_node_id(0x6865_786D, 0x0069_6D75, 31).is_ok());
        assert!(validate_role_node_id(0x6865_786D, 0x0069_6D75, 30).is_err());
        assert!(validate_role_node_id(0x6865_786D, 0x006C_6674, 20).is_ok());
        assert!(validate_role_node_id(0x6865_786D, 0x006C_6674, 21).is_err());
        assert!(validate_role_node_id(0x4859_444C, 0xAAAA_0001, 99).is_ok());
    }

    #[tokio::test]
    async fn motor_3525_discovery_reports_available_can_config() {
        let motor = identity(0x0068_6578, 0x6C64_AE8D);
        let bus = ScriptedSdoBus::new([
            ((0x2001, 0), vec![4]),
            ((0x2001, 1), vec![1]),
            ((0x2001, 2), 1_000_000u32.to_le_bytes().to_vec()),
            ((0x2001, 3), 5_000_000u32.to_le_bytes().to_vec()),
            ((0x2001, 4), vec![1]),
        ]);
        let status = read_device_can_config(&bus, &motor, Duration::from_millis(20)).await;
        assert_eq!(
            status,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                stored_node_id: 1,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(5_000_000),
                transmit_pdo_brs: Some(true),
            })
        );
        assert_eq!(
            bus.requests(),
            vec![
                (0x2001, 0),
                (0x2001, 1),
                (0x2001, 2),
                (0x2001, 3),
                (0x2001, 4)
            ]
        );
        assert_eq!(
            crate::cia402::known_devices::human_friendly_name(&motor),
            "hexmeow 3525"
        );
    }

    #[tokio::test]
    async fn reads_object_2001_profile() {
        let bus = ScriptedSdoBus::new([
            ((0x2001, 0), vec![4]),
            ((0x2001, 1), vec![1]),
            ((0x2001, 2), 1_000_000u32.to_le_bytes().to_vec()),
            ((0x2001, 3), 5_000_000u32.to_le_bytes().to_vec()),
            ((0x2001, 4), vec![1]),
        ]);
        let status = read_device_can_config(
            &bus,
            &identity(0x4859_444C, 0xAAAA_0001),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(
            status,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                stored_node_id: 1,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(5_000_000),
                transmit_pdo_brs: Some(true),
            })
        );
        assert_eq!(
            bus.requests(),
            vec![
                (0x2001, 0),
                (0x2001, 1),
                (0x2001, 2),
                (0x2001, 3),
                (0x2001, 4)
            ]
        );
    }

    #[tokio::test]
    async fn object_2100_max_subindex_one_is_explicitly_classic_only() {
        let bus = ScriptedSdoBus::new([
            ((0x2100, 0), vec![1]),
            ((0x2101, 0), vec![16]),
            ((0x2100, 1), 1_000_000u32.to_le_bytes().to_vec()),
        ]);
        let status = read_device_can_config(
            &bus,
            &identity(0x6865_786D, 0x006C_6674),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(
            status,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                stored_node_id: 16,
                nominal_bitrate: 1_000_000,
                data_bitrate: None,
                transmit_pdo_brs: None,
            })
        );
        assert_eq!(
            bus.requests(),
            vec![(0x2100, 0), (0x2101, 0), (0x2100, 1)]
        );
    }

    #[tokio::test]
    async fn reads_object_2100_fd_profile() {
        let bus = ScriptedSdoBus::new([
            ((0x2100, 0), vec![3]),
            ((0x2101, 0), vec![31]),
            ((0x2100, 1), 1_000_000u32.to_le_bytes().to_vec()),
            ((0x2100, 2), 4_000_000u32.to_le_bytes().to_vec()),
            ((0x2100, 3), vec![0]),
        ]);
        let status = read_device_can_config(
            &bus,
            &identity(0x6865_786D, 0x6169_6D75),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(
            status,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                stored_node_id: 31,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(4_000_000),
                transmit_pdo_brs: Some(false),
            })
        );
        assert_eq!(
            bus.requests(),
            vec![
                (0x2100, 0),
                (0x2101, 0),
                (0x2100, 1),
                (0x2100, 2),
                (0x2100, 3)
            ]
        );
    }

    #[tokio::test]
    async fn unknown_2100_shape_is_read_failed_not_classic() {
        let bus = ScriptedSdoBus::new([((0x2100, 0), vec![2])]);
        let status = read_device_can_config(
            &bus,
            &identity(0x6865_786D, 0x0069_6D75),
            Duration::from_millis(20),
        )
        .await;

        assert!(matches!(
            status,
            DeviceCanConfigStatus::ReadFailed { reason }
                if reason.contains("expected max sub-index 1 or 3")
        ));
        assert_eq!(bus.requests(), vec![(0x2100, 0)]);
    }

    #[tokio::test]
    async fn unknown_tuple_is_unsupported_without_any_sdo_probe() {
        let bus = ScriptedSdoBus::new([]);
        let status = read_device_can_config(
            &bus,
            &identity(0x0068_6578, 0xAAAA_0001),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(status, DeviceCanConfigStatus::Unsupported);
        assert!(bus.requests().is_empty());
    }

    #[test]
    fn read_failed_status_keeps_identity_and_identified_lifecycle() {
        let entry = MotorEntry::new(1);
        let identity = identity(0x6865_786D, 0x006C_6674);
        assert!(begin_device_can_config_read(&entry, 0, &identity));
        assert!(finish_device_can_config_read(
            &entry,
            0,
            DeviceCanConfigStatus::ReadFailed {
                reason: "simulated SDO failure".to_string(),
            },
        ));

        let inner = entry.inner.lock().unwrap();
        assert_eq!(inner.identity.as_ref(), Some(&identity));
        assert_eq!(inner.lifecycle, MotorLifecycle::Identified);
        assert!(matches!(
            inner.can_config,
            DeviceCanConfigStatus::ReadFailed { ref reason }
                if reason == "simulated SDO failure"
        ));
    }

    struct ScriptedSdoBus {
        values: HashMap<(u16, u8), Vec<u8>>,
        requests: Mutex<Vec<(u16, u8)>>,
        response_tx: Mutex<Option<mpsc::UnboundedSender<CanFrame>>>,
    }

    impl ScriptedSdoBus {
        fn new<const N: usize>(values: [((u16, u8), Vec<u8>); N]) -> Self {
            Self {
                values: values.into_iter().collect(),
                requests: Mutex::new(Vec::new()),
                response_tx: Mutex::new(None),
            }
        }

        fn requests(&self) -> Vec<(u16, u8)> {
            self.requests.lock().unwrap().clone()
        }
    }

    #[async_trait]
    impl CanBus for ScriptedSdoBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            let request = frame.data();
            if request.len() != 8 || request[0] != 0x40 {
                return Err(CanIoError::backend(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "test bus expected an expedited SDO upload request",
                )));
            }
            let nid = match frame.id() {
                CanId::Standard(id) if (0x601..=0x67F).contains(&id) => (id - 0x600) as u8,
                _ => {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "test bus received an invalid RSDO COB-ID",
                    )))
                }
            };
            let index = u16::from_le_bytes([request[1], request[2]]);
            let subindex = request[3];
            self.requests.lock().unwrap().push((index, subindex));

            let mut response = [0u8; 8];
            response[1] = request[1];
            response[2] = request[2];
            response[3] = subindex;
            match self.values.get(&(index, subindex)) {
                Some(value) if (1..=4).contains(&value.len()) => {
                    response[0] = 0x43 | (((4 - value.len()) as u8) << 2);
                    response[4..4 + value.len()].copy_from_slice(value);
                }
                Some(value) => {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        format!("test value has unsupported length {}", value.len()),
                    )))
                }
                None => {
                    response[0] = 0x80;
                    response[4..8].copy_from_slice(&0x0602_0000u32.to_le_bytes());
                }
            }

            let response = CanFrame::new_data(0x580u16 + nid as u16, &response)?;
            self.response_tx
                .lock()
                .unwrap()
                .as_ref()
                .ok_or(CanIoError::Disconnected)?
                .send(response)
                .map_err(|_| CanIoError::Disconnected)
        }

        async fn subscribe(&self, _filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            *self.response_tx.lock().unwrap() = Some(tx);
            Ok(Box::new(ScriptedSdoRx { rx }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    struct ScriptedSdoRx {
        rx: mpsc::UnboundedReceiver<CanFrame>,
    }

    #[async_trait]
    impl CanRx for ScriptedSdoRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            self.rx.recv().await.ok_or(CanIoError::Disconnected)
        }

        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            match self.rx.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
            }
        }
    }
}
