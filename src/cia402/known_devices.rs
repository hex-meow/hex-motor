//! 已知 CANopen 设备的 0x1018 标识表 + 人类可读名称查找。
//!
//! ## 维护指南
//!
//! 当你确认了某型号的 `(vendor_id, product_code)`，在 [`KNOWN_DEVICES`]
//! 里加一条 [`KnownDevice`]：
//!
//! `vendor_id` 和 `product_code` 必须同时精确匹配。不能仅凭 vendor ID
//! 推断产品类型、对象字典或展示名称。
//!
//! ## 人类可读名称的优先级
//!
//! 见 [`human_friendly_name`]：
//! 1. 电机自己的 0x1008 (`MotorIdentity::product_name`) 非空，用它
//! 2. 否则查 [`KNOWN_DEVICES`]
//! 3. 否则返回一个 `Unknown CANopen (vendor 0x..., product 0x...)` 的兜底字符串

use crate::types::MotorIdentity;

/// 一条已知设备记录。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KnownDevice {
    pub vendor_id: u32,
    pub product_code: u32,
    /// 默认人类可读名称。可以随时改成更具体的型号名。
    pub name: &'static str,
}

/// 已知设备表。按 vendor 分组列出，方便维护。
///
/// 用户在自己的下游 fork 里可以**直接编辑此文件**追加精确的 `(vendor_id,
/// product_code, name)`。
pub const KNOWN_DEVICES: &[KnownDevice] = &[
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: 0xAAAA_0001,
        name: "CiA402 HEX-4310",
    },
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: 0xAAAA_0002,
        name: "CiA402 HEX-4342P",
    },
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: 0xAAAA_0005,
        name: "CiA402 HEX-4360P",
    },
    KnownDevice {
        vendor_id: 0x6865_786D,
        product_code: 0x0069_6D75,
        name: "HexMeow IMU (bench/demo)",
    },
    KnownDevice {
        vendor_id: 0x6865_786D,
        product_code: 0x6169_6D75,
        name: "HexMeow Arm IMU",
    },
    KnownDevice {
        vendor_id: 0x6865_786D,
        product_code: 0x006C_6674,
        name: "HexMeow Lift Driver",
    },
];

/// 在 [`KNOWN_DEVICES`] 中查 `(vendor_id, product_code)`。
pub fn lookup_known_device(vendor_id: u32, product_code: u32) -> Option<&'static KnownDevice> {
    KNOWN_DEVICES
        .iter()
        .find(|d| d.vendor_id == vendor_id && d.product_code == product_code)
}

/// 取一个"展示给人看"的设备名称。
///
/// 优先级见模块文档。
pub fn human_friendly_name(identity: &MotorIdentity) -> String {
    // 1. 电机自己说的 (0x1008 Manufacturer device name)
    if let Some(name) = identity.product_name.as_deref() {
        let trimmed = name.trim();
        if !trimmed.is_empty() {
            return trimmed.to_string();
        }
    }

    // 2. 已知设备表
    if let Some(known) = lookup_known_device(identity.vendor_id, identity.product_code) {
        return known.name.to_string();
    }

    // 3. 兜底
    format!(
        "Unknown CANopen device (vendor 0x{:08X}, product 0x{:08X})",
        identity.vendor_id, identity.product_code
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity(vendor: u32, product: u32, name: Option<&str>) -> MotorIdentity {
        MotorIdentity {
            node_id: 0x10,
            vendor_id: vendor,
            product_code: product,
            revision_number: 0,
            serial_number: 0,
            product_name: name.map(|s| s.to_string()),
        }
    }

    #[test]
    fn lookup_hex_series_exact_products() {
        let d = lookup_known_device(0x4859_444C, 0xAAAA_0001).unwrap();
        assert_eq!(d.name, "CiA402 HEX-4310");
        assert_eq!(d.product_code, 0xAAAA_0001);

        let d = lookup_known_device(0x4859_444C, 0xAAAA_0002).unwrap();
        assert_eq!(d.name, "CiA402 HEX-4342P");

        let d = lookup_known_device(0x4859_444C, 0xAAAA_0005).unwrap();
        assert_eq!(d.name, "CiA402 HEX-4360P");
    }

    #[test]
    fn lookup_never_falls_back_by_vendor() {
        assert!(lookup_known_device(0x4859_444C, 0xDEAD_BEEF).is_none());
        assert!(lookup_known_device(0x0068_6578, 0xAAAA_0001).is_none());
    }

    #[test]
    fn lookup_unknown_vendor_returns_none() {
        assert!(lookup_known_device(0xDEAD_BEEF, 0).is_none());
    }

    #[test]
    fn name_prefers_0x1008_when_present() {
        let id = identity(0x0068_6578, 0xAAAA_0002, Some("Custom Label"));
        assert_eq!(human_friendly_name(&id), "Custom Label");
    }

    #[test]
    fn name_uses_table_when_no_0x1008() {
        let id = identity(0x6865_786D, 0x006C_6674, None);
        assert_eq!(human_friendly_name(&id), "HexMeow Lift Driver");
    }

    #[test]
    fn name_ignores_empty_0x1008() {
        let id = identity(0x0068_6578, 0xAAAA_0002, Some(""));
        assert!(human_friendly_name(&id).starts_with("Unknown CANopen"));
    }

    #[test]
    fn name_ignores_whitespace_only_0x1008() {
        let id = identity(0x0068_6578, 0x9999_9999, Some("   \t\n"));
        assert!(human_friendly_name(&id).starts_with("Unknown CANopen"));
    }

    #[test]
    fn name_falls_back_to_generic() {
        let id = identity(0xDEAD_BEEF, 0xCAFEBABE, None);
        let n = human_friendly_name(&id);
        assert!(n.contains("Unknown CANopen"));
        assert!(n.contains("0xDEADBEEF"));
        assert!(n.contains("0xCAFEBABE"));
    }
}
