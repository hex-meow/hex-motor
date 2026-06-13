//! 已知 CiA402 电机的 0x1018 标识表 + 人类可读名称查找。
//!
//! ## 维护指南
//!
//! 当你确认了某型号的 `(vendor_id, product_code)`，在 [`KNOWN_DEVICES`]
//! 里加一条 [`KnownDevice`]：
//!
//! - `product_code: Some(0xAAAA0001)` —— 精确匹配该型号
//! - `product_code: None`             —— 匹配该 vendor 下任意 product（兜底）
//!
//! 查询时优先返回"精确匹配"，没有再退到"vendor 兜底"。
//!
//! ## 人类可读名称的优先级
//!
//! 见 [`human_friendly_name`]：
//! 1. 电机自己的 0x1008 (`MotorIdentity::product_name`) 非空，用它
//! 2. 否则查 [`KNOWN_DEVICES`]
//! 3. 否则返回一个 `Unknown CiA402 (vendor 0x..., product 0x...)` 的兜底字符串

use crate::types::MotorIdentity;

/// 一条已知设备记录。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KnownDevice {
    pub vendor_id: u32,
    /// `None` 表示匹配该 vendor 下任意 product code。
    pub product_code: Option<u32>,
    /// 默认人类可读名称。可以随时改成更具体的型号名。
    pub name: &'static str,
}

/// 已知设备表。按 vendor 分组列出，方便维护。
///
/// 用户在自己的下游 fork 里可以**直接编辑此文件**追加 `(vendor_id,
/// product_code, name)`：精确匹配优先，同 vendor 的 `product_code: None`
/// 兜底项次之。
pub const KNOWN_DEVICES: &[KnownDevice] = &[
    // ===== HexMeow (vendor 0x00686578 == "hex" + 0x00 LE) =====
    KnownDevice {
        vendor_id: 0x0068_6578,
        product_code: None,
        name: "HexMeow Motor",
    },
    // ===== vendor 0x4859_444C - HexMeow CiA402 系列电机 =====
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: Some(0xAAAA_0001),
        name: "CiA402 HEX-4310",
    },
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: Some(0xAAAA_0002),
        name: "CiA402 HEX-4342P",
    },
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: Some(0xAAAA_0005),
        name: "CiA402 HEX-4360P",
    },
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: None,
        name: "CiA402 HEX Motor (未知型号)",
    },
];

/// 在 [`KNOWN_DEVICES`] 中查 `(vendor_id, product_code)`。
///
/// 优先返回精确匹配（`product_code == Some(p)`）；如果没有，再尝试同
/// vendor 下的 `product_code == None` 兜底项。
pub fn lookup_known_device(vendor_id: u32, product_code: u32) -> Option<&'static KnownDevice> {
    KNOWN_DEVICES
        .iter()
        .find(|d| d.vendor_id == vendor_id && d.product_code == Some(product_code))
        .or_else(|| {
            KNOWN_DEVICES
                .iter()
                .find(|d| d.vendor_id == vendor_id && d.product_code.is_none())
        })
}

/// 取一个"展示给人看"的电机名称。
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
        "Unknown CiA402 device (vendor 0x{:08X}, product 0x{:08X})",
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
    fn lookup_hexmeow_wildcard_matches_any_product() {
        let d = lookup_known_device(0x0068_6578, 0x1234_5678).unwrap();
        assert_eq!(d.name, "HexMeow Motor");
        assert_eq!(d.product_code, None);
    }

    #[test]
    fn lookup_hex_series_exact_products() {
        let d = lookup_known_device(0x4859_444C, 0xAAAA_0001).unwrap();
        assert_eq!(d.name, "CiA402 HEX-4310");
        assert_eq!(d.product_code, Some(0xAAAA_0001));

        let d = lookup_known_device(0x4859_444C, 0xAAAA_0002).unwrap();
        assert_eq!(d.name, "CiA402 HEX-4342P");

        let d = lookup_known_device(0x4859_444C, 0xAAAA_0005).unwrap();
        assert_eq!(d.name, "CiA402 HEX-4360P");
    }

    #[test]
    fn lookup_hex_series_falls_back_to_vendor_wildcard() {
        // 该 vendor 下不存在的 product code 应该走 "(未知型号)" 兜底。
        let d = lookup_known_device(0x4859_444C, 0xDEAD_BEEF).unwrap();
        assert_eq!(d.name, "CiA402 HEX Motor (未知型号)");
        assert_eq!(d.product_code, None);
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
        let id = identity(0x0068_6578, 0xAAAA_0002, None);
        assert_eq!(human_friendly_name(&id), "HexMeow Motor");
    }

    #[test]
    fn name_ignores_empty_0x1008() {
        let id = identity(0x0068_6578, 0xAAAA_0002, Some(""));
        assert_eq!(human_friendly_name(&id), "HexMeow Motor");
    }

    #[test]
    fn name_ignores_whitespace_only_0x1008() {
        let id = identity(0x0068_6578, 0x9999_9999, Some("   \t\n"));
        assert_eq!(human_friendly_name(&id), "HexMeow Motor");
    }

    #[test]
    fn name_falls_back_to_generic() {
        let id = identity(0xDEAD_BEEF, 0xCAFEBABE, None);
        let n = human_friendly_name(&id);
        assert!(n.contains("Unknown CiA402"));
        assert!(n.contains("0xDEADBEEF"));
        assert!(n.contains("0xCAFEBABE"));
    }
}
