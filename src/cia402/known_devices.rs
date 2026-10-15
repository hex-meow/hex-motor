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
//!
//! ## 固件版本门控
//!
//! `(vendor_id, product_code)` 只说明"是哪个型号"，不说明"对象字典是哪一版"。
//! 同一型号出过对象字典不兼容的两版固件时，用 [`KnownDevice::min_revision_number`]
//! 声明本驱动支持的最低 `0x1018:03`，并由 [`check_revision`] 在 `initialize()`
//! 之前拒掉过旧的固件。本驱动**只实现最新一版对象字典**，不按固件版本分叉。

use std::fmt;

use crate::types::MotorIdentity;

/// 一条已知设备记录。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KnownDevice {
    pub vendor_id: u32,
    pub product_code: u32,
    /// 默认人类可读名称。可以随时改成更具体的型号名。
    pub name: &'static str,
    /// 本驱动支持的最低 `0x1018:03` revision number。
    ///
    /// `0` = 该型号没有已知的不兼容旧固件，不做门控。见 [`check_revision`]。
    pub min_revision_number: u32,
}

/// vendor `0x4859444C` 系列 CiA402 电机所需的最低 `0x1018:03`。
///
/// revision ≤ 7 是该系列的**旧版对象字典**，与本驱动实现的新版互不兼容：
///
/// - `0x6076` 峰值力矩：旧版 `Uint32`（mNm），新版 `REAL32`（Nm）。按新版读旧版
///   会把整数 mNm 位模式当成 f32，得到非规格化的垃圾值（如 8658 → 1.2e-41）。
/// - `0x2003:01..:03` MIT 目标位置/速度/前馈力矩：旧版 `Int32` Q21 定点，新版 `REAL32`；
///   `0x2003:07` KP/KD 尺度因子同理。
/// - `0x2004` 压缩型 MIT 控制参数、`0x2040` 短接制动使能：旧版**不存在**，
///   对其 SDO 写会以 `0x06020000 (ObjectDoesNotExist)` 中止。
///
/// 处理办法只有一个：把设备升级到 revision ≥ 8 的固件。
pub const MIN_REVISION_0X4859444C: u32 = 8;

/// 已知设备表。按 vendor 分组列出，方便维护。
///
/// 用户在自己的下游 fork 里可以**直接编辑此文件**追加精确的 `(vendor_id,
/// product_code, name, min_revision_number)`。
pub const KNOWN_DEVICES: &[KnownDevice] = &[
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: 0xAAAA_0001,
        name: "CiA402 HEX-4310",
        min_revision_number: MIN_REVISION_0X4859444C,
    },
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: 0xAAAA_0002,
        name: "CiA402 HEX-4342P",
        min_revision_number: MIN_REVISION_0X4859444C,
    },
    KnownDevice {
        vendor_id: 0x4859_444C,
        product_code: 0xAAAA_0005,
        name: "CiA402 HEX-4360P",
        min_revision_number: MIN_REVISION_0X4859444C,
    },
    KnownDevice {
        vendor_id: 0x0068_6578,
        product_code: 0x6C64_AE8D,
        name: "hexmeow 3525",
        min_revision_number: 0,
    },
    KnownDevice {
        vendor_id: 0x0068_6578,
        product_code: 0x6C64_BC78,
        name: "hexmeow 4310",
        min_revision_number: 0,
    },
    KnownDevice {
        vendor_id: 0x0068_6578,
        product_code: 0x6C64_BCAA,
        name: "hexmeow 4342",
        min_revision_number: 0,
    },
    KnownDevice {
        vendor_id: 0x6865_786D,
        product_code: 0x0069_6D75,
        name: "HexMeow IMU (bench/demo)",
        min_revision_number: 0,
    },
    KnownDevice {
        vendor_id: 0x6865_786D,
        product_code: 0x6169_6D75,
        name: "HexMeow Arm IMU",
        min_revision_number: 0,
    },
    KnownDevice {
        vendor_id: 0x6865_786D,
        product_code: 0x006C_6674,
        name: "HexMeow Lift Driver",
        min_revision_number: 0,
    },
];

/// 在 [`KNOWN_DEVICES`] 中查 `(vendor_id, product_code)`。
pub fn lookup_known_device(vendor_id: u32, product_code: u32) -> Option<&'static KnownDevice> {
    KNOWN_DEVICES
        .iter()
        .find(|d| d.vendor_id == vendor_id && d.product_code == product_code)
}

/// 已知型号的固件太旧，对象字典与本驱动不兼容。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnsupportedRevision {
    pub vendor_id: u32,
    pub product_code: u32,
    /// 设备实际报告的 `0x1018:03`。
    pub revision_number: u32,
    /// [`KNOWN_DEVICES`] 为该型号声明的最低 revision。
    pub min_revision_number: u32,
}

impl fmt::Display for UnsupportedRevision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "device (vendor 0x{:08X}, product 0x{:08X}) reports 0x1018:03 revision {} \
             but this driver requires >= {}; its object dictionary is an older, \
             incompatible revision — upgrade the device firmware",
            self.vendor_id, self.product_code, self.revision_number, self.min_revision_number
        )
    }
}

impl std::error::Error for UnsupportedRevision {}

/// 检查 `0x1018:03` 是否达到该已知型号的最低 revision 要求。
///
/// **不在 [`KNOWN_DEVICES`] 里的设备一律返回 `Ok(())`**：本函数只做"已知型号的
/// 固件版本门控"，"这个 identity 认不认识"由调用方的白名单逻辑负责，两件事不要
/// 在这里混。`min_revision_number == 0` 的型号同样直接放行。
pub fn check_revision(identity: &MotorIdentity) -> Result<(), UnsupportedRevision> {
    let Some(known) = lookup_known_device(identity.vendor_id, identity.product_code) else {
        return Ok(());
    };
    if identity.revision_number >= known.min_revision_number {
        return Ok(());
    }
    Err(UnsupportedRevision {
        vendor_id: identity.vendor_id,
        product_code: identity.product_code,
        revision_number: identity.revision_number,
        min_revision_number: known.min_revision_number,
    })
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
        identity_rev(vendor, product, name, 0)
    }

    fn identity_rev(
        vendor: u32,
        product: u32,
        name: Option<&str>,
        revision_number: u32,
    ) -> MotorIdentity {
        MotorIdentity {
            node_id: 0x10,
            vendor_id: vendor,
            product_code: product,
            revision_number,
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

        let d = lookup_known_device(0x0068_6578, 0x6C64_BC78).unwrap();
        assert_eq!(d.name, "hexmeow 4310");
        let d = lookup_known_device(0x0068_6578, 0x6C64_BCAA).unwrap();
        assert_eq!(d.name, "hexmeow 4342");
    }

    #[test]
    fn vendor_0x4859444c_products_all_require_the_same_minimum_revision() {
        for product in [0xAAAA_0001, 0xAAAA_0002, 0xAAAA_0005] {
            let d = lookup_known_device(0x4859_444C, product).unwrap();
            assert_eq!(d.min_revision_number, MIN_REVISION_0X4859444C);
            assert_eq!(d.min_revision_number, 8);
        }
    }

    #[test]
    fn revision_gate_rejects_below_minimum_and_accepts_at_minimum() {
        // 旧版对象字典（0x6076 为 Uint32、无 0x2004/0x2040）必须拒绝。
        for revision in 0..MIN_REVISION_0X4859444C {
            let id = identity_rev(0x4859_444C, 0xAAAA_0001, None, revision);
            let err = check_revision(&id).expect_err("revision below the minimum must be rejected");
            assert_eq!(err.revision_number, revision);
            assert_eq!(err.min_revision_number, MIN_REVISION_0X4859444C);
        }
        // 边界与更高版本都放行。
        for revision in [
            MIN_REVISION_0X4859444C,
            MIN_REVISION_0X4859444C + 1,
            u32::MAX,
        ] {
            let id = identity_rev(0x4859_444C, 0xAAAA_0001, None, revision);
            assert!(check_revision(&id).is_ok());
        }
    }

    #[test]
    fn revision_gate_ignores_devices_without_a_declared_minimum() {
        // min_revision_number == 0 的已知型号不受门控。
        let id = identity_rev(0x0068_6578, 0x6C64_BC78, None, 0);
        assert!(check_revision(&id).is_ok());
    }

    #[test]
    fn revision_gate_does_not_judge_unknown_devices() {
        // "认不认识这个 identity" 是调用方白名单的事，不在本门控里混判。
        let id = identity_rev(0xDEAD_BEEF, 0xCAFE_BABE, None, 0);
        assert!(check_revision(&id).is_ok());
    }

    #[test]
    fn unsupported_revision_message_names_both_revisions() {
        let id = identity_rev(0x4859_444C, 0xAAAA_0001, None, 7);
        let err = check_revision(&id).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("revision 7"), "{text}");
        assert!(text.contains(">= 8"), "{text}");
        assert!(text.contains("upgrade the device firmware"), "{text}");
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
