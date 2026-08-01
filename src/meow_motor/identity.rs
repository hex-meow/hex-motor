//! 精确的 0x1018 身份识别。

use crate::types::MotorIdentity;

/// 该产品族在 0x1018:01 中报告的 vendor ID。
pub const VENDOR_ID: u32 = 0x0068_6578;
/// 4310 的 0x1018:02 product code。
pub const PRODUCT_CODE_4310: u32 = 0x6C64_BC78;
/// 4342 的 0x1018:02 product code。
pub const PRODUCT_CODE_4342: u32 = 0x6C64_BCAA;

/// 已支持的 Meow Motor 型号。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MeowMotorModel {
    Model4310,
    Model4342,
}

impl MeowMotorModel {
    /// GUI/日志使用的固定公开显示名。它有意覆盖设备可选的 0x1008 文本。
    pub const fn display_name(self) -> &'static str {
        match self {
            Self::Model4310 => "hexmeow 4310",
            Self::Model4342 => "hexmeow 4342",
        }
    }

    pub const fn product_code(self) -> u32 {
        match self {
            Self::Model4310 => PRODUCT_CODE_4310,
            Self::Model4342 => PRODUCT_CODE_4342,
        }
    }
}

/// 只按完整 `(vendor_id, product_code)` 查找；不按 vendor 或文本猜测。
pub const fn lookup_model(vendor_id: u32, product_code: u32) -> Option<MeowMotorModel> {
    match (vendor_id, product_code) {
        (VENDOR_ID, PRODUCT_CODE_4310) => Some(MeowMotorModel::Model4310),
        (VENDOR_ID, PRODUCT_CODE_4342) => Some(MeowMotorModel::Model4342),
        _ => None,
    }
}

/// 从完整身份查找型号。revision、serial 和 0x1008 只保留作诊断，不参与分派。
pub fn model_for_identity(identity: &MotorIdentity) -> Option<MeowMotorModel> {
    lookup_model(identity.vendor_id, identity.product_code)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_products_are_recognized() {
        assert_eq!(
            lookup_model(VENDOR_ID, PRODUCT_CODE_4310),
            Some(MeowMotorModel::Model4310)
        );
        assert_eq!(
            lookup_model(VENDOR_ID, PRODUCT_CODE_4342),
            Some(MeowMotorModel::Model4342)
        );
    }

    #[test]
    fn neither_half_of_identity_is_enough() {
        assert_eq!(lookup_model(VENDOR_ID, 0xDEAD_BEEF), None);
        assert_eq!(lookup_model(0x6865_786D, PRODUCT_CODE_4310), None);
    }

    #[test]
    fn fixed_name_does_not_depend_on_device_text() {
        let identity = MotorIdentity {
            node_id: 1,
            vendor_id: VENDOR_ID,
            product_code: PRODUCT_CODE_4310,
            revision_number: 7,
            serial_number: 9,
            product_name: Some("untrusted device label".into()),
        };
        let model = model_for_identity(&identity).unwrap();
        assert_eq!(model.display_name(), "hexmeow 4310");
    }
}
