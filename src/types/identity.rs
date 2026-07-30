//! 通过 CANopen 0x1018 / 0x1008 拿到的电机身份。

/// 电机身份（来自 CANopen 0x1018 + 可选 0x1008）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotorIdentity {
    pub node_id: u8,
    pub vendor_id: u32,
    pub product_code: u32,
    pub revision_number: u32,
    pub serial_number: u32,
    pub product_name: Option<String>,
}

impl MotorIdentity {
    /// HexFellow Vendor ID。
    pub const HEXFELLOW_VENDOR_ID: u32 = 0x0068_6578;
    /// HexMeow Vendor ID。
    pub const HEXMEOW_VENDOR_ID: u32 = 0x6865_786D;
}

#[cfg(test)]
mod tests {
    use super::MotorIdentity;

    #[test]
    fn vendor_ids_are_distinct_and_correctly_named() {
        assert_eq!(MotorIdentity::HEXFELLOW_VENDOR_ID, 0x0068_6578);
        assert_eq!(MotorIdentity::HEXMEOW_VENDOR_ID, 0x6865_786D);
        assert_ne!(
            MotorIdentity::HEXFELLOW_VENDOR_ID,
            MotorIdentity::HEXMEOW_VENDOR_ID
        );
    }
}
