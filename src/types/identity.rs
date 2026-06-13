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
    /// HexMeow Vendor ID。来自 custom-protocol.md / custom-od.md。
    pub const HEXMEOW_VENDOR_ID: u32 = 0x0068_6578;
}
