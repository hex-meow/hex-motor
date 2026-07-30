//! 身份感知的设备通信设置事务输入与结果。

/// 一次设备设置事务的完整目标值。
///
/// `data_bitrate` / `transmit_pdo_brs` 为 `None` 只适用于对象字典明确报告为
/// Classic CAN only 的设备；CAN FD 设备必须同时给出这两个字段。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceSettingsUpdate {
    /// 设备重启后使用的 CANopen Node-ID。
    pub new_node_id: u8,
    /// 仲裁段波特率，当前规范只允许 1 Mbit/s。
    pub nominal_bitrate: u32,
    /// 数据段波特率；CAN FD 设备只允许 1/2/4/5 Mbit/s。
    pub data_bitrate: Option<u32>,
    /// 设备发送 TPDO 时是否使用 BRS。
    pub transmit_pdo_brs: Option<bool>,
}

/// 成功完成一次设备设置事务后的效果说明。
///
/// 本类型不表示设备已经重启；驱动不会主动发送 NMT 或复位命令。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceSettingsResult {
    /// 至少一个对象值与目标值不同并已写入。
    pub changed: bool,
    /// 本事务完成后需要设备重新上电或重启，包含持久化重试后的保守要求。
    pub restart_required: bool,
    /// 设备使用 write-through 持久化，成功返回时底层持久化仍可能在排队。
    pub persistence_pending: bool,
    /// 本事务实际修改了可立即生效的 TPDO BRS 设置。
    pub brs_applied_immediately: bool,
}
