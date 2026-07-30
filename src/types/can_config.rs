//! 设备自身报告的 CAN 链路配置快照。
//!
//! 本层只负责读取并保存设备值，不判断它是否与主机配置或应用策略匹配。

/// 从设备对象字典读取到的 CAN 配置。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceCanConfig {
    /// 对象字典里保存的 Node-ID。它可能等待重启，因此不一定等于当前活动 COB-ID。
    pub stored_node_id: u8,
    /// 仲裁段波特率，单位 bit/s。
    pub nominal_bitrate: u32,
    /// 数据段波特率，单位 bit/s。
    ///
    /// `None` 只表示已识别设备明确报告自己为 Classic CAN only。
    pub data_bitrate: Option<u32>,
    /// 设备发送 PDO 时是否使用 bit-rate switching。
    ///
    /// Classic-only 设备为 `None`。上层不应要求不同设备的这个值相同。
    pub transmit_pdo_brs: Option<bool>,
}

/// 设备 CAN 配置读取状态。
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum DeviceCanConfigStatus {
    /// 身份或配置读取尚未完成。
    #[default]
    Pending,
    /// 已按受支持的精确 `(vendor_id, product_code)` 方言完成读取。
    Available(DeviceCanConfig),
    /// 该精确 `(vendor_id, product_code)` 没有已知对象字典方言。
    Unsupported,
    /// 方言已知，但对象读取或内容校验失败。
    ReadFailed { reason: String },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_defaults_to_pending() {
        assert_eq!(
            DeviceCanConfigStatus::default(),
            DeviceCanConfigStatus::Pending
        );
    }
}
