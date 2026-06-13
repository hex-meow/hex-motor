//! 心跳：我方出向广播 + 电机端 0x1016 consumer 条目编码。
//!
//! 两类心跳：
//! - **出向**（我们 → 电机）：周期发布，电机端 0x1016 监听，超时触发电机自身保护。
//! - **入向**（电机 → 我们）：解析 NMT 状态字节 + 用作发现/在线判断（见 `nmt::parse_heartbeat`）。

use can_transport::{CanFrame, CanIoError};

use super::nmt::NmtState;

/// 构造一帧出向心跳。
///
/// `hb_node_id` 是"我们"在 CAN 上的 NMT 节点 ID，电机端的 0x1016 会被配置成
/// 监听这个 ID。`state` 通常为 [`NmtState::Operational`]。
pub fn build_heartbeat_frame(hb_node_id: u8, state: NmtState) -> Result<CanFrame, CanIoError> {
    let cob_id = 0x700u16 | (hb_node_id as u16 & 0x7F);
    CanFrame::new_data(cob_id, &[state as u8])
}

/// 编码 CANopen 0x1016 sub 1..n 的 "Consumer heartbeat time" 条目。
///
/// 格式：
/// - bit 24-31: 0 (保留)
/// - bit 16-23: 被监听的 producer node id (1..=127)；为 0 或 > 127 表示"该 sub 未使用"
/// - bit  0-15: 超时时间 ms (为 0 表示"该 sub 未使用")
pub fn encode_consumer_heartbeat_entry(producer_node_id: u8, timeout_ms: u16) -> u32 {
    ((producer_node_id as u32 & 0xFF) << 16) | (timeout_ms as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_transport::CanId;

    #[test]
    fn hb_frame_format() {
        let f = build_heartbeat_frame(0x10, NmtState::Operational).unwrap();
        assert_eq!(f.id(), CanId::Standard(0x710));
        assert_eq!(f.data(), &[0x05]);
    }

    #[test]
    fn hb_frame_preop() {
        let f = build_heartbeat_frame(0x10, NmtState::PreOperational).unwrap();
        assert_eq!(f.data(), &[0x7F]);
    }

    #[test]
    fn consumer_entry_typical() {
        // 监听节点 0x10，超时 250 ms = 0xFA
        let e = encode_consumer_heartbeat_entry(0x10, 250);
        assert_eq!(e, 0x0010_00FA);
    }

    #[test]
    fn consumer_entry_disable() {
        // producer = 0 + timeout = 0 → 整条不使用
        assert_eq!(encode_consumer_heartbeat_entry(0, 0), 0);
    }
}
