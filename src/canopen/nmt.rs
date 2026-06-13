//! NMT 命令帧构造 + 心跳帧解码（含 NMT state 字节）。

use can_transport::{CanFrame, CanId, CanIoError, FrameKind};

/// 电机当前 NMT 状态（出现在入向心跳帧的 data[0]）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum NmtState {
    /// 0x00：刚上电的 Boot-Up 帧。
    BootUp = 0x00,
    /// 0x04：Stopped。
    Stopped = 0x04,
    /// 0x05：Operational，正常工作态。
    Operational = 0x05,
    /// 0x7F：Pre-Operational。
    PreOperational = 0x7F,
}

impl NmtState {
    pub fn try_from_byte(b: u8) -> Option<Self> {
        Some(match b {
            0x00 => Self::BootUp,
            0x04 => Self::Stopped,
            0x05 => Self::Operational,
            0x7F => Self::PreOperational,
            _ => return None,
        })
    }
}

/// NMT 命令字节 (主站 → 节点)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NmtCommand {
    /// 0x01：进入 Operational。
    StartRemoteNode = 0x01,
    /// 0x02：进入 Stopped。
    StopRemoteNode = 0x02,
    /// 0x80：进入 Pre-Operational。
    EnterPreOperational = 0x80,
    /// 0x81：复位节点。
    ResetNode = 0x81,
    /// 0x82：复位通讯。
    ResetCommunication = 0x82,
}

/// 构造一帧 NMT 命令帧。
///
/// `target_node_id == 0` 表示广播给所有节点。
pub fn build_nmt_command(cmd: NmtCommand, target_node_id: u8) -> Result<CanFrame, CanIoError> {
    let data = [cmd as u8, target_node_id];
    CanFrame::new_data(CanId::Standard(0x000), &data)
}

/// 尝试解析心跳帧，返回 `(node_id, NMT 状态)`。
///
/// 不是心跳帧或长度不对，返回 `None`。
pub fn parse_heartbeat(frame: &CanFrame) -> Option<(u8, NmtState)> {
    if !matches!(frame.kind(), FrameKind::Data) {
        return None;
    }
    let CanId::Standard(cob_id) = frame.id() else {
        return None;
    };
    // function code 0x700, node id 1..=127
    if cob_id & 0x780 != 0x700 {
        return None;
    }
    let nid = (cob_id & 0x7F) as u8;
    if nid == 0 {
        return None;
    }
    let data = frame.data();
    if data.len() != 1 {
        return None;
    }
    NmtState::try_from_byte(data[0]).map(|s| (nid, s))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nmt_cmd_start_node() {
        let f = build_nmt_command(NmtCommand::StartRemoteNode, 0x10).unwrap();
        assert_eq!(f.id(), CanId::Standard(0x000));
        assert_eq!(f.data(), &[0x01, 0x10]);
    }

    #[test]
    fn nmt_cmd_broadcast_preop() {
        let f = build_nmt_command(NmtCommand::EnterPreOperational, 0).unwrap();
        assert_eq!(f.data(), &[0x80, 0x00]);
    }

    #[test]
    fn parse_hb_operational() {
        let f = CanFrame::new_data(CanId::Standard(0x710), &[0x05]).unwrap();
        assert_eq!(parse_heartbeat(&f), Some((0x10, NmtState::Operational)));
    }

    #[test]
    fn parse_hb_bootup() {
        let f = CanFrame::new_data(CanId::Standard(0x711), &[0x00]).unwrap();
        assert_eq!(parse_heartbeat(&f), Some((0x11, NmtState::BootUp)));
    }

    #[test]
    fn parse_hb_preop() {
        let f = CanFrame::new_data(CanId::Standard(0x77F), &[0x7F]).unwrap();
        assert_eq!(parse_heartbeat(&f), Some((0x7F, NmtState::PreOperational)));
    }

    #[test]
    fn parse_hb_wrong_size() {
        let f = CanFrame::new_data(CanId::Standard(0x710), &[0x05, 0x00]).unwrap();
        assert_eq!(parse_heartbeat(&f), None);
    }

    #[test]
    fn parse_hb_wrong_cob() {
        let f = CanFrame::new_data(CanId::Standard(0x180), &[0x05]).unwrap();
        assert_eq!(parse_heartbeat(&f), None);
    }

    #[test]
    fn parse_hb_unknown_state_byte() {
        let f = CanFrame::new_data(CanId::Standard(0x710), &[0x42]).unwrap();
        assert_eq!(parse_heartbeat(&f), None);
    }

    #[test]
    fn parse_hb_node_zero() {
        let f = CanFrame::new_data(CanId::Standard(0x700), &[0x05]).unwrap();
        assert_eq!(parse_heartbeat(&f), None);
    }
}
