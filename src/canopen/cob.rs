//! COB-ID 计算 + custom-protocol.md 中保留 ID 的检查。
//!
//! 本模块只做纯函数运算；不发任何帧。

use crate::error::{Error, Result};

/// CANopen 通信对象 (function code 分类)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommunicationObject {
    /// NMT 命令帧 (0x000，主站发出)。
    Nmt,
    /// 同步帧 (0x080)。
    Sync,
    /// 紧急帧 (0x080 + node_id)。
    Emergency,
    /// 时间帧 (0x100)。
    Time,
    Tpdo1,
    Rpdo1,
    Tpdo2,
    Rpdo2,
    Tpdo3,
    Rpdo3,
    Tpdo4,
    Rpdo4,
    /// 服务器 → 客户端 (0x580 + nid)。
    Tsdo,
    /// 客户端 → 服务器 (0x600 + nid)。
    Rsdo,
    /// 心跳 (0x700 + nid)。
    Heartbeat,
}

impl CommunicationObject {
    /// 该 object 的 function code 部分（COB-ID 高 4 bits）。
    pub fn function_code(self) -> u16 {
        match self {
            CommunicationObject::Nmt => 0x000,
            CommunicationObject::Sync => 0x080,
            CommunicationObject::Emergency => 0x080,
            CommunicationObject::Time => 0x100,
            CommunicationObject::Tpdo1 => 0x180,
            CommunicationObject::Rpdo1 => 0x200,
            CommunicationObject::Tpdo2 => 0x280,
            CommunicationObject::Rpdo2 => 0x300,
            CommunicationObject::Tpdo3 => 0x380,
            CommunicationObject::Rpdo3 => 0x400,
            CommunicationObject::Tpdo4 => 0x480,
            CommunicationObject::Rpdo4 => 0x500,
            CommunicationObject::Tsdo => 0x580,
            CommunicationObject::Rsdo => 0x600,
            CommunicationObject::Heartbeat => 0x700,
        }
    }

    /// 拼出标准 COB-ID。
    ///
    /// `Nmt` / `Sync` / `Time` 不带 node id，调用方应传 0；其他必须 1..=127。
    pub fn cob_id(self, node_id: u8) -> Result<u16> {
        let base = self.function_code();
        match self {
            CommunicationObject::Nmt
            | CommunicationObject::Sync
            | CommunicationObject::Time => Ok(base),
            _ => {
                if node_id == 0 || node_id > 0x7F {
                    return Err(Error::InvalidNodeId(node_id));
                }
                Ok(base | node_id as u16)
            }
        }
    }
}

/// 从 COB-ID 反解 `(CommunicationObject, node_id)`。
/// 对 Sync / Nmt / Time 返回 `node_id = 0`。
pub fn parse_cob_id(cob_id: u16) -> Option<(CommunicationObject, u8)> {
    if cob_id == 0x000 {
        return Some((CommunicationObject::Nmt, 0));
    }
    if cob_id == 0x080 {
        return Some((CommunicationObject::Sync, 0));
    }
    if cob_id == 0x100 {
        return Some((CommunicationObject::Time, 0));
    }

    let function = cob_id & 0x780;
    let nid = (cob_id & 0x7F) as u8;
    if nid == 0 || nid > 0x7F {
        return None;
    }

    let obj = match function {
        0x080 => CommunicationObject::Emergency,
        0x180 => CommunicationObject::Tpdo1,
        0x200 => CommunicationObject::Rpdo1,
        0x280 => CommunicationObject::Tpdo2,
        0x300 => CommunicationObject::Rpdo2,
        0x380 => CommunicationObject::Tpdo3,
        0x400 => CommunicationObject::Rpdo3,
        0x480 => CommunicationObject::Tpdo4,
        0x500 => CommunicationObject::Rpdo4,
        0x580 => CommunicationObject::Tsdo,
        0x600 => CommunicationObject::Rsdo,
        0x700 => CommunicationObject::Heartbeat,
        _ => return None,
    };
    Some((obj, nid))
}

/// custom-protocol.md "保留的 CAN 帧范围 - 标准帧" 检查。
/// 用户自定义帧应避开这些 COB-ID。
pub fn is_reserved_standard_cob_id(cob_id: u16) -> bool {
    matches!(
        cob_id,
        // 单点
        0x000        // NMT
        | 0x080      // SYNC
        | 0x100      // TIME
    ) || (0x081..=0x0FF).contains(&cob_id)   // EMCY
        || (0x181..=0x1FF).contains(&cob_id) // TPDO1
        || (0x201..=0x27F).contains(&cob_id) // RPDO1
        || (0x281..=0x2FF).contains(&cob_id) // TPDO2
        || (0x301..=0x37F).contains(&cob_id) // RPDO2
        || (0x381..=0x3FF).contains(&cob_id) // TPDO3
        || (0x401..=0x47F).contains(&cob_id) // RPDO3
        || (0x481..=0x4FF).contains(&cob_id) // TPDO4
        || (0x501..=0x57F).contains(&cob_id) // RPDO4
        || (0x581..=0x5FF).contains(&cob_id) // TSDO
        || (0x601..=0x67F).contains(&cob_id) // RSDO
        || (0x701..=0x77F).contains(&cob_id) // HB
        || (0x680..=0x6FF).contains(&cob_id) // LSS-ish
        || (0x780..=0x7FF).contains(&cob_id) // LSS
}

/// custom-protocol.md "保留的 CAN 帧范围 - 扩展帧" 检查。
pub fn is_reserved_extended_cob_id(cob_id: u32) -> bool {
    matches!(cob_id, 0x09 | 0xA9 | 0xAA) || (0xFE00..=0xFEFF).contains(&cob_id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cob_id_tpdo1_node10() {
        assert_eq!(CommunicationObject::Tpdo1.cob_id(0x10).unwrap(), 0x190);
    }

    #[test]
    fn cob_id_heartbeat_node1() {
        assert_eq!(CommunicationObject::Heartbeat.cob_id(0x01).unwrap(), 0x701);
    }

    #[test]
    fn cob_id_rsdo_node127() {
        assert_eq!(CommunicationObject::Rsdo.cob_id(0x7F).unwrap(), 0x67F);
    }

    #[test]
    fn cob_id_nmt_ignores_node() {
        assert_eq!(CommunicationObject::Nmt.cob_id(0).unwrap(), 0x000);
    }

    #[test]
    fn cob_id_rejects_zero_node() {
        assert!(matches!(
            CommunicationObject::Tpdo1.cob_id(0),
            Err(Error::InvalidNodeId(0))
        ));
    }

    #[test]
    fn cob_id_rejects_node_over_127() {
        assert!(matches!(
            CommunicationObject::Tpdo1.cob_id(128),
            Err(Error::InvalidNodeId(128))
        ));
    }

    #[test]
    fn parse_heartbeat() {
        assert_eq!(parse_cob_id(0x710), Some((CommunicationObject::Heartbeat, 0x10)));
    }

    #[test]
    fn parse_tpdo1() {
        assert_eq!(parse_cob_id(0x190), Some((CommunicationObject::Tpdo1, 0x10)));
    }

    #[test]
    fn parse_sync() {
        assert_eq!(parse_cob_id(0x080), Some((CommunicationObject::Sync, 0)));
    }

    #[test]
    fn parse_emergency() {
        // 0x081 是 EMCY for node 1
        assert_eq!(parse_cob_id(0x081), Some((CommunicationObject::Emergency, 1)));
    }

    #[test]
    fn parse_invalid() {
        assert_eq!(parse_cob_id(0x150), None); // 0x100..=0x17F 无效区间
    }

    #[test]
    fn reserved_standard_covers_all_canopen_objects() {
        // 心跳
        assert!(is_reserved_standard_cob_id(0x701));
        assert!(is_reserved_standard_cob_id(0x77F));
        // TPDO1 区间
        assert!(is_reserved_standard_cob_id(0x181));
        assert!(is_reserved_standard_cob_id(0x1FF));
        // SDO 双向
        assert!(is_reserved_standard_cob_id(0x581));
        assert!(is_reserved_standard_cob_id(0x67F));
        // SYNC / TIME / NMT
        assert!(is_reserved_standard_cob_id(0x000));
        assert!(is_reserved_standard_cob_id(0x080));
        assert!(is_reserved_standard_cob_id(0x100));
        // LSS 区
        assert!(is_reserved_standard_cob_id(0x7E5));
    }

    #[test]
    fn reserved_standard_allows_safe_ids() {
        // 自定义安全区域示例。这些 ID 不在 custom-protocol.md 的保留列表里：
        // 因为列表是 "181-1FF" 等（每段去掉了 base 那个 +0 的 ID）。
        assert!(!is_reserved_standard_cob_id(0x101));
        assert!(!is_reserved_standard_cob_id(0x17F));
        assert!(!is_reserved_standard_cob_id(0x180));
        assert!(!is_reserved_standard_cob_id(0x200));
        assert!(!is_reserved_standard_cob_id(0x280));
        assert!(!is_reserved_standard_cob_id(0x300));
        assert!(!is_reserved_standard_cob_id(0x380));
        assert!(!is_reserved_standard_cob_id(0x400));
        assert!(!is_reserved_standard_cob_id(0x480));
        assert!(!is_reserved_standard_cob_id(0x500));
        assert!(!is_reserved_standard_cob_id(0x580));
        assert!(!is_reserved_standard_cob_id(0x600));
        assert!(!is_reserved_standard_cob_id(0x700));
    }

    #[test]
    fn reserved_standard_includes_lss_range() {
        // 0x680-0x6FF 撞 LSS，应当被认为保留。
        assert!(is_reserved_standard_cob_id(0x680));
        assert!(is_reserved_standard_cob_id(0x6FF));
        assert!(is_reserved_standard_cob_id(0x780));
        assert!(is_reserved_standard_cob_id(0x7FF));
    }

    #[test]
    fn reserved_extended_known() {
        assert!(is_reserved_extended_cob_id(0x09));
        assert!(is_reserved_extended_cob_id(0xA9));
        assert!(is_reserved_extended_cob_id(0xAA));
        assert!(is_reserved_extended_cob_id(0xFE00));
        assert!(is_reserved_extended_cob_id(0xFEFF));
        assert!(!is_reserved_extended_cob_id(0xAB));
        assert!(!is_reserved_extended_cob_id(0xFDFF));
    }
}
