//! CiA402 专属数据模型：节点生命周期、控制逻辑状态、连接与测量快照。
//!
//! 跨协议复用的通用类型见 [`crate::types`]；本模块只放 CiA402 形态电机
//! 才有意义的概念。

use std::time::Instant;

use crate::canopen::nmt::NmtState;
use crate::types::{MotorErrorKind, MotorIdentity, MotorMode};

/// 节点在 Manager 视角下的生命周期（详见 `DESIGN.md` §1）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MotorLifecycle {
    /// 收到过 HB，但 0x1018 还没拉到（或者拉失败）。
    Unknown,
    /// `identity` 已知，但 TPDO / 0x1016 还没配置。
    Identified,
    /// `initialize()` 正在跑（M3+）。
    Initializing,
    /// TPDO 在流、0x1016 已设、NMT Operational。**可控制**。
    Initialized,
    /// 曾经 Initialized，但电机离开了 Operational 等异常情况，需要再次 `initialize()`。
    NeedsReinit { reason: ReinitReason },
}

impl MotorLifecycle {
    /// 仅当 `Initialized` 时返回 true。
    pub fn is_ready(&self) -> bool {
        matches!(self, MotorLifecycle::Initialized)
    }
}

/// `NeedsReinit` 的原因。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReinitReason {
    /// 心跳显示电机的 NMT 状态离开了 Operational。
    LeftOperational,
    // 预留：CommunicationLost / UserRequested / TpdoStreamLost ...
}

/// 控制逻辑状态。M2 中此字段始终为 `None`，从 M4 开始由 Runner 维护。
#[derive(Debug, Clone, PartialEq)]
pub enum Logic {
    /// 控制字处于 "Switch On Disabled" 或类似禁用状态。
    Disabled,
    /// "Operation Enabled" 且 mode_display 为指定模式。
    Enabled(MotorMode),
    /// 状态字含 Fault。
    Error { kind: MotorErrorKind, raw_code: u16 },
}

/// 给用户看的电机条目：身份 + 生命周期 + 在线 + 控制逻辑。
///
/// 由 [`crate::cia402::manager::Cia402Manager::list`] 返回。结构是快照式的
/// `Clone`，与 Manager 内部解耦。
#[derive(Debug, Clone)]
pub struct MotorInfo {
    pub node_id: u8,
    /// `Unknown` 阶段为 `None`。
    pub identity: Option<MotorIdentity>,
    pub lifecycle: MotorLifecycle,
    /// 最近 ~2.5 个 `heartbeat_period` 内有过 HB 或 TPDO。
    pub online: bool,
    /// 控制逻辑（M4+ 填充；M2/M3 始终 `None`）。
    pub logic: Option<Logic>,
    /// 来自最近一次入向心跳的 NMT 状态。
    pub nmt_state: Option<NmtState>,
    /// 峰值力矩（Nm），`initialize()` 从 `0x6076` 读出来缓存。供上位机把
    /// `0x6072` 的千分比输入换算成 Nm 显示。读失败为 `None`。
    pub peak_torque_nm: Option<f32>,
}

impl MotorInfo {
    /// 仅当 lifecycle == Initialized && online 时返回 true。
    pub fn is_ready(&self) -> bool {
        self.lifecycle.is_ready() && self.online
    }

    /// 人类可读名称。详见 [`crate::cia402::known_devices::human_friendly_name`]；
    /// identity 还没拉到时退化为 `"Node 0xNN"`。
    pub fn friendly_name(&self) -> String {
        match &self.identity {
            Some(id) => crate::cia402::known_devices::human_friendly_name(id),
            None => format!("Node 0x{:02X}", self.node_id),
        }
    }
}

/// 由 TPDO 解码出的测量值（M4+ 由 codec 填）。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Measurements {
    pub position_rev: Option<f32>,
    /// 上位机用电机时间戳 [`Measurements::timestamp_us`] 对位置做解卷绕 +
    /// 滑动窗口最小二乘斜率算出来的滤波速度（rev/s）。样本不足时 `None`。
    pub velocity_rev_per_s: Option<f32>,
    /// 由 TPDO1 的 `0x6077`（i16 ‰ of peak）× 缓存的 `peak_torque_nm` 得到。
    /// `initialize()` 没读到 `0x6076` 时保持 `None`。
    pub torque_nm: Option<f32>,
    pub driver_temp_c: Option<f32>,
    pub motor_temp_c: Option<f32>,
    /// 原始留一份方便排查。
    pub status_word: Option<u16>,
    pub mode_display: Option<u8>,
    pub error_register: Option<u8>,
    /// 电机 `0x1013` 高分辨率时间戳，单位 μs（u32，~71min 回绕）。
    /// 速度滤波与 CSV 录制都用它。
    pub timestamp_us: Option<u32>,
}

/// 连接相关信息。
#[derive(Debug, Clone, Default)]
pub struct Connection {
    pub last_heartbeat: Option<Instant>,
    pub last_tpdo: Option<Instant>,
    pub online: bool,
    pub nmt_state: Option<NmtState>,
}

/// [`crate::cia402::Cia402Manager::status`] / [`crate::cia402::Cia402Manager::subscribe_status`]
/// 共享的快照类型。由 [`crate::cia402::motor_entry::MotorEntry::publish`] 在
/// 每帧 TPDO / HB / offline 翻转时原子刷新。
#[derive(Debug, Clone)]
pub struct LiveState {
    pub connection: Connection,
    pub logic: Option<Logic>,
    pub measurements: Measurements,
    pub timestamp: Instant,
}

impl LiveState {
    pub fn empty(now: Instant) -> Self {
        Self {
            connection: Connection::default(),
            logic: None,
            measurements: Measurements::default(),
            timestamp: now,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lifecycle_is_ready_only_initialized() {
        assert!(MotorLifecycle::Initialized.is_ready());
        assert!(!MotorLifecycle::Unknown.is_ready());
        assert!(!MotorLifecycle::Identified.is_ready());
        assert!(!MotorLifecycle::Initializing.is_ready());
        assert!(!MotorLifecycle::NeedsReinit {
            reason: ReinitReason::LeftOperational
        }
        .is_ready());
    }

    fn motor_info(
        nid: u8,
        identity: Option<MotorIdentity>,
        lifecycle: MotorLifecycle,
        online: bool,
    ) -> MotorInfo {
        MotorInfo {
            node_id: nid,
            identity,
            lifecycle,
            online,
            logic: None,
            nmt_state: None,
            peak_torque_nm: None,
        }
    }

    #[test]
    fn motor_info_is_ready_requires_both() {
        let id = Some(MotorIdentity {
            node_id: 0x10,
            vendor_id: 0,
            product_code: 0,
            revision_number: 0,
            serial_number: 0,
            product_name: None,
        });
        assert!(motor_info(0x10, id.clone(), MotorLifecycle::Initialized, true).is_ready());
        assert!(!motor_info(0x10, id.clone(), MotorLifecycle::Initialized, false).is_ready());
        assert!(!motor_info(0x10, id.clone(), MotorLifecycle::Identified, true).is_ready());
        assert!(!motor_info(0x10, None, MotorLifecycle::Unknown, true).is_ready());
    }

    #[test]
    fn friendly_name_falls_back_when_no_identity() {
        let m = motor_info(0x42, None, MotorLifecycle::Unknown, true);
        assert_eq!(m.friendly_name(), "Node 0x42");
    }

    #[test]
    fn friendly_name_uses_known_devices_table() {
        let id = MotorIdentity {
            node_id: 0x10,
            vendor_id: 0x0068_6578,
            product_code: 0xAAAA_0002,
            revision_number: 0,
            serial_number: 0,
            product_name: None,
        };
        let m = motor_info(0x10, Some(id), MotorLifecycle::Identified, true);
        assert_eq!(m.friendly_name(), "HexMeow Motor");
    }
}
