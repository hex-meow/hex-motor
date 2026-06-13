//! [`Cia402Event`] + [`EventStream`]：Manager 异步事件广播。
//!
//! Manager 内部用 [`tokio::sync::broadcast`] 做 1-to-many fan-out。
//! 订阅者可以慢，但慢到掉队会收到 [`EventStreamItem::Lagged`]，并不会阻塞别的订阅者。

use tokio::sync::broadcast;

use crate::types::{MotorErrorKind, MotorIdentity};

use super::types::ReinitReason;

/// 默认事件 channel 容量。Manager Options 可覆盖。
pub const DEFAULT_EVENTS_CAPACITY: usize = 256;

/// Manager 广播给订阅者的事件。
///
/// M2 中实际发送的子集：
/// - [`Cia402Event::NodeAppeared`]
/// - [`Cia402Event::Identified`]
/// - [`Cia402Event::IdentifyFailed`]
/// - [`Cia402Event::NodeOnline`] / [`Cia402Event::NodeOffline`]
///
/// 其余 variants 是 M3-M5 的占位，提前定义以稳定 enum 形状。
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum Cia402Event {
    /// 第一次看到某 nid 的入向心跳（之前不在表里）。
    NodeAppeared { nid: u8 },

    /// 成功读到了 0x1018 (+ 可选 0x1008)。
    Identified { nid: u8, identity: MotorIdentity },

    /// 试图 identify 一个节点但 SDO 失败 / 超时。节点保留在 `Unknown` 状态，
    /// 下次再有 HB 触发时会再尝试。
    IdentifyFailed { nid: u8, reason: String },

    /// 节点恢复 online（曾经 offline 或刚出现）。
    NodeOnline { nid: u8 },

    /// 节点持续 ~2.5 个 `heartbeat_period` 没收到 HB / TPDO。
    NodeOffline { nid: u8 },

    // ===== M3 及以后会真正发送的事件 =====
    /// `initialize()` 已开始。
    Initializing { nid: u8 },

    /// `initialize()` 已完成，电机 NMT Operational + TPDO 已配。
    Initialized { nid: u8 },

    /// 曾经 `Initialized` 的电机出现需要重新 init 的情况。
    NeedsReinit { nid: u8, reason: ReinitReason },

    /// 电机的 TPDO 报告进入了 Error 状态。
    EnteredError {
        nid: u8,
        kind: MotorErrorKind,
        raw: u16,
    },
}

/// `EventStream::recv()` 返回值。
#[derive(Debug)]
pub enum EventStreamItem {
    /// 一个正常事件。
    Event(Cia402Event),
    /// channel 容量溢出，丢了 `dropped` 条事件。继续 recv 即可拿后续事件。
    Lagged { dropped: u64 },
}

/// Manager 事件流。`!Clone` —— 每路订阅独立。
///
/// 用 [`crate::cia402::manager::Cia402Manager::subscribe_events`] 创建。
pub struct EventStream {
    rx: broadcast::Receiver<Cia402Event>,
}

impl std::fmt::Debug for EventStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EventStream").finish_non_exhaustive()
    }
}

impl EventStream {
    pub(crate) fn new(rx: broadcast::Receiver<Cia402Event>) -> Self {
        Self { rx }
    }

    /// 等下一个事件 / Lagged 通知 / Manager 已 drop。
    pub async fn recv(&mut self) -> Option<EventStreamItem> {
        match self.rx.recv().await {
            Ok(ev) => Some(EventStreamItem::Event(ev)),
            Err(broadcast::error::RecvError::Lagged(n)) => {
                Some(EventStreamItem::Lagged { dropped: n })
            }
            Err(broadcast::error::RecvError::Closed) => None,
        }
    }
}
