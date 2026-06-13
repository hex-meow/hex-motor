//! [`StatusStream`]：每电机一个的 `LiveState` 流。详见 `DESIGN.md` §3。
//!
//! ## 数据通路
//!
//! 每路订阅独立持有一个有界 `tokio::sync::mpsc` channel。
//! [`crate::cia402::motor_entry::MotorEntry::publish`] 在 TPDO 帧、心跳帧、
//! 在线状态翻转等"会改变 `LiveState`"的时刻被调用，会：
//!
//! 1. 用 `ArcSwap` 原子地把新快照存进 [`MotorEntry::snapshot`]，
//!    [`crate::cia402::Cia402Manager::status`] 通过 `load_full` 无锁读取。
//! 2. 遍历当前订阅者列表 `try_send` 投递 [`StatusStreamItem::Sample`]；
//!    通道关闭的订阅者被 retain 掉。
//!
//! ## 溢出（`OverflowPolicy::Lagged`）
//!
//! channel 满时 sender 把"丢失计数" `pending_lagged` 累加到该订阅者本地；
//! 下一次发布时优先把 `Lagged { dropped }` 排进队列，成功后 `pending_lagged`
//! 清零；如果连 `Lagged` 都塞不进去，说明用户完全停了 `recv()`，继续
//! 累计。channel 一旦被对端关闭（订阅者 drop 了 `StatusStream`），下一次
//! `try_send` 会返回 `Closed`，我们就把该订阅者从列表里删除。

use tokio::sync::mpsc;

use super::types::LiveState;

/// 默认 channel 容量。`8192 = 1 ms × 8 s`，足够普通 UI 慢消费几秒不丢。
pub const DEFAULT_STREAM_CAPACITY: usize = 8192;

/// `subscribe_status` 的参数。
#[derive(Debug, Clone)]
pub struct StreamOptions {
    /// channel 容量（每订阅者独立）。必须 > 0。
    pub capacity: usize,
    /// 满了之后怎么办。v0.1 只支持 [`OverflowPolicy::Lagged`]。
    pub on_overflow: OverflowPolicy,
}

impl Default for StreamOptions {
    fn default() -> Self {
        Self {
            capacity: DEFAULT_STREAM_CAPACITY,
            on_overflow: OverflowPolicy::Lagged,
        }
    }
}

/// channel 满时的策略。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum OverflowPolicy {
    /// 丢弃新样本并在用户侧的下一个 `recv()` 处插入一条
    /// [`StatusStreamItem::Lagged`]，告知共丢了多少条。
    Lagged,
}

/// [`StatusStream::recv`] 的返回值。
#[derive(Debug, Clone)]
pub enum StatusStreamItem {
    /// 一个新的状态快照。
    Sample(LiveState),
    /// 自上次成功投递以来累计丢了 `dropped` 条样本。
    Lagged { dropped: u64 },
}

/// 单路状态订阅 —— 通过 [`crate::cia402::Cia402Manager::subscribe_status`]
/// 创建。`!Clone`：每路订阅者独立。
///
/// Manager / 对应电机被移除时，channel 关闭，[`StatusStream::recv`] 返回
/// `None`。
pub struct StatusStream {
    rx: mpsc::Receiver<StatusStreamItem>,
}

impl std::fmt::Debug for StatusStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StatusStream").finish_non_exhaustive()
    }
}

impl StatusStream {
    pub(crate) fn new(rx: mpsc::Receiver<StatusStreamItem>) -> Self {
        Self { rx }
    }

    /// 拿下一个事件。channel 关闭返回 `None`。
    pub async fn recv(&mut self) -> Option<StatusStreamItem> {
        self.rx.recv().await
    }

    /// 不阻塞地拿当前队列里的下一条；没有则返回 `None`。
    pub fn try_recv(&mut self) -> Option<StatusStreamItem> {
        self.rx.try_recv().ok()
    }
}

// =====================================================================
// Manager-side fan-out helpers
// =====================================================================

/// 单个订阅者在 Manager 侧的句柄。
pub(crate) struct Subscriber {
    tx: mpsc::Sender<StatusStreamItem>,
    /// 累计丢失（自上一条 `Lagged` 成功投递以来）。
    pending_lagged: u64,
    /// channel 创建时由调用方指定，仅作日志。
    capacity: usize,
}

impl std::fmt::Debug for Subscriber {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Subscriber")
            .field("capacity", &self.capacity)
            .field("pending_lagged", &self.pending_lagged)
            .field("closed", &self.tx.is_closed())
            .finish()
    }
}

impl Subscriber {
    /// 创建一个新的订阅者 + 配套 [`StatusStream`]。
    pub(crate) fn new(opts: &StreamOptions) -> (Self, StatusStream) {
        let capacity = opts.capacity.max(1);
        let (tx, rx) = mpsc::channel(capacity);
        (
            Self {
                tx,
                pending_lagged: 0,
                capacity,
            },
            StatusStream::new(rx),
        )
    }

    /// 投递一个 `Sample`。返回 `false` 表示对端已 drop，应从列表里删除。
    pub(crate) fn push(&mut self, state: &LiveState) -> bool {
        if self.tx.is_closed() {
            return false;
        }
        // 先把欠的 Lagged 还掉。
        if self.pending_lagged > 0 {
            match self.tx.try_send(StatusStreamItem::Lagged {
                dropped: self.pending_lagged,
            }) {
                Ok(()) => self.pending_lagged = 0,
                Err(mpsc::error::TrySendError::Full(_)) => {
                    // 用户完全停了 recv：这次 sample 也丢掉，继续累计。
                    self.pending_lagged = self.pending_lagged.saturating_add(1);
                    return true;
                }
                Err(mpsc::error::TrySendError::Closed(_)) => return false,
            }
        }
        match self.tx.try_send(StatusStreamItem::Sample(state.clone())) {
            Ok(()) => true,
            Err(mpsc::error::TrySendError::Full(_)) => {
                self.pending_lagged = self.pending_lagged.saturating_add(1);
                true
            }
            Err(mpsc::error::TrySendError::Closed(_)) => false,
        }
    }
}

// =====================================================================
// 单测
// =====================================================================

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;
    use crate::cia402::types::{Connection, Logic, Measurements};
    use crate::types::MotorMode;

    fn sample(seq: u8) -> LiveState {
        LiveState {
            connection: Connection {
                online: true,
                ..Default::default()
            },
            logic: Some(Logic::Enabled(MotorMode::ProfileVelocity)),
            measurements: Measurements {
                status_word: Some(0x0237),
                position_rev: Some(seq as f32 * 0.01),
                ..Default::default()
            },
            timestamp: Instant::now(),
        }
    }

    #[test]
    fn options_default_uses_lagged_policy() {
        let o = StreamOptions::default();
        assert_eq!(o.on_overflow, OverflowPolicy::Lagged);
        assert_eq!(o.capacity, DEFAULT_STREAM_CAPACITY);
    }

    #[tokio::test]
    async fn push_delivers_samples_in_order() {
        let opts = StreamOptions {
            capacity: 4,
            on_overflow: OverflowPolicy::Lagged,
        };
        let (mut sub, mut stream) = Subscriber::new(&opts);

        for i in 0..4 {
            assert!(sub.push(&sample(i)));
        }
        for i in 0..4 {
            let item = stream.recv().await.unwrap();
            match item {
                StatusStreamItem::Sample(s) => {
                    assert!((s.measurements.position_rev.unwrap() - i as f32 * 0.01).abs() < 1e-6);
                }
                StatusStreamItem::Lagged { .. } => panic!("unexpected lagged"),
            }
        }
    }

    #[tokio::test]
    async fn push_overflow_emits_lagged_with_dropped_count() {
        let opts = StreamOptions {
            capacity: 2,
            on_overflow: OverflowPolicy::Lagged,
        };
        let (mut sub, mut stream) = Subscriber::new(&opts);

        // 填满 channel：cap=2 能装 2 条 Sample。
        assert!(sub.push(&sample(0)));
        assert!(sub.push(&sample(1)));
        // 这两条会被丢，pending_lagged 累到 2。
        assert!(sub.push(&sample(2)));
        assert!(sub.push(&sample(3)));

        // 用户消费一条 → 腾出一格。
        let first = stream.recv().await.unwrap();
        assert!(matches!(first, StatusStreamItem::Sample(_)));

        // 再 push：先把 Lagged 还了，sample 进不进队都行（看是否还有空位）。
        // cap=2，刚消费 1 条，所以现在有 1 个空位；Lagged 占掉它；新 sample 又满 → +1。
        assert!(sub.push(&sample(4)));

        // 现在队列里依次：Sample(1), Lagged{dropped:2}
        let second = stream.recv().await.unwrap();
        assert!(matches!(second, StatusStreamItem::Sample(_)));
        let lagged = stream.recv().await.unwrap();
        match lagged {
            StatusStreamItem::Lagged { dropped } => assert_eq!(dropped, 2),
            _ => panic!("expected Lagged"),
        }
    }

    #[tokio::test]
    async fn push_returns_false_when_subscriber_dropped() {
        let opts = StreamOptions {
            capacity: 4,
            on_overflow: OverflowPolicy::Lagged,
        };
        let (mut sub, stream) = Subscriber::new(&opts);
        drop(stream);
        assert!(!sub.push(&sample(0)));
    }
}
