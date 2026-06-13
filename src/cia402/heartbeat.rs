//! 出向心跳广播 task。
//!
//! 周期发 NMT Operational 心跳，电机端 0x1016 监听这个 producer node id，
//! 超时触发电机自身保护（详见 `DESIGN.md` §2）。

use std::sync::Arc;
use std::time::Duration;

use can_transport::CanBus;
use tokio::time::MissedTickBehavior;
use tokio_util::sync::CancellationToken;

use crate::canopen::heartbeat::build_heartbeat_frame;
use crate::canopen::nmt::NmtState;

/// 常驻 task：每 `period` 广播一次自己的心跳，直到 `cancel`。
pub(crate) async fn run_hb_broadcast(
    bus: Arc<dyn CanBus>,
    hb_node_id: u8,
    period: Duration,
    cancel: CancellationToken,
) {
    let mut tick = tokio::time::interval(period);
    tick.set_missed_tick_behavior(MissedTickBehavior::Delay);
    // 第一次 tick 立即返回，不延迟 `period` 才发第一帧。
    tick.tick().await;
    loop {
        let frame = match build_heartbeat_frame(hb_node_id, NmtState::Operational) {
            Ok(f) => f,
            Err(e) => {
                log::error!("HB build failed (hb_node_id=0x{hb_node_id:02X}): {e}");
                // 不再重试，退出 task；真要复活需重建 Manager。
                return;
            }
        };
        if let Err(e) = bus.send(frame).await {
            log::warn!("HB broadcast send failed: {e}");
        }
        tokio::select! {
            _ = cancel.cancelled() => {
                log::debug!("HB broadcaster cancelled");
                return;
            }
            _ = tick.tick() => {}
        }
    }
}
