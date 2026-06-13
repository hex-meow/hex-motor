//! M1 真机自测：心跳收发。
//!
//! 验证内容：
//! - [`can_transport::socketcan::SocketCanBus`] 能正常打开物理 / vcan 接口
//! - [`hex_motor::canopen::heartbeat::build_heartbeat_frame`] 产生的帧能被
//!   总线接受并发出
//! - [`hex_motor::canopen::nmt::parse_heartbeat`] 能正确解析收到的心跳
//! - 总线 fan-out 订阅工作正常
//!
//! 用法：
//! ```bash
//! # 准备一个 vcan（如果没有真硬件）：
//! sudo modprobe vcan
//! sudo ip link add dev vcan0 type vcan
//! sudo ip link set up vcan0
//!
//! # 我方 HB 节点 ID 0x10，周期 50ms，监听 5 秒：
//! cargo run --example m1_heartbeat -- vcan0 0x10 50 5
//! ```
//!
//! 如果你有一台真电机连在 can0 上，把 vcan0 换成 can0；电机的心跳会被打印出来。

use std::sync::Arc;
use std::time::Duration;

mod common;
use can_transport::{CanBus, CanFilter};
use hex_motor::canopen::{
    heartbeat::build_heartbeat_frame,
    nmt::{parse_heartbeat, NmtState},
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let iface = std::env::args().nth(1).unwrap_or_else(|| "vcan0".to_string());
    let our_nid: u8 = std::env::args()
        .nth(2)
        .map(|s| parse_u8(&s))
        .transpose()?
        .unwrap_or(0x10);
    let period_ms: u64 = std::env::args()
        .nth(3)
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(50);
    let listen_secs: u64 = std::env::args()
        .nth(4)
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(5);

    println!(
        "Opening {iface}; broadcasting HB from 0x{our_nid:02X} every {period_ms}ms; \
         listening for {listen_secs}s"
    );

    let bus: Arc<dyn CanBus> = common::open_bus(&iface).await?;

    // 监听任务：订阅 0x700-0x77F 区间（function code 0x700, mask 0x780）。
    let listener = {
        let bus = bus.clone();
        tokio::spawn(async move {
            let mut rx = bus
                .subscribe(CanFilter::standard(0x700, 0x780))
                .await
                .expect("subscribe");
            loop {
                match rx.recv().await {
                    Ok(frame) => match parse_heartbeat(&frame) {
                        Some((nid, state)) => {
                            println!("  <- HB nid=0x{nid:02X} state={state:?}");
                        }
                        None => {
                            log::debug!("非心跳帧 / 长度异常: {frame:?}");
                        }
                    },
                    Err(e) => {
                        log::warn!("接收错误: {e}");
                    }
                }
            }
        })
    };

    // 心跳广播任务：周期发我们的心跳。
    let broadcaster = {
        let bus = bus.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(period_ms));
            loop {
                tick.tick().await;
                let frame = build_heartbeat_frame(our_nid, NmtState::Operational)
                    .expect("build hb");
                if let Err(e) = bus.send(frame).await {
                    log::warn!("发送心跳失败: {e}");
                }
            }
        })
    };

    tokio::time::sleep(Duration::from_secs(listen_secs)).await;
    listener.abort();
    broadcaster.abort();

    println!("done");
    Ok(())
}

fn parse_u8(s: &str) -> anyhow::Result<u8> {
    let s = s.trim_start_matches("0x");
    u8::from_str_radix(s, 16)
        .or_else(|_| s.parse())
        .map_err(|e| anyhow::anyhow!("invalid u8 '{s}': {e}"))
}
