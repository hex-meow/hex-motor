//! M2 真机自测：[`Cia402Manager`] 启动 + 被动发现 + 事件流。
//!
//! 验证内容：
//! - 启动 manager 后自动开始广播心跳（电机端 0x1016 因此不会超时）
//! - 总线上的电机一上电就被发现 → `NodeAppeared` + 自动 SDO 读 0x1018 → `Identified`
//! - `list()` 周期打印当前所有看到的电机，名字带 `friendly_name`
//! - 断电/掉线时（持续 > 2.5 × heartbeat_period 没有 HB），收到 `NodeOffline`
//! - `Ctrl+C` 退出时 manager drop，后台 task 全部退出
//!
//! 用法：
//! ```bash
//! # 准备 vcan（没真机时）或用真 can0：
//! sudo modprobe vcan
//! sudo ip link add dev vcan0 type vcan
//! sudo ip link set up vcan0
//!
//! # 监听 can0，我方 HB 节点 ID 0x10，HB 周期 50 ms：
//! cargo run --example m2_discover -- can0 0x10 50
//!
//! # 在另一个 terminal 模拟一个电机心跳（nid 0x21 = COB 0x721, NMT Operational = 0x05）：
//! cansend can0 721#05
//! # 隔几秒后停发，看 NodeOffline 事件。
//! ```
//!
//! 注意：模拟节点对 SDO 读 0x1018 不会响应，会看到 `IdentifyFailed`。
//! 接真电机时应该能拿到 `Identified` 事件。

use std::sync::Arc;
use std::time::Duration;

use can_transport::CanBus;
mod common;
use hex_motor::cia402::{
    Cia402Manager, Cia402ManagerOptions, Cia402Event, EventStreamItem, MotorInfo,
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

    let bus: Arc<dyn CanBus> = common::open_bus(&iface).await?;

    let opts = Cia402ManagerOptions {
        heartbeat_node_id: our_nid,
        heartbeat_period: Duration::from_millis(period_ms),
        ..Default::default()
    };
    println!(
        "Opening {iface}\n  \
         our HB:               nid=0x{:02X}  period={:?}\n  \
         motor HB (pre-init):  expected period={:?}, offline after ~{:?}\n  \
         post-init (TPDO):     offline after {:?}\n\
         Press Ctrl+C to quit.",
        opts.heartbeat_node_id,
        opts.heartbeat_period,
        opts.motor_heartbeat_period,
        (opts.motor_heartbeat_period * 5) / 2,
        opts.initialized_stale_threshold,
    );

    let mgr = Cia402Manager::new(bus, opts)?;

    let mut events = mgr.subscribe_events();

    // 事件打印 task
    let events_task = tokio::spawn(async move {
        while let Some(item) = events.recv().await {
            match item {
                EventStreamItem::Event(ev) => print_event(&ev),
                EventStreamItem::Lagged { dropped } => {
                    println!("  [EVENT] !! lagged, dropped {dropped} events");
                }
            }
        }
    });

    // 周期打印列表
    let mut tick = tokio::time::interval(Duration::from_secs(2));
    let mut iter = 0u32;
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nCtrl+C, shutting down…");
                break;
            }
            _ = tick.tick() => {
                iter += 1;
                let list = mgr.list();
                print_list(iter, &list);
            }
        }
    }

    // mgr drop → cancel → events_tx closed → events_task exits
    drop(mgr);
    let _ = events_task.await;

    println!("done");
    Ok(())
}

fn print_event(ev: &Cia402Event) {
    match ev {
        Cia402Event::NodeAppeared { nid } => {
            println!("  [EVENT] NodeAppeared       nid=0x{nid:02X}");
        }
        Cia402Event::Identified { nid, identity } => {
            println!(
                "  [EVENT] Identified         nid=0x{nid:02X}  \
                 vendor=0x{:08X} product=0x{:08X} rev=0x{:08X} serial=0x{:08X} name={:?}",
                identity.vendor_id,
                identity.product_code,
                identity.revision_number,
                identity.serial_number,
                identity.product_name,
            );
            println!(
                "                              → friendly_name = {}",
                hex_motor::cia402::known_devices::human_friendly_name(identity)
            );
        }
        Cia402Event::IdentifyFailed { nid, reason } => {
            println!("  [EVENT] IdentifyFailed     nid=0x{nid:02X}  reason={reason}");
        }
        Cia402Event::NodeOnline { nid } => {
            println!("  [EVENT] NodeOnline         nid=0x{nid:02X}");
        }
        Cia402Event::NodeOffline { nid } => {
            println!("  [EVENT] NodeOffline        nid=0x{nid:02X}");
        }
        Cia402Event::Initializing { nid } => {
            println!("  [EVENT] Initializing       nid=0x{nid:02X}");
        }
        Cia402Event::Initialized { nid } => {
            println!("  [EVENT] Initialized        nid=0x{nid:02X}");
        }
        Cia402Event::NeedsReinit { nid, reason } => {
            println!("  [EVENT] NeedsReinit        nid=0x{nid:02X}  reason={reason:?}");
        }
        Cia402Event::EnteredError { nid, kind, raw } => {
            println!("  [EVENT] EnteredError       nid=0x{nid:02X}  kind={kind:?} raw=0x{raw:04X}");
        }
        // 非穷尽匹配兜底：M2 之后可能新增 variants。
        other => {
            println!("  [EVENT] (unhandled) {other:?}");
        }
    }
}

fn print_list(iter: u32, list: &[MotorInfo]) {
    println!();
    println!("=== list() @ iter {iter} ({} motor(s)) ===", list.len());
    if list.is_empty() {
        println!("  (empty - no motors / nodes seen yet)");
        return;
    }
    println!(
        "  {:<6} {:<24} {:<14} {:<7} nmt",
        "nid", "friendly_name", "lifecycle", "online"
    );
    for m in list {
        println!(
            "  0x{:02X}   {:<24} {:<14} {:<7} {:?}",
            m.node_id,
            m.friendly_name(),
            format!("{:?}", m.lifecycle),
            m.online,
            m.nmt_state,
        );
    }
}

fn parse_u8(s: &str) -> anyhow::Result<u8> {
    let s = s.trim_start_matches("0x");
    u8::from_str_radix(s, 16)
        .or_else(|_| s.parse())
        .map_err(|e| anyhow::anyhow!("invalid u8 '{s}': {e}"))
}
