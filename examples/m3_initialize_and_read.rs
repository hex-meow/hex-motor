//! M3 真机自测：完整 `initialize()` + 看 TPDO 是否在流。
//!
//! 验证内容：
//! - 电机被 `Cia402Manager` 自动发现并 `Identified`
//! - `mgr.initialize(target)` 跑完整序列（NMT PreOp → 配 TPDO1+TPDO2 → 配 0x1016 → NMT Op）
//! - 之后能在 0x180+nid 上看到周期 TPDO1 帧（默认 1 ms，12 字节）和 0x280+nid
//!   上的 TPDO2 帧（默认 20 ms，10 字节）
//! - `list()` 中 target 节点 lifecycle == `Initialized` 且 `online == true`
//!
//! 用法：
//! ```bash
//! cargo run --example m3_initialize_and_read -- <iface> <our_hb_nid> <target_motor_nid>
//! # 例如：
//! cargo run --example m3_initialize_and_read -- can0 0x10 0x21
//! ```
//!
//! 第三个参数是要初始化的电机 nid（hex 或十进制）。
//!
//! 退出：Ctrl+C。退出时不会把电机 NMT 拉回 PreOp（v0.1 设计如此）。

use std::sync::Arc;
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFilter, CanId};
mod common;
use hex_motor::cia402::{
    default_tpdo1_recipe, default_tpdo2_recipe, Cia402Event, Cia402Manager, Cia402ManagerOptions,
    EventStreamItem, MotorLifecycle,
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
    let target: u8 = std::env::args()
        .nth(3)
        .map(|s| parse_u8(&s))
        .transpose()?
        .ok_or_else(|| anyhow::anyhow!("3rd arg required: target motor nid"))?;

    println!(
        "Opening {iface}\n  our HB nid=0x{our_nid:02X}\n  target motor nid=0x{target:02X}"
    );

    let bus: Arc<dyn CanBus> = common::open_bus(&iface).await?;
    let mgr = Cia402Manager::new(
        bus.clone(),
        Cia402ManagerOptions {
            heartbeat_node_id: our_nid,
            ..Default::default()
        },
    )?;

    // ===== 1. 等到 target 被 Identified =====
    println!("\n=== Waiting for nid 0x{target:02X} to be Identified ===");
    wait_for_identified(&mgr, target, Duration::from_secs(10)).await?;
    let id = mgr
        .list()
        .into_iter()
        .find(|m| m.node_id == target)
        .and_then(|m| m.identity)
        .ok_or_else(|| anyhow::anyhow!("identity missing for 0x{target:02X}"))?;
    println!(
        "Identified: vendor=0x{:08X} product=0x{:08X} friendly={:?}",
        id.vendor_id,
        id.product_code,
        hex_motor::cia402::known_devices::human_friendly_name(&id),
    );

    // ===== 2. 跑 initialize =====
    let r1 = default_tpdo1_recipe(target);
    let r2 = default_tpdo2_recipe(target);
    println!(
        "\n=== Initialize ===\n  TPDO1: cob=0x{:03X}  entries={}  {}B/frame  event={}ms\n  \
         TPDO2: cob=0x{:03X}  entries={}  {}B/frame  event={}ms",
        r1.cob_id,
        r1.entries.len(),
        r1.total_bytes(),
        r1.comm.event_timer_ms,
        r2.cob_id,
        r2.entries.len(),
        r2.total_bytes(),
        r2.comm.event_timer_ms,
    );
    let t0 = Instant::now();
    mgr.initialize(target).await?;
    println!("  -> Initialized in {:?}", t0.elapsed());

    // ===== 3. 订阅 TPDO1 + TPDO2 看原始帧 & 周期 list() =====
    println!("\n=== Watching TPDO1+TPDO2 (Ctrl+C to quit) ===");
    let tpdo1_cob = 0x180u16 + target as u16;
    let tpdo2_cob = 0x280u16 + target as u16;
    let mut rx1 = bus.subscribe(CanFilter::standard(tpdo1_cob, 0x7FF)).await?;
    let mut rx2 = bus.subscribe(CanFilter::standard(tpdo2_cob, 0x7FF)).await?;
    let mut events = mgr.subscribe_events();

    let mut last_list = Instant::now();
    let mut last_tpdo1_print = Instant::now();
    let mut last_tpdo2_print = Instant::now();
    let mut tpdo1_count = 0u64;
    let mut tpdo2_count = 0u64;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nCtrl+C, exiting.");
                break;
            }

            // TPDO1 —— 高速 (1ms)，限速 500ms 打印一行
            frame_res = rx1.recv() => match frame_res {
                Ok(frame) => {
                    tpdo1_count += 1;
                    if last_tpdo1_print.elapsed() >= Duration::from_millis(500) {
                        println!(
                            "  TPDO1 {}  len={}  data={:02X?}  (total: {tpdo1_count})",
                            fmt_id(frame.id()),
                            frame.data().len(),
                            frame.data(),
                        );
                        last_tpdo1_print = Instant::now();
                    }
                }
                Err(e) => println!("  TPDO1 rx error: {e}"),
            },

            // TPDO2 —— 低速 (20ms)，限速 1s 打印一行
            frame_res = rx2.recv() => match frame_res {
                Ok(frame) => {
                    tpdo2_count += 1;
                    if last_tpdo2_print.elapsed() >= Duration::from_secs(1) {
                        println!(
                            "  TPDO2 {}  len={}  data={:02X?}  (total: {tpdo2_count})",
                            fmt_id(frame.id()),
                            frame.data().len(),
                            frame.data(),
                        );
                        last_tpdo2_print = Instant::now();
                    }
                }
                Err(e) => println!("  TPDO2 rx error: {e}"),
            },

            // 事件 —— 全部打印
            ev = events.recv() => {
                match ev {
                    Some(EventStreamItem::Event(ev)) => println!("  [EVENT] {ev:?}"),
                    Some(EventStreamItem::Lagged { dropped }) =>
                        println!("  [EVENT] !! lagged, dropped {dropped}"),
                    None => break,
                }
            }
        }

        if last_list.elapsed() >= Duration::from_secs(2) {
            let m = mgr.list().into_iter().find(|m| m.node_id == target);
            println!();
            match m {
                Some(m) => println!(
                    "  list: nid=0x{:02X} lifecycle={:?} online={} nmt={:?}",
                    m.node_id, m.lifecycle, m.online, m.nmt_state
                ),
                None => println!("  list: target dropped from list?!"),
            }
            last_list = Instant::now();
        }
    }

    Ok(())
}

async fn wait_for_identified(mgr: &Cia402Manager, nid: u8, timeout: Duration) -> anyhow::Result<()> {
    let deadline = Instant::now() + timeout;
    let mut events = mgr.subscribe_events();

    // 先看一眼 list（可能已经 Identified 了）
    if mgr
        .list()
        .iter()
        .any(|m| m.node_id == nid && matches!(m.lifecycle, MotorLifecycle::Identified | MotorLifecycle::NeedsReinit { .. }))
    {
        return Ok(());
    }

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            anyhow::bail!("timeout waiting for nid 0x{nid:02X} to be Identified");
        }
        match tokio::time::timeout(remaining, events.recv()).await {
            Ok(Some(EventStreamItem::Event(Cia402Event::Identified { nid: n, .. }))) if n == nid => {
                return Ok(());
            }
            Ok(Some(_)) => continue,
            Ok(None) => anyhow::bail!("event stream closed"),
            Err(_) => anyhow::bail!("timeout waiting for nid 0x{nid:02X} to be Identified"),
        }
    }
}

fn fmt_id(id: CanId) -> String {
    match id {
        CanId::Standard(c) => format!("0x{c:03X}"),
        other => format!("{other:?}"),
    }
}

fn parse_u8(s: &str) -> anyhow::Result<u8> {
    let s = s.trim_start_matches("0x");
    u8::from_str_radix(s, 16)
        .or_else(|_| s.parse())
        .map_err(|e| anyhow::anyhow!("invalid u8 '{s}': {e}"))
}
