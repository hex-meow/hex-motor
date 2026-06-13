//! M4 真机自测：在 ProfileVelocity 模式下匀速转一台 CiA402 电机。
//!
//! 流程（和 `m3_initialize_and_read` 是叠加关系）：
//! 1. 等到 target 被 `Cia402Manager` 自动 identify。
//! 2. `mgr.initialize(target)` 把 TPDO1+TPDO2 配好，进 NMT Operational。
//! 3. `mgr.set_mode(target, ProfileVelocity)` 走 CiA402 ramp 并等 TPDO2 反馈
//!    `Logic::Enabled(ProfileVelocity)`。
//! 4. `mgr.set_target(target, Velocity { rev_per_s })` 写一次 `0x60FF`。
//! 5. 周期性打印 `list()` 里 target 节点的 logic / measurements；Ctrl+C 退出。
//!    退出前会尝试 `mgr.disable(target)` 让电机短刹车（best-effort）。
//!
//! ## 用法
//!
//! ```bash
//! cargo run --example m4_run_profile_velocity -- <iface> <our_hb_nid> <target_nid> [rev_per_s]
//! # 例如：
//! cargo run --example m4_run_profile_velocity -- can0 0x10 0x21 0.5
//! ```
//!
//! 默认 `rev_per_s = 0.3`。**先把电机架空 / 卸负载** 再跑。
//!
//! ## 退出前
//!
//! - Ctrl+C → 调一次 `disable()`，**不**主动 NMT Stop。
//! - 不调用 disable 也没关系：我方 HB 一断（drop 时 cancel），电机端 0x1016
//!   消费者 250 ms 后自己触发安全保护。

use std::sync::Arc;
use std::time::{Duration, Instant};

use can_transport::CanBus;
mod common;
use hex_motor::cia402::{
    Cia402Event, Cia402Manager, Cia402ManagerOptions, EventStreamItem, MotorLifecycle,
};
use hex_motor::types::{MotorMode, MotorTarget};

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
    let rev_per_s: f32 = std::env::args()
        .nth(4)
        .map(|s| s.parse::<f32>())
        .transpose()?
        .unwrap_or(0.3);

    println!(
        "Opening {iface}\n  our HB nid = 0x{our_nid:02X}\n  target motor nid = 0x{target:02X}\n  \
         target velocity = {rev_per_s:.3} Rev/s"
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
    {
        let m = mgr.list().into_iter().find(|m| m.node_id == target).unwrap();
        println!("Identified: {} ({:?})", m.friendly_name(), m.identity);
    }

    // ===== 2. Initialize =====
    println!("\n=== Initialize ===");
    let t0 = Instant::now();
    mgr.initialize(target).await?;
    println!("  -> Initialized in {:?}", t0.elapsed());

    // ===== 3. set_mode(ProfileVelocity) + 等 TPDO 反馈 =====
    println!("\n=== set_mode(ProfileVelocity) ===");
    let t0 = Instant::now();
    mgr.set_mode(target, MotorMode::ProfileVelocity).await?;
    println!("  -> Enabled(ProfileVelocity) confirmed in {:?}", t0.elapsed());

    // ===== 4. set_target(Velocity) =====
    println!("\n=== set_target(Velocity {{ rev_per_s = {rev_per_s:.3} }}) ===");
    mgr.set_target(target, MotorTarget::Velocity { rev_per_s }).await?;
    println!("  -> 0x60FF written");

    // ===== 5. 周期打印 + 监听事件 =====
    println!("\n=== Running (Ctrl+C to disable + exit) ===");
    let mut events = mgr.subscribe_events();
    let mut last_print = Instant::now();

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nCtrl+C, disabling motor...");
                if let Err(e) = mgr.disable(target).await {
                    println!("  disable() failed: {e}");
                } else {
                    println!("  disable() ok");
                }
                break;
            }

            ev = events.recv() => {
                match ev {
                    Some(EventStreamItem::Event(Cia402Event::EnteredError { nid, kind, raw }))
                        if nid == target => {
                        println!(
                            "  !! [EVENT] EnteredError nid=0x{nid:02X} kind={kind:?} raw=0x{raw:04X}"
                        );
                    }
                    Some(EventStreamItem::Event(ev)) => println!("  [EVENT] {ev:?}"),
                    Some(EventStreamItem::Lagged { dropped }) =>
                        println!("  [EVENT] !! lagged, dropped {dropped}"),
                    None => break,
                }
            }
        }

        if last_print.elapsed() >= Duration::from_secs(1) {
            print_status(&mgr, target);
            last_print = Instant::now();
        }
    }

    Ok(())
}

fn print_status(mgr: &Cia402Manager, nid: u8) {
    let Some(m) = mgr.list().into_iter().find(|m| m.node_id == nid) else {
        println!("  status: target not in list?!");
        return;
    };
    println!(
        "  status nid=0x{:02X}  lifecycle={:?}  online={}  logic={:?}  nmt={:?}",
        m.node_id, m.lifecycle, m.online, m.logic, m.nmt_state,
    );
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

fn parse_u8(s: &str) -> anyhow::Result<u8> {
    let s = s.trim_start_matches("0x");
    u8::from_str_radix(s, 16)
        .or_else(|_| s.parse())
        .map_err(|e| anyhow::anyhow!("invalid u8 '{s}': {e}"))
}
