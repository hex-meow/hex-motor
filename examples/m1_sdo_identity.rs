//! M1 真机自测：通过 SDO 读取一台已知节点的 0x1018 身份信息 + 0x1008 产品名。
//!
//! 验证内容：
//! - [`hex_motor::canopen::sdo`] 的类型化便利函数走通真实 SDO 事务
//! - 错误透传到 `Result<_, hex_motor::Error>` 正常
//! - 字符串字段（0x1008）segmented 上传正常
//!
//! 用法：
//! ```bash
//! # 真机：节点 0x01
//! cargo run --example m1_sdo_identity -- can0 0x01
//! ```

use std::time::Duration;

mod common;
use hex_motor::canopen::sdo;
use hex_motor::cia402::known_devices::human_friendly_name;
use hex_motor::types::MotorIdentity;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let iface = std::env::args().nth(1).unwrap_or_else(|| "vcan0".to_string());
    let nid: u8 = std::env::args()
        .nth(2)
        .map(|s| parse_u8(&s))
        .transpose()?
        .unwrap_or(0x01);

    println!("Opening {iface}, reading identity of node 0x{nid:02X}");
    let bus = common::open_bus(&iface).await?;
    let timeout = Some(Duration::from_millis(200));

    // 0x1018 强制支持
    let vendor_id = sdo::upload_u32(&bus, nid, 0x1018, 1, timeout).await?;
    println!("  0x1018:01 Vendor ID       = 0x{vendor_id:08X}");

    let product_code = sdo::upload_u32(&bus, nid, 0x1018, 2, timeout).await?;
    println!("  0x1018:02 Product Code    = 0x{product_code:08X}");

    let revision = sdo::upload_u32(&bus, nid, 0x1018, 3, timeout).await?;
    println!("  0x1018:03 Revision Number = 0x{revision:08X}");

    let serial = sdo::upload_u32(&bus, nid, 0x1018, 4, timeout).await?;
    println!("  0x1018:04 Serial Number   = 0x{serial:08X}");

    // 0x1008 是 optional 的，识别失败不影响后续
    let product_name = match sdo::upload_string(&bus, nid, 0x1008, 0, timeout).await {
        Ok(name) => {
            println!("  0x1008    Product Name    = {name:?}");
            Some(name)
        }
        Err(e) => {
            println!("  0x1008    Product Name    = <unavailable: {e}>");
            None
        }
    };

    let identity = MotorIdentity {
        node_id: nid,
        vendor_id,
        product_code,
        revision_number: revision,
        serial_number: serial,
        product_name,
    };

    println!();
    println!("  -> 人类可读名称：{}", human_friendly_name(&identity));

    Ok(())
}

fn parse_u8(s: &str) -> anyhow::Result<u8> {
    let s = s.trim_start_matches("0x");
    u8::from_str_radix(s, 16)
        .or_else(|_| s.parse())
        .map_err(|e| anyhow::anyhow!("invalid u8 '{s}': {e}"))
}
