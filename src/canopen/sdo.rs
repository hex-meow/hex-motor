//! 薄封装 [`canopen_sdo::asynch`]：统一错误类型 + 数值类型化便利函数。
//!
//! SDO 失败**不在本层重试**；上层（manager / runner）决定要不要重试。

use std::time::Duration;

use can_transport::CanBus;
use canopen_sdo::asynch;

use crate::error::{Error, Result};

/// 读 SDO 原始字节。
pub async fn upload(
    bus: &(impl CanBus + ?Sized),
    node_id: u8,
    index: u16,
    subindex: u8,
    timeout: Option<Duration>,
) -> Result<Vec<u8>> {
    asynch::upload_bytes(bus, node_id, index, subindex, timeout)
        .await
        .map_err(Error::from)
}

/// 写 SDO 原始字节。
pub async fn download(
    bus: &(impl CanBus + ?Sized),
    node_id: u8,
    index: u16,
    subindex: u8,
    data: &[u8],
    timeout: Option<Duration>,
) -> Result<()> {
    asynch::download_bytes(bus, node_id, index, subindex, data, timeout)
        .await
        .map_err(Error::from)
}

// =================== 类型化便利函数 ===================

pub async fn upload_u32(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    timeout: Option<Duration>,
) -> Result<u32> {
    let v = upload(bus, nid, index, sub, timeout).await?;
    if v.len() < 4 {
        return Err(Error::Internal(format!(
            "expected 4 bytes for 0x{index:04X}:{sub:02X}, got {}",
            v.len()
        )));
    }
    Ok(u32::from_le_bytes([v[0], v[1], v[2], v[3]]))
}

pub async fn upload_u16(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    timeout: Option<Duration>,
) -> Result<u16> {
    let v = upload(bus, nid, index, sub, timeout).await?;
    if v.len() < 2 {
        return Err(Error::Internal(format!(
            "expected 2 bytes for 0x{index:04X}:{sub:02X}, got {}",
            v.len()
        )));
    }
    Ok(u16::from_le_bytes([v[0], v[1]]))
}

pub async fn upload_u8(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    timeout: Option<Duration>,
) -> Result<u8> {
    let v = upload(bus, nid, index, sub, timeout).await?;
    if v.is_empty() {
        return Err(Error::Internal(format!(
            "expected 1 byte for 0x{index:04X}:{sub:02X}, got 0"
        )));
    }
    Ok(v[0])
}

pub async fn upload_f32(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    timeout: Option<Duration>,
) -> Result<f32> {
    let v = upload(bus, nid, index, sub, timeout).await?;
    if v.len() < 4 {
        return Err(Error::Internal(format!(
            "expected 4 bytes for 0x{index:04X}:{sub:02X}, got {}",
            v.len()
        )));
    }
    Ok(f32::from_le_bytes([v[0], v[1], v[2], v[3]]))
}

pub async fn upload_string(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    timeout: Option<Duration>,
) -> Result<String> {
    let v = upload(bus, nid, index, sub, timeout).await?;
    // CANopen 字符串通常是 visible string；按 UTF-8 lossy 解码，去掉末尾 NUL。
    let end = v.iter().position(|&b| b == 0).unwrap_or(v.len());
    Ok(String::from_utf8_lossy(&v[..end]).into_owned())
}

pub async fn download_u32(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    val: u32,
    timeout: Option<Duration>,
) -> Result<()> {
    download(bus, nid, index, sub, &val.to_le_bytes(), timeout).await
}

pub async fn download_u16(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    val: u16,
    timeout: Option<Duration>,
) -> Result<()> {
    download(bus, nid, index, sub, &val.to_le_bytes(), timeout).await
}

pub async fn download_u8(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    val: u8,
    timeout: Option<Duration>,
) -> Result<()> {
    download(bus, nid, index, sub, &[val], timeout).await
}

pub async fn download_i16(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    val: i16,
    timeout: Option<Duration>,
) -> Result<()> {
    download(bus, nid, index, sub, &val.to_le_bytes(), timeout).await
}

pub async fn download_i32(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    val: i32,
    timeout: Option<Duration>,
) -> Result<()> {
    download(bus, nid, index, sub, &val.to_le_bytes(), timeout).await
}

pub async fn download_f32(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    index: u16,
    sub: u8,
    val: f32,
    timeout: Option<Duration>,
) -> Result<()> {
    download(bus, nid, index, sub, &val.to_le_bytes(), timeout).await
}
