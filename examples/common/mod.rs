//! Shared backend selector for the hex-motor examples.
//!
//! The `<iface>` argument picks the backend:
//! - `gs_usb` / `gs_usb0` / `gs_usb1` — a gs_usb / candleLight adapter over
//!   USB (CAN-FD, 1 Mbit / 5 Mbit). The trailing number selects the channel
//!   on a multi-channel adapter (`can0` = 0).
//! - anything else — a SocketCAN interface of that name (e.g. `can0`, `vcan0`).
//!
//! On Linux, `gs_usb` requires the kernel `gs_usb` driver to be detached
//! (the backend does this itself) and usbfs access (run with sudo or add a
//! udev rule).

use std::sync::Arc;

use can_transport::CanBus;

pub async fn open_bus(iface: &str) -> anyhow::Result<Arc<dyn CanBus>> {
    if let Some(channel) = gs_usb_channel(iface) {
        use can_transport::gs_usb::{GsUsbBus, GsUsbConfig};
        log::info!("opening gs_usb backend channel {channel} (CAN-FD 1M/5M)");
        let bus = GsUsbBus::open(GsUsbConfig::fd_1m_5m().with_channel(channel)).await?;
        log::info!("gs_usb opened: {:?}", bus.capabilities());
        Ok(Arc::new(bus))
    } else {
        use can_transport::socketcan::SocketCanBus;
        log::info!("opening SocketCAN interface {iface}");
        Ok(Arc::new(SocketCanBus::open(iface)?))
    }
}

/// Parse a gs_usb interface spec into a channel number, or `None` if `iface`
/// is not a gs_usb spec. Accepts `gs_usb`, `gs_usb0`, `gs_usb1`, `gs_usb:1`,
/// and the underscore-less `gsusb2` variants.
fn gs_usb_channel(iface: &str) -> Option<u16> {
    let s = iface.trim().to_ascii_lowercase();
    let rest = s.strip_prefix("gs_usb").or_else(|| s.strip_prefix("gsusb"))?;
    let rest = rest.strip_prefix(':').unwrap_or(rest);
    if rest.is_empty() {
        Some(0)
    } else {
        rest.parse().ok()
    }
}
