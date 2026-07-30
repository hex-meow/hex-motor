//! 按精确设备身份读取 CAN 配置对象。
//!
//! 这里的分派键始终是完整的 `(vendor_id, product_code)`。即使两个产品共用
//! vendor ID，也不能据此推断它们使用相同对象字典。

use std::time::Duration;

use can_transport::CanBus;

use crate::canopen::sdo;
use crate::types::{DeviceCanConfig, DeviceCanConfigStatus, MotorIdentity};

use super::motor_entry::MotorEntry;
use super::types::MotorLifecycle;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CanConfigDialect {
    Object2001,
    Object2100,
}

const OBJECT_2001_IDENTITIES: &[(u32, u32)] = &[
    (0x4859_444C, 0xAAAA_0001),
    (0x4859_444C, 0xAAAA_0002),
    (0x4859_444C, 0xAAAA_0005),
];

const OBJECT_2100_IDENTITIES: &[(u32, u32)] = &[
    (0x6865_786D, 0x0069_6D75),
    (0x6865_786D, 0x6169_6D75),
    (0x6865_786D, 0x006C_6674),
];

fn dialect_for(identity: &MotorIdentity) -> Option<CanConfigDialect> {
    let key = (identity.vendor_id, identity.product_code);
    if OBJECT_2001_IDENTITIES.contains(&key) {
        Some(CanConfigDialect::Object2001)
    } else if OBJECT_2100_IDENTITIES.contains(&key) {
        Some(CanConfigDialect::Object2100)
    } else {
        None
    }
}

/// 身份成功后立即缓存，并把设备配置标成正在读取。
pub(crate) fn begin_device_can_config_read(entry: &MotorEntry, identity: &MotorIdentity) {
    let mut inner = entry.inner.lock().unwrap();
    inner.identity = Some(identity.clone());
    inner.can_config = DeviceCanConfigStatus::Pending;
    if matches!(inner.lifecycle, MotorLifecycle::Unknown) {
        inner.lifecycle = MotorLifecycle::Identified;
    }
}

/// 缓存最终配置状态。即使读取失败，也不修改已成功读取的 identity/lifecycle。
pub(crate) fn finish_device_can_config_read(entry: &MotorEntry, status: DeviceCanConfigStatus) {
    entry.inner.lock().unwrap().can_config = status;
}

pub(crate) async fn read_device_can_config(
    bus: &dyn CanBus,
    identity: &MotorIdentity,
    sdo_timeout: Duration,
) -> DeviceCanConfigStatus {
    let Some(dialect) = dialect_for(identity) else {
        return DeviceCanConfigStatus::Unsupported;
    };

    let timeout = Some(sdo_timeout);
    let result = match dialect {
        CanConfigDialect::Object2001 => read_object_2001(bus, identity.node_id, timeout).await,
        CanConfigDialect::Object2100 => read_object_2100(bus, identity.node_id, timeout).await,
    };

    match result {
        Ok(config) => DeviceCanConfigStatus::Available(config),
        Err(reason) => DeviceCanConfigStatus::ReadFailed { reason },
    }
}

async fn read_object_2001(
    bus: &dyn CanBus,
    nid: u8,
    timeout: Option<Duration>,
) -> std::result::Result<DeviceCanConfig, String> {
    let max_subindex = sdo::upload_u8(bus, nid, 0x2001, 0, timeout)
        .await
        .map_err(|e| format!("read 0x2001:00 failed: {e}"))?;
    if max_subindex != 4 {
        return Err(format!(
            "0x2001:00 expected max sub-index 4, got {max_subindex}"
        ));
    }

    let nominal_bitrate = sdo::upload_u32(bus, nid, 0x2001, 2, timeout)
        .await
        .map_err(|e| format!("read 0x2001:02 failed: {e}"))?;
    let data_bitrate = sdo::upload_u32(bus, nid, 0x2001, 3, timeout)
        .await
        .map_err(|e| format!("read 0x2001:03 failed: {e}"))?;
    let transmit_pdo_brs = sdo::upload_u8(bus, nid, 0x2001, 4, timeout)
        .await
        .map_err(|e| format!("read 0x2001:04 failed: {e}"))?;

    Ok(DeviceCanConfig {
        nominal_bitrate,
        data_bitrate: Some(data_bitrate),
        transmit_pdo_brs: Some(parse_bool(0x2001, 4, transmit_pdo_brs)?),
    })
}

async fn read_object_2100(
    bus: &dyn CanBus,
    nid: u8,
    timeout: Option<Duration>,
) -> std::result::Result<DeviceCanConfig, String> {
    let max_subindex = sdo::upload_u8(bus, nid, 0x2100, 0, timeout)
        .await
        .map_err(|e| format!("read 0x2100:00 failed: {e}"))?;

    match max_subindex {
        1 => {
            let nominal_bitrate = sdo::upload_u32(bus, nid, 0x2100, 1, timeout)
                .await
                .map_err(|e| format!("read 0x2100:01 failed: {e}"))?;
            Ok(DeviceCanConfig {
                nominal_bitrate,
                data_bitrate: None,
                transmit_pdo_brs: None,
            })
        }
        3 => {
            let nominal_bitrate = sdo::upload_u32(bus, nid, 0x2100, 1, timeout)
                .await
                .map_err(|e| format!("read 0x2100:01 failed: {e}"))?;
            let data_bitrate = sdo::upload_u32(bus, nid, 0x2100, 2, timeout)
                .await
                .map_err(|e| format!("read 0x2100:02 failed: {e}"))?;
            let transmit_pdo_brs = sdo::upload_u8(bus, nid, 0x2100, 3, timeout)
                .await
                .map_err(|e| format!("read 0x2100:03 failed: {e}"))?;
            Ok(DeviceCanConfig {
                nominal_bitrate,
                data_bitrate: Some(data_bitrate),
                transmit_pdo_brs: Some(parse_bool(0x2100, 3, transmit_pdo_brs)?),
            })
        }
        other => Err(format!(
            "0x2100:00 expected max sub-index 1 or 3, got {other}"
        )),
    }
}

fn parse_bool(index: u16, subindex: u8, raw: u8) -> std::result::Result<bool, String> {
    match raw {
        0 => Ok(false),
        1 => Ok(true),
        other => Err(format!(
            "0x{index:04X}:{subindex:02X} expected 0 or 1, got {other}"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashMap, HashSet};
    use std::sync::Mutex;

    use async_trait::async_trait;
    use can_transport::{CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanRx};
    use tokio::sync::mpsc;

    fn identity(vendor_id: u32, product_code: u32) -> MotorIdentity {
        MotorIdentity {
            node_id: 1,
            vendor_id,
            product_code,
            revision_number: 0,
            serial_number: 0,
            product_name: None,
        }
    }

    #[test]
    fn object_2001_dialect_requires_an_exact_tuple() {
        for product_code in [0xAAAA_0001, 0xAAAA_0002, 0xAAAA_0005] {
            assert_eq!(
                dialect_for(&identity(0x4859_444C, product_code)),
                Some(CanConfigDialect::Object2001)
            );
        }
        assert_eq!(dialect_for(&identity(0x4859_444C, 0xAAAA_9999)), None);
        assert_eq!(dialect_for(&identity(0x0068_6578, 0xAAAA_0001)), None);
    }

    #[test]
    fn object_2100_dialect_requires_an_exact_tuple() {
        for product_code in [0x0069_6D75, 0x6169_6D75, 0x006C_6674] {
            assert_eq!(
                dialect_for(&identity(0x6865_786D, product_code)),
                Some(CanConfigDialect::Object2100)
            );
        }
        assert_eq!(dialect_for(&identity(0x6865_786D, 0xDEAD_BEEF)), None);
        assert_eq!(dialect_for(&identity(0x0068_6578, 0x006C_6674)), None);
    }

    #[test]
    fn schema_tuples_are_unique_disjoint_and_registered_as_known_devices() {
        let object_2001: HashSet<_> = OBJECT_2001_IDENTITIES.iter().copied().collect();
        let object_2100: HashSet<_> = OBJECT_2100_IDENTITIES.iter().copied().collect();

        assert_eq!(
            object_2001.len(),
            OBJECT_2001_IDENTITIES.len(),
            "0x2001 identity table contains a duplicate tuple"
        );
        assert_eq!(
            object_2100.len(),
            OBJECT_2100_IDENTITIES.len(),
            "0x2100 identity table contains a duplicate tuple"
        );
        assert!(
            object_2001.is_disjoint(&object_2100),
            "a tuple cannot select both 0x2001 and 0x2100"
        );

        for &(vendor_id, product_code) in
            OBJECT_2001_IDENTITIES.iter().chain(OBJECT_2100_IDENTITIES)
        {
            let known = crate::cia402::known_devices::lookup_known_device(vendor_id, product_code);
            assert!(
                known.is_some(),
                "schema tuple 0x{vendor_id:08X}/0x{product_code:08X} lacks an exact KNOWN_DEVICES entry"
            );
        }
    }

    #[test]
    fn bool_fields_only_accept_zero_or_one() {
        assert_eq!(parse_bool(0x2100, 3, 0), Ok(false));
        assert_eq!(parse_bool(0x2100, 3, 1), Ok(true));
        assert!(parse_bool(0x2100, 3, 2).is_err());
    }

    #[tokio::test]
    async fn reads_object_2001_profile() {
        let bus = ScriptedSdoBus::new([
            ((0x2001, 0), vec![4]),
            ((0x2001, 2), 1_000_000u32.to_le_bytes().to_vec()),
            ((0x2001, 3), 5_000_000u32.to_le_bytes().to_vec()),
            ((0x2001, 4), vec![1]),
        ]);
        let status = read_device_can_config(
            &bus,
            &identity(0x4859_444C, 0xAAAA_0001),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(
            status,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(5_000_000),
                transmit_pdo_brs: Some(true),
            })
        );
        assert_eq!(
            bus.requests(),
            vec![(0x2001, 0), (0x2001, 2), (0x2001, 3), (0x2001, 4)]
        );
    }

    #[tokio::test]
    async fn object_2100_max_subindex_one_is_explicitly_classic_only() {
        let bus = ScriptedSdoBus::new([
            ((0x2100, 0), vec![1]),
            ((0x2100, 1), 1_000_000u32.to_le_bytes().to_vec()),
        ]);
        let status = read_device_can_config(
            &bus,
            &identity(0x6865_786D, 0x006C_6674),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(
            status,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                nominal_bitrate: 1_000_000,
                data_bitrate: None,
                transmit_pdo_brs: None,
            })
        );
        assert_eq!(bus.requests(), vec![(0x2100, 0), (0x2100, 1)]);
    }

    #[tokio::test]
    async fn reads_object_2100_fd_profile() {
        let bus = ScriptedSdoBus::new([
            ((0x2100, 0), vec![3]),
            ((0x2100, 1), 1_000_000u32.to_le_bytes().to_vec()),
            ((0x2100, 2), 4_000_000u32.to_le_bytes().to_vec()),
            ((0x2100, 3), vec![0]),
        ]);
        let status = read_device_can_config(
            &bus,
            &identity(0x6865_786D, 0x6169_6D75),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(
            status,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(4_000_000),
                transmit_pdo_brs: Some(false),
            })
        );
        assert_eq!(
            bus.requests(),
            vec![(0x2100, 0), (0x2100, 1), (0x2100, 2), (0x2100, 3)]
        );
    }

    #[tokio::test]
    async fn unknown_2100_shape_is_read_failed_not_classic() {
        let bus = ScriptedSdoBus::new([((0x2100, 0), vec![2])]);
        let status = read_device_can_config(
            &bus,
            &identity(0x6865_786D, 0x0069_6D75),
            Duration::from_millis(20),
        )
        .await;

        assert!(matches!(
            status,
            DeviceCanConfigStatus::ReadFailed { reason }
                if reason.contains("expected max sub-index 1 or 3")
        ));
        assert_eq!(bus.requests(), vec![(0x2100, 0)]);
    }

    #[tokio::test]
    async fn unknown_tuple_is_unsupported_without_any_sdo_probe() {
        let bus = ScriptedSdoBus::new([]);
        let status = read_device_can_config(
            &bus,
            &identity(0x0068_6578, 0xAAAA_0001),
            Duration::from_millis(20),
        )
        .await;

        assert_eq!(status, DeviceCanConfigStatus::Unsupported);
        assert!(bus.requests().is_empty());
    }

    #[test]
    fn read_failed_status_keeps_identity_and_identified_lifecycle() {
        let entry = MotorEntry::new(1);
        let identity = identity(0x6865_786D, 0x006C_6674);
        begin_device_can_config_read(&entry, &identity);
        finish_device_can_config_read(
            &entry,
            DeviceCanConfigStatus::ReadFailed {
                reason: "simulated SDO failure".to_string(),
            },
        );

        let inner = entry.inner.lock().unwrap();
        assert_eq!(inner.identity.as_ref(), Some(&identity));
        assert_eq!(inner.lifecycle, MotorLifecycle::Identified);
        assert!(matches!(
            inner.can_config,
            DeviceCanConfigStatus::ReadFailed { ref reason }
                if reason == "simulated SDO failure"
        ));
    }

    struct ScriptedSdoBus {
        values: HashMap<(u16, u8), Vec<u8>>,
        requests: Mutex<Vec<(u16, u8)>>,
        response_tx: Mutex<Option<mpsc::UnboundedSender<CanFrame>>>,
    }

    impl ScriptedSdoBus {
        fn new<const N: usize>(values: [((u16, u8), Vec<u8>); N]) -> Self {
            Self {
                values: values.into_iter().collect(),
                requests: Mutex::new(Vec::new()),
                response_tx: Mutex::new(None),
            }
        }

        fn requests(&self) -> Vec<(u16, u8)> {
            self.requests.lock().unwrap().clone()
        }
    }

    #[async_trait]
    impl CanBus for ScriptedSdoBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            let request = frame.data();
            if request.len() != 8 || request[0] != 0x40 {
                return Err(CanIoError::backend(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "test bus expected an expedited SDO upload request",
                )));
            }
            let nid = match frame.id() {
                CanId::Standard(id) if (0x601..=0x67F).contains(&id) => (id - 0x600) as u8,
                _ => {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "test bus received an invalid RSDO COB-ID",
                    )))
                }
            };
            let index = u16::from_le_bytes([request[1], request[2]]);
            let subindex = request[3];
            self.requests.lock().unwrap().push((index, subindex));

            let mut response = [0u8; 8];
            response[1] = request[1];
            response[2] = request[2];
            response[3] = subindex;
            match self.values.get(&(index, subindex)) {
                Some(value) if (1..=4).contains(&value.len()) => {
                    response[0] = 0x43 | (((4 - value.len()) as u8) << 2);
                    response[4..4 + value.len()].copy_from_slice(value);
                }
                Some(value) => {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        format!("test value has unsupported length {}", value.len()),
                    )))
                }
                None => {
                    response[0] = 0x80;
                    response[4..8].copy_from_slice(&0x0602_0000u32.to_le_bytes());
                }
            }

            let response = CanFrame::new_data(0x580u16 + nid as u16, &response)?;
            self.response_tx
                .lock()
                .unwrap()
                .as_ref()
                .ok_or(CanIoError::Disconnected)?
                .send(response)
                .map_err(|_| CanIoError::Disconnected)
        }

        async fn subscribe(&self, _filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            *self.response_tx.lock().unwrap() = Some(tx);
            Ok(Box::new(ScriptedSdoRx { rx }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    struct ScriptedSdoRx {
        rx: mpsc::UnboundedReceiver<CanFrame>,
    }

    #[async_trait]
    impl CanRx for ScriptedSdoRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            self.rx.recv().await.ok_or(CanIoError::Disconnected)
        }

        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            match self.rx.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
            }
        }
    }
}
