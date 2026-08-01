use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use std::time::Instant;

use can_transport::{CanBus, CanFilter, CanFrame, CanId, CanIoError};
use tokio::sync::broadcast;
use tokio_util::sync::CancellationToken;

use super::codec::{decode_tpdo1, decode_tpdo2, TPDO1_LEN, TPDO2_LEN};
use super::entry::MotorEntry;
use super::events::MeowMotorEvent;
use super::pdo::pdo_frame_kind;
use super::types::MeowMotorCanSettingsStatus;

const FUNCTION_CODE_MASK: u16 = 0x780;
const TPDO1_BASE: u16 = 0x180;
const TPDO2_BASE: u16 = 0x280;

#[derive(Debug, Clone, Copy)]
enum TpdoKind {
    Tpdo1,
    Tpdo2,
}

pub(crate) async fn run_tpdo_listener(
    bus: Arc<dyn CanBus>,
    entries: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events: broadcast::Sender<MeowMotorEvent>,
    cancel: CancellationToken,
) {
    let mut tpdo1 = match bus
        .subscribe(CanFilter::standard(TPDO1_BASE, FUNCTION_CODE_MASK))
        .await
    {
        Ok(receiver) => receiver,
        Err(error) => {
            log::error!("Meow Motor TPDO1 subscription failed: {error}");
            return;
        }
    };
    let mut tpdo2 = match bus
        .subscribe(CanFilter::standard(TPDO2_BASE, FUNCTION_CODE_MASK))
        .await
    {
        Ok(receiver) => receiver,
        Err(error) => {
            log::error!("Meow Motor TPDO2 subscription failed: {error}");
            return;
        }
    };

    loop {
        tokio::select! {
            _ = cancel.cancelled() => return,
            result = tpdo1.recv() => handle_frame(result, TpdoKind::Tpdo1, &entries, &events),
            result = tpdo2.recv() => handle_frame(result, TpdoKind::Tpdo2, &entries, &events),
        }
    }
}

fn handle_frame(
    result: std::result::Result<CanFrame, CanIoError>,
    kind: TpdoKind,
    entries: &Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events: &broadcast::Sender<MeowMotorEvent>,
) {
    let frame = match result {
        Ok(frame) => frame,
        Err(error) => {
            log::warn!("Meow Motor {kind:?} receive failed: {error}");
            return;
        }
    };
    let CanId::Standard(cob_id) = frame.id() else {
        return;
    };
    let node_id = (cob_id & 0x7F) as u8;
    if node_id == 0 {
        return;
    }
    let Some(entry) = entries.read().unwrap().get(&node_id).cloned() else {
        return;
    };
    {
        let inner = entry.inner.lock().unwrap();
        // 0x2001:04 applies to every PDO regardless of payload length.  With BRS enabled,
        // even these 8-byte/7-byte Device TPDOs must be FD+BRS.  With BRS disabled they
        // remain Classic because both mappings fit in 8 bytes.
        let expected_kind = match inner.can_settings {
            MeowMotorCanSettingsStatus::Available(settings) => Some(pdo_frame_kind(
                match kind {
                    TpdoKind::Tpdo1 => TPDO1_LEN,
                    TpdoKind::Tpdo2 => TPDO2_LEN,
                },
                settings.transmit_pdo_brs,
            )),
            _ => None,
        };
        if inner.model.is_none() || Some(frame.kind()) != expected_kind {
            return;
        }
    }

    let now = Instant::now();
    let (live, logic_change) = match kind {
        TpdoKind::Tpdo1 => {
            let decoded = match decode_tpdo1(frame.data()) {
                Ok(decoded) => decoded,
                Err(error) => {
                    log::warn!("Meow Motor TPDO1 from 0x{node_id:02X} rejected: {error}");
                    return;
                }
            };
            let mut inner = entry.inner.lock().unwrap();
            inner.apply_tpdo1(decoded, now);
            (inner.build_live_state(now), None)
        }
        TpdoKind::Tpdo2 => {
            let decoded = match decode_tpdo2(frame.data()) {
                Ok(decoded) => decoded,
                Err(error) => {
                    log::warn!("Meow Motor TPDO2 from 0x{node_id:02X} rejected: {error}");
                    return;
                }
            };
            let mut inner = entry.inner.lock().unwrap();
            let previous_logic = inner.logic;
            if let Err(error) = inner.apply_tpdo2(decoded, now) {
                log::warn!("Meow Motor TPDO2 from 0x{node_id:02X} rejected: {error}");
                return;
            }
            let logic_change = (inner.logic != previous_logic).then_some(inner.logic.unwrap());
            (inner.build_live_state(now), logic_change)
        }
    };

    entry.publish(live);
    if let Some(logic) = logic_change {
        let _ = events.send(MeowMotorEvent::LogicChanged { node_id, logic });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::meow_motor::identity::MeowMotorModel;
    use crate::meow_motor::settings::MeowMotorCanSettings;

    fn identified_entry(transmit_pdo_brs: bool) -> Arc<MotorEntry> {
        let entry = Arc::new(MotorEntry::new(1));
        let mut inner = entry.inner.lock().unwrap();
        inner.model = Some(MeowMotorModel::Model4310);
        inner.can_settings = MeowMotorCanSettingsStatus::Available(MeowMotorCanSettings {
            node_id: 1,
            nominal_bitrate: 1_000_000,
            data_bitrate: 5_000_000,
            transmit_pdo_brs,
        });
        drop(inner);
        entry
    }

    #[test]
    fn malformed_frame_does_not_refresh_generation_or_liveness() {
        let entry = identified_entry(false);
        let entries = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let (events, _) = broadcast::channel(4);
        let frame = CanFrame::new_data(0x181, &[0; 7]).unwrap();
        handle_frame(Ok(frame), TpdoKind::Tpdo1, &entries, &events);
        let inner = entry.inner.lock().unwrap();
        assert_eq!(inner.measurements.tpdo1_generation, 0);
        assert!(inner.last_tpdo1.is_none());
        assert!(!inner.online);
    }

    #[test]
    fn device_feedback_requires_fd_brs_when_brs_is_enabled() {
        let entry = identified_entry(true);
        let entries = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let (events, _) = broadcast::channel(4);

        let classic = CanFrame::new_data(0x181, &[0; 8]).unwrap();
        handle_frame(Ok(classic), TpdoKind::Tpdo1, &entries, &events);
        assert_eq!(entry.inner.lock().unwrap().measurements.tpdo1_generation, 0);

        let fd_without_brs = CanFrame::new_fd(0x181, &[0; 8], false).unwrap();
        handle_frame(Ok(fd_without_brs), TpdoKind::Tpdo1, &entries, &events);
        assert_eq!(entry.inner.lock().unwrap().measurements.tpdo1_generation, 0);

        let frame = CanFrame::new_fd(0x181, &[0; 8], true).unwrap();
        handle_frame(Ok(frame), TpdoKind::Tpdo1, &entries, &events);
        assert_eq!(entry.inner.lock().unwrap().measurements.tpdo1_generation, 1);
    }

    #[test]
    fn device_feedback_requires_classic_when_brs_is_disabled_and_payload_fits() {
        let entry = identified_entry(false);
        let entries = Arc::new(RwLock::new(HashMap::from([(1, entry.clone())])));
        let (events, _) = broadcast::channel(4);

        let fd = CanFrame::new_fd(0x181, &[0; 8], false).unwrap();
        handle_frame(Ok(fd), TpdoKind::Tpdo1, &entries, &events);
        assert_eq!(entry.inner.lock().unwrap().measurements.tpdo1_generation, 0);

        let classic = CanFrame::new_data(0x181, &[0; 8]).unwrap();
        handle_frame(Ok(classic), TpdoKind::Tpdo1, &entries, &events);
        assert_eq!(entry.inner.lock().unwrap().measurements.tpdo1_generation, 1);
    }
}
