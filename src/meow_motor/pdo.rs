//! Meow Motor 的 Device TPDO 与共享 Host PDO recipe。

use can_transport::FrameKind;

use crate::canopen::rpdo_config::RpdoRecipe;
use crate::canopen::tpdo_config::{TpdoCommParams, TpdoEntry, TpdoRecipe};
use crate::cia402::PdoProfile;
use crate::error::{Error, Result};

const TPDO1_ENTRIES: [TpdoEntry; 3] = [
    TpdoEntry {
        index: 0x4564,
        subindex: 0,
        bit_len: 32,
    },
    TpdoEntry {
        index: 0x4713,
        subindex: 0,
        bit_len: 16,
    },
    TpdoEntry {
        index: 0x4577,
        subindex: 0,
        bit_len: 16,
    },
];

const TPDO2_ENTRIES: [TpdoEntry; 4] = [
    TpdoEntry {
        index: 0x4402,
        subindex: 0,
        bit_len: 8,
    },
    TpdoEntry {
        index: 0x453F,
        subindex: 0,
        bit_len: 16,
    },
    TpdoEntry {
        index: 0x4104,
        subindex: 1,
        bit_len: 16,
    },
    TpdoEntry {
        index: 0x4104,
        subindex: 2,
        bit_len: 16,
    },
];

const FILLER_U8: TpdoEntry = TpdoEntry {
    index: 0x3000,
    subindex: 1,
    bit_len: 8,
};

const FILLER_U32: TpdoEntry = TpdoEntry {
    index: 0x3000,
    subindex: 3,
    bit_len: 32,
};

/// Select the wire frame kind for any PDO in this protocol.
///
/// BRS has priority over payload length: when enabled, every PDO is FD+BRS. Without
/// BRS, payloads up to 8 bytes use Classic CAN and larger payloads use FD without BRS.
/// Non-PDO traffic must not use this helper and remains Classic CAN.
pub const fn pdo_frame_kind(payload_len: usize, brs: bool) -> FrameKind {
    if brs {
        FrameKind::Fd { brs: true }
    } else if payload_len <= 8 {
        FrameKind::Data
    } else {
        FrameKind::Fd { brs: false }
    }
}

fn validate_node_id(node_id: u8, role: &'static str) -> Result<()> {
    if !(1..=127).contains(&node_id) {
        return Err(Error::Internal(format!(
            "{role} node id must be 1..=127, got {node_id}"
        )));
    }
    Ok(())
}

fn validate_slot(slot: u8, total: u8) -> Result<()> {
    if total == 0 || total > 8 {
        return Err(Error::Internal(format!(
            "shared PDO slot count must be 1..=8, got {total}"
        )));
    }
    if slot >= total {
        return Err(Error::Internal(format!(
            "shared PDO slot {slot} is outside total {total}"
        )));
    }
    Ok(())
}

/// 指定 node 的 Device TPDO1：8 bytes，节拍由 [`PdoProfile`] 给定。
pub fn device_tpdo1_recipe(node_id: u8, profile: PdoProfile) -> Result<TpdoRecipe> {
    validate_node_id(node_id, "motor")?;
    let recipe = TpdoRecipe {
        tpdo_index: 0,
        cob_id: 0x180 + node_id as u16,
        entries: TPDO1_ENTRIES.to_vec(),
        comm: TpdoCommParams {
            transmission_type: 255,
            inhibit_time_x100us: profile.inhibit_time_x100us(),
            event_timer_ms: profile.event_timer_ms(),
        },
    };
    recipe.validate()?;
    Ok(recipe)
}

/// 指定 node 的 Device TPDO2：7 bytes，固定 50 Hz，不做 padding。
pub fn device_tpdo2_recipe(node_id: u8) -> Result<TpdoRecipe> {
    validate_node_id(node_id, "motor")?;
    let recipe = TpdoRecipe {
        tpdo_index: 1,
        cob_id: 0x280 + node_id as u16,
        entries: TPDO2_ENTRIES.to_vec(),
        comm: TpdoCommParams {
            transmission_type: 255,
            inhibit_time_x100us: 190,
            event_timer_ms: 20,
        },
    };
    recipe.validate()?;
    Ok(recipe)
}

/// 从 host node ID 推导 Host TPDO1..4 COB-ID。
pub fn host_tpdo_cob_id(host_node_id: u8, tpdo_index: u8) -> Result<u16> {
    validate_node_id(host_node_id, "host")?;
    if tpdo_index > 3 {
        return Err(Error::Internal(format!(
            "host TPDO index must be 0..=3, got {tpdo_index}"
        )));
    }
    Ok(0x180 + 0x100 * tpdo_index as u16 + host_node_id as u16)
}

/// 让一台电机从共享 Host TPDO1 的指定 8-byte lane 接收 0x4103:02/:03。
pub fn compressed_mit_rpdo1_recipe(
    motor_slot: u8,
    total_motors: u8,
    host_node_id: u8,
) -> Result<RpdoRecipe> {
    validate_slot(motor_slot, total_motors)?;
    let mut entries = Vec::with_capacity(total_motors as usize * 2);
    for slot in 0..total_motors {
        if slot == motor_slot {
            entries.push(TpdoEntry {
                index: 0x4103,
                subindex: 2,
                bit_len: 32,
            });
            entries.push(TpdoEntry {
                index: 0x4103,
                subindex: 3,
                bit_len: 32,
            });
        } else {
            entries.push(FILLER_U32);
            entries.push(FILLER_U32);
        }
    }
    let recipe = RpdoRecipe {
        rpdo_index: 0,
        cob_id: host_tpdo_cob_id(host_node_id, 0)?,
        entries,
        transmission_type: 255,
    };
    recipe.validate()?;
    Ok(recipe)
}

/// 让一台电机从共享 Host TPDO2 的指定 1-byte slot 接收 0x4401 mode command。
pub fn mode_rpdo2_recipe(motor_slot: u8, total_motors: u8, host_node_id: u8) -> Result<RpdoRecipe> {
    validate_slot(motor_slot, total_motors)?;
    let entries = (0..total_motors)
        .map(|slot| {
            if slot == motor_slot {
                TpdoEntry {
                    index: 0x4401,
                    subindex: 0,
                    bit_len: 8,
                }
            } else {
                FILLER_U8
            }
        })
        .collect();
    let recipe = RpdoRecipe {
        rpdo_index: 1,
        cob_id: host_tpdo_cob_id(host_node_id, 1)?,
        entries,
        transmission_type: 255,
    };
    recipe.validate()?;
    Ok(recipe)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn device_tpdo1_is_exactly_eight_bytes_at_every_rate() {
        // 载荷宽度与节拍无关：降频只改通信参数，不改映射，否则解码器要跟着变。
        for ms in [1u16, 2, 4] {
            let recipe = device_tpdo1_recipe(10, PdoProfile::from_event_timer_ms(ms).unwrap())
                .unwrap();
            assert_eq!(recipe.total_bytes(), 8, "payload width @ {ms}ms");
            assert_eq!(recipe.entries, TPDO1_ENTRIES);
            assert_eq!(recipe.comm.event_timer_ms, ms);
            assert_eq!(recipe.comm.inhibit_time_x100us, ms * 10 - 5);
        }
    }

    #[test]
    fn pdo_frame_kind_uses_brs_before_payload_length() {
        assert_eq!(pdo_frame_kind(1, true), FrameKind::Fd { brs: true });
        assert_eq!(pdo_frame_kind(8, true), FrameKind::Fd { brs: true });
        assert_eq!(pdo_frame_kind(64, true), FrameKind::Fd { brs: true });
        assert_eq!(pdo_frame_kind(1, false), FrameKind::Data);
        assert_eq!(pdo_frame_kind(8, false), FrameKind::Data);
        assert_eq!(pdo_frame_kind(9, false), FrameKind::Fd { brs: false });
        assert_eq!(pdo_frame_kind(64, false), FrameKind::Fd { brs: false });
    }

    #[test]
    fn device_tpdo2_is_seven_bytes_without_padding() {
        let recipe = device_tpdo2_recipe(0x2A).unwrap();
        assert_eq!(recipe.total_bytes(), 7);
        assert_eq!(recipe.entries, TPDO2_ENTRIES);
        assert_eq!(recipe.comm.event_timer_ms, 20);
        assert_eq!(recipe.comm.inhibit_time_x100us, 190);
    }

    #[test]
    fn host_cob_ids_derive_from_actual_host_node() {
        assert_eq!(host_tpdo_cob_id(10, 0).unwrap(), 0x18A);
        assert_eq!(host_tpdo_cob_id(10, 1).unwrap(), 0x28A);
        assert_eq!(host_tpdo_cob_id(16, 0).unwrap(), 0x190);
    }

    #[test]
    fn compressed_mit_lane_maps_low_then_high() {
        let recipe = compressed_mit_rpdo1_recipe(2, 6, 10).unwrap();
        assert_eq!(recipe.rpdo_index, 0);
        assert_eq!(recipe.cob_id, 0x18A);
        assert_eq!(recipe.total_bytes(), 48);
        assert_eq!(recipe.entries[4].index, 0x4103);
        assert_eq!(recipe.entries[4].subindex, 2);
        assert_eq!(recipe.entries[5].index, 0x4103);
        assert_eq!(recipe.entries[5].subindex, 3);
        assert_eq!(recipe.entries[0], FILLER_U32);
    }

    #[test]
    fn mode_slot_maps_only_its_own_byte() {
        let recipe = mode_rpdo2_recipe(3, 6, 10).unwrap();
        assert_eq!(recipe.rpdo_index, 1);
        assert_eq!(recipe.cob_id, 0x28A);
        assert_eq!(recipe.total_bytes(), 6);
        assert_eq!(recipe.entries[3].index, 0x4401);
        assert!(recipe
            .entries
            .iter()
            .enumerate()
            .all(|(i, entry)| i == 3 || *entry == FILLER_U8));
    }

    #[test]
    fn recipes_reject_invalid_nodes_and_slots() {
        assert!(device_tpdo1_recipe(0, PdoProfile::default()).is_err());
        assert!(compressed_mit_rpdo1_recipe(6, 6, 10).is_err());
        assert!(mode_rpdo2_recipe(0, 0, 10).is_err());
        assert!(mode_rpdo2_recipe(0, 9, 10).is_err());
    }
}
