//! Read-only hardware probe for a Meow Motor.
//!
//! This example performs only SDO uploads. It does not change NMT state, PDO mappings,
//! communication settings, limits, targets, or the motor mode.
//!
//! ```bash
//! cargo run --example meow_motor_probe -- can0 0x01
//! ```

use std::time::Duration;

mod common;
use hex_motor::canopen::sdo;
use hex_motor::meow_motor::{
    calibration, lookup_model, MeowFactoryCalibration, MeowMotorCanSettings, SettingsError,
    SignedQ8_24,
};
use hex_motor::types::MotorIdentity;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let iface = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "vcan0".to_string());
    let node_id = std::env::args()
        .nth(2)
        .map(|value| parse_u8(&value))
        .transpose()?
        .unwrap_or(1);
    let sdo_timeout = Duration::from_millis(250);
    let timeout = Some(sdo_timeout);
    let bus = common::open_bus(&iface).await?;

    let vendor_id = sdo::upload_u32(bus.as_ref(), node_id, 0x1018, 1, timeout).await?;
    let product_code = sdo::upload_u32(bus.as_ref(), node_id, 0x1018, 2, timeout).await?;
    let revision = sdo::upload_u32(bus.as_ref(), node_id, 0x1018, 3, timeout).await?;
    let serial = sdo::upload_u32(bus.as_ref(), node_id, 0x1018, 4, timeout).await?;
    let model = lookup_model(vendor_id, product_code).ok_or_else(|| {
        anyhow::anyhow!(
            "unsupported identity vendor=0x{vendor_id:08X}, product=0x{product_code:08X}"
        )
    })?;

    println!("{} at node 0x{node_id:02X}", model.display_name());
    println!("  identity       = {vendor_id:08X}/{product_code:08X}/{revision:08X}/{serial:08X}");

    // 0x4001 出厂标定：只验 CRC，不验签发 token，也不把系数应用到任何命令。
    let identity = MotorIdentity {
        node_id,
        vendor_id,
        product_code,
        revision_number: revision,
        serial_number: serial,
        product_name: None,
    };
    let calibration =
        calibration::read_from_device(bus.as_ref(), node_id, &identity, sdo_timeout).await?;
    match &calibration {
        MeowFactoryCalibration::Valid(v1) => {
            println!(
                "  0x4001 v1      = torque factor {:.6}, fit RMSE {:.6} Nm, token 0x{:016X} (not verified offline)",
                v1.torque.factor, v1.torque.fit_rmse_nm, v1.issuance_token
            );
            match &v1.friction {
                Some(friction) => println!(
                    "  friction (raw) = static +{:.6}/-{:.6} Nm, kinetic +{:.6}/-{:.6} Nm at {:.3} rad/s, {:.1} C",
                    friction.static_pos_raw_nm,
                    friction.static_neg_raw_nm,
                    friction.kinetic_pos_raw_nm,
                    friction.kinetic_neg_raw_nm,
                    friction.kinetic_reference_speed_rad_per_s,
                    friction.calibration_temperature_c
                ),
                None => println!("  friction (raw) = not calibrated at the factory"),
            }
            println!(
                "  note           = hex-motor never applies these; the caller decides whether to \
                 use raw = torque * factor + friction"
            );
        }
        MeowFactoryCalibration::Missing { reason, .. } => {
            println!("  0x4001         = no usable factory calibration ({reason})");
        }
    }

    let highest = sdo::upload_u8(bus.as_ref(), node_id, 0x2001, 0, timeout).await?;
    MeowMotorCanSettings::validate_highest_subindex(highest)
        .map_err(|error| anyhow::anyhow!(error))?;
    let raw_brs = sdo::upload_u8(bus.as_ref(), node_id, 0x2001, 4, timeout).await?;
    let transmit_pdo_brs = match raw_brs {
        0 => false,
        1 => true,
        value => return Err(SettingsError::InvalidBrsValue(value).into()),
    };
    let settings = MeowMotorCanSettings {
        node_id: sdo::upload_u8(bus.as_ref(), node_id, 0x2001, 1, timeout).await?,
        nominal_bitrate: sdo::upload_u32(bus.as_ref(), node_id, 0x2001, 2, timeout).await?,
        data_bitrate: sdo::upload_u32(bus.as_ref(), node_id, 0x2001, 3, timeout).await?,
        transmit_pdo_brs,
    };
    settings.validate()?;
    println!("  0x2001         = {settings:?}");

    let save_capability = sdo::upload_u32(bus.as_ref(), node_id, 0x1010, 4, timeout).await?;
    let consumer_heartbeat = sdo::upload_u32(bus.as_ref(), node_id, 0x1016, 1, timeout).await?;
    let producer_heartbeat = sdo::upload_u16(bus.as_ref(), node_id, 0x1017, 0, timeout).await?;
    println!("  save 0x1010:04 = {save_capability}");
    println!(
        "  heartbeat      = consumer 0x{consumer_heartbeat:08X}, producer {producer_heartbeat} ms"
    );

    for (index, label) in [(0x4102, "MIT"), (0x4103, "compressed MIT")] {
        match sdo::upload_u8(bus.as_ref(), node_id, index, 0, timeout).await {
            Ok(highest) => println!("  {label:<15} = 0x{index:04X}:00 highest {highest}"),
            Err(error) => println!("  {label:<15} = 0x{index:04X}:00 unavailable ({error})"),
        }
    }

    let mode = sdo::upload_u8(bus.as_ref(), node_id, 0x4402, 0, timeout).await?;
    let detailed_error = sdo::upload_u16(bus.as_ref(), node_id, 0x453F, 0, timeout).await?;
    let position_raw = sdo::upload_u32(bus.as_ref(), node_id, 0x4564, 0, timeout).await? as i32;
    let position = SignedQ8_24::from_raw(position_raw);
    let velocity = sdo::upload_f32(bus.as_ref(), node_id, 0x456C, 0, timeout).await?;
    let torque = sdo::upload_u16(bus.as_ref(), node_id, 0x4577, 0, timeout).await? as i16;
    let driver_temp = sdo::upload_u16(bus.as_ref(), node_id, 0x4104, 1, timeout).await? as i16;
    let motor_temp = sdo::upload_u16(bus.as_ref(), node_id, 0x4104, 2, timeout).await? as i16;
    println!("  state           = mode 0x{mode:02X}, error 0x{detailed_error:04X}");
    println!(
        "  feedback        = {:.9} rev (raw 0x{:08X}), {velocity:.6} rev/s, torque {torque} permille",
        position.to_revolutions(),
        position_raw as u32
    );
    println!(
        "  temperature     = driver {:.1} C, motor {:.1} C",
        driver_temp as f32 / 10.0,
        motor_temp as f32 / 10.0
    );

    let max_torque = sdo::upload_u16(bus.as_ref(), node_id, 0x4572, 0, timeout).await?;
    let peak_torque = sdo::upload_f32(bus.as_ref(), node_id, 0x4576, 0, timeout).await?;
    let profile_velocity = sdo::upload_f32(bus.as_ref(), node_id, 0x4581, 0, timeout).await?;
    let profile_acceleration = sdo::upload_f32(bus.as_ref(), node_id, 0x4583, 0, timeout).await?;
    let profile_deceleration = sdo::upload_f32(bus.as_ref(), node_id, 0x4584, 0, timeout).await?;
    println!("  torque limits   = max {max_torque} permille, peak {peak_torque:.6} Nm");
    println!(
        "  profile         = velocity {profile_velocity:.6} rev/s, acceleration {profile_acceleration:.6} rev/s^2, deceleration {profile_deceleration:.6} rev/s^2"
    );

    Ok(())
}

fn parse_u8(value: &str) -> anyhow::Result<u8> {
    let parsed = if let Some(hex) = value.strip_prefix("0x") {
        u8::from_str_radix(hex, 16)
    } else {
        value.parse()
    }
    .map_err(|error| anyhow::anyhow!("invalid node ID '{value}': {error}"))?;
    if !(1..=127).contains(&parsed) {
        anyhow::bail!("node ID must be 1..=127, got {parsed}");
    }
    Ok(parsed)
}
