use flight_core::{Snapshot, State, Tick};
pub fn heartbeat(snapshot: Snapshot, pilot_acro: bool) -> [u8; 9] {
    let mut p = [0; 9];
    let mode = match snapshot.source {
        Some(flight_core::Source::Automatic) => 3u32,
        _ if pilot_acro => 1,
        _ => 2,
    };
    p[..4].copy_from_slice(&mode.to_le_bytes());
    p[4] = 2;
    p[5] = 0;
    p[6] = 1 | if snapshot.state == State::Armed {
        128
    } else {
        0
    };
    p[7] = if snapshot.state == State::Fault { 5 } else { 3 };
    p[8] = 3;
    p
}
/// MAVLink `time_boot_ms` is a u32 that wraps after about 49.7 days of uptime.
#[expect(
    clippy::cast_possible_truncation,
    reason = "MAVLink time_boot_ms wraps by definition"
)]
fn boot_ms(at: Tick) -> u32 {
    (at.0 / 1000) as u32
}
/// SCALED_IMU fields are i16 (mg, mrad/s). Values outside the range saturate and NaN
/// reports 0: gyro above about 32.7 rad/s (1,875 deg/s) is clipped in telemetry only.
#[expect(
    clippy::cast_possible_truncation,
    reason = "float-to-int `as` saturates; clipping is the intended telemetry behaviour"
)]
fn saturating_i16(value: f32) -> i16 {
    value as i16
}
pub fn attitude(snapshot: Snapshot, at: Tick) -> [u8; 28] {
    let mut p = [0; 28];
    p[..4].copy_from_slice(&boot_ms(at).to_le_bytes());
    let v = snapshot.estimate.attitude.angles();
    let r = snapshot.estimate.rates;
    for (i, f) in [v.0, -v.1, -v.2, r.0, -r.1, -r.2].iter().enumerate() {
        p[4 + i * 4..8 + i * 4].copy_from_slice(&f.to_le_bytes());
    }
    p
}
pub fn attitude_quaternion(snapshot: Snapshot, at: Tick) -> [u8; 32] {
    let mut p = [0; 32];
    p[..4].copy_from_slice(&boot_ms(at).to_le_bytes());
    let [w, x, y, z] = snapshot.estimate.attitude.array();
    let r = snapshot.estimate.rates;
    for (i, f) in [w, x, -y, -z, r.0, -r.1, -r.2].iter().enumerate() {
        p[4 + i * 4..8 + i * 4].copy_from_slice(&f.to_le_bytes());
    }
    p
}
pub fn scaled_imu(sensor: flight_core::Sensor) -> [u8; 22] {
    let mut p = [0; 22];
    p[..4].copy_from_slice(&boot_ms(sensor.at).to_le_bytes());
    let a = sensor.acceleration;
    let g = sensor.gyro;
    let scale = 1000.0 / flight_core::control::GRAVITY;
    for (i, f) in [
        a.0 * scale,
        -a.1 * scale,
        -a.2 * scale,
        g.0 * 1000.0,
        -g.1 * 1000.0,
        -g.2 * 1000.0,
    ]
    .iter()
    .enumerate()
    {
        p[4 + i * 2..6 + i * 2].copy_from_slice(&saturating_i16(*f).to_le_bytes());
    }
    p
}
/// HIGHRES_IMU (105) payload: the exact f32 sample in FRD, so the full MPU6050 range
/// is reported where SCALED_IMU's i16 mrad/s clips near 1,877 deg/s. Non-finite axes
/// are sent as 0 with their `fields_updated` bit clear; unmeasured fields stay clear.
pub fn highres_imu(sensor: flight_core::Sensor) -> [u8; 63] {
    let mut p = [0; 63];
    p[..8].copy_from_slice(&sensor.at.0.to_le_bytes());
    let a = sensor.acceleration;
    let g = sensor.gyro;
    let mut updated = 0u16;
    // Payload order xacc..zgyro matches HIGHRES_IMU_UPDATED_* bits 0..=5.
    for (i, v) in [a.0, -a.1, -a.2, g.0, -g.1, -g.2].into_iter().enumerate() {
        if v.is_finite() {
            p[8 + i * 4..12 + i * 4].copy_from_slice(&v.to_le_bytes());
            updated |= 1 << i;
        }
    }
    // Magnetometer, pressure and temperature (offsets 32..60) are not measured.
    p[60..62].copy_from_slice(&updated.to_le_bytes());
    p[62] = 0; // sensor instance id
    p
}
pub fn rc(channels: [u16; 8], at: Tick) -> [u8; 22] {
    let mut p = [0; 22];
    p[..4].copy_from_slice(&boot_ms(at).to_le_bytes());
    for (i, v) in channels.iter().enumerate() {
        p[4 + i * 2..6 + i * 2].copy_from_slice(&v.to_le_bytes());
    }
    p[21] = 255;
    p
}
pub fn unknown_battery() -> [u8; 36] {
    let mut p = [0; 36];
    p[..8].fill(255);
    p[8..10].copy_from_slice(&i16::MAX.to_le_bytes());
    p[10..32].fill(255);
    p[35] = 255;
    p
}
/// Only pack voltage is measured; current, temperature, energy and capacity stay unknown.
/// No invented per-cell measurements: MAVLink permits total voltage in first slot.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "0 < volts * 1000 < 65535 is checked before the conversion"
)]
pub fn battery_voltage(volts: f32, cells: u8) -> [u8; 36] {
    let mut p = unknown_battery();
    if !volts.is_finite() || volts <= 0.0 || !(1..=10).contains(&cells) || volts * 1000.0 >= 65535.0
    {
        return p;
    }
    p[10..12].copy_from_slice(&((volts * 1000.0) as u16).to_le_bytes());
    p
}
pub fn motors(values: [f32; 4], at: Tick) -> [u8; 41] {
    let mut p = [0; 41];
    p[..8].copy_from_slice(&at.0.to_le_bytes());
    for (i, f) in values.iter().enumerate() {
        p[8 + i * 4..12 + i * 4].copy_from_slice(&f.to_le_bytes());
    }
    p // all remaining controls initialized zero
}
pub fn ack(command: u16, result: u8, system: u8, component: u8) -> [u8; 10] {
    let mut p = [0; 10];
    p[..2].copy_from_slice(&command.to_le_bytes());
    p[2] = result;
    p[3] = 255;
    p[8] = system;
    p[9] = component;
    p
}

/// Advertise only implemented MAVLink 2, float/C-cast parameters and attitude target.
/// Unknown board IDs and uncommitted source identity remain zero.
pub fn autopilot_version() -> [u8; 60] {
    let mut payload = [0; 60];
    let capabilities = 2_u64 | 64 | 8192 | 131072;
    payload[..8].copy_from_slice(&capabilities.to_le_bytes());
    // Workspace 0.1.0 development build, FIRMWARE_VERSION_TYPE_DEV=0.
    payload[16..20].copy_from_slice(&0x0001_0000u32.to_le_bytes());
    payload
}

/// Explicit negotiation: this endpoint implements MAVLink 2 only.
pub fn protocol_version() -> [u8; 22] {
    let mut payload = [0; 22];
    for offset in [0, 2, 4] {
        payload[offset..offset + 2].copy_from_slice(&200_u16.to_le_bytes());
    }
    payload
}
