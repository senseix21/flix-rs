//! IMU telemetry at default rates: both SCALED_IMU (26) and HIGHRES_IMU (105) are
//! emitted from one rate gate, and the bounded reply queue does not overflow when
//! drained as the flight task drains it (at most four replies per 1 ms tick).
use flight_core::{Sensor, Tick, Vec3};
use flight_esp32::Owner;

const DEG: f32 = core::f32::consts::PI / 180.0;

fn sample(at: Tick, gyro: Vec3) -> Sensor {
    Sensor {
        at,
        gyro,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}

fn id(reply: &flight_esp32::runtime::Reply) -> u32 {
    let b = &reply.bytes;
    u32::from_le_bytes([b[7], b[8], b[9], 0])
}

/// Run `seconds` of 1 kHz ticks, draining up to four replies per tick.
fn run(owner: &mut Owner, seconds: u64, gyro: Vec3) -> Vec<(u64, u32, Vec<u8>)> {
    let mut seen = Vec::new();
    for ms in 0..seconds * 1000 {
        let now = Tick(ms * 1000);
        owner.tick(now, Some(sample(now, gyro)));
        for _ in 0..4 {
            let Some(reply) = owner.reply() else { break };
            seen.push((ms, id(&reply), reply.bytes[..reply.len].to_vec()));
        }
    }
    seen
}

#[test]
fn both_imu_messages_share_the_rate_and_fit_the_reply_queue() {
    let mut owner = Owner::default();
    let seen = run(&mut owner, 10, Vec3::ZERO);
    let count = |want| seen.iter().filter(|(_, id, _)| *id == want).count();
    println!("reply_overflow={}", owner.counters.reply_overflow);
    // MAV_RATE_IMU defaults to 15 Hz: 150 samples in 10 s, give or take the boundary.
    assert!((148..=151).contains(&count(26)), "SCALED_IMU {}", count(26));
    assert_eq!(count(105), count(26), "HIGHRES_IMU follows the same gate");
    assert_eq!(owner.counters.reply_overflow, 0, "reply queue overflowed");
    let bytes: usize = seen.iter().map(|(_, _, b)| b.len()).sum();
    // Recorded in evidence: total telemetry bytes/s at default rates on the wire.
    println!("telemetry_bytes_per_second={}", bytes / 10);
}

#[test]
fn every_stream_is_delivered_at_its_configured_rate() {
    let mut owner = Owner::default();
    let seen = run(&mut owner, 10, Vec3::ZERO);
    let count = |want| seen.iter().filter(|(_, id, _)| *id == want).count();
    // Schema defaults over 10 s: SLOW 2 Hz (heartbeat 0 + battery 147), ATT 20 Hz (31),
    // MOT 10 Hz (140), IMU 15 Hz (26 + 105), RC 10 Hz (35). Deferral shifts a message
    // by at most a few ticks, so counts may differ by one at the window edges.
    for (id, hz) in [
        (0, 2),
        (147, 2),
        (31, 20),
        (140, 10),
        (26, 15),
        (105, 15),
        (35, 10),
    ] {
        let want = hz * 10;
        let got = count(id);
        assert!(
            got.abs_diff(want) <= 2,
            "message {id}: {got} sent, {want} expected"
        );
    }
    assert_eq!(owner.counters.reply_overflow, 0);
}

#[test]
fn highres_reports_gyro_beyond_scaled_imu_range() {
    let mut owner = Owner::default();
    // 1,990 deg/s on roll: inside the MPU6050 +/-2000 deg/s range, beyond i16 mrad/s.
    let gyro = Vec3(1990.0 * DEG, 0.0, 0.0);
    let seen = run(&mut owner, 1, gyro);
    let (_, _, highres) = seen
        .iter()
        .find(|(_, id, _)| *id == 105)
        .expect("HIGHRES_IMU");
    let xgyro = f32::from_le_bytes(highres[10 + 20..10 + 24].try_into().unwrap());
    assert_eq!(xgyro, gyro.0, "HIGHRES_IMU carries the exact sample");
    let (_, _, scaled) = seen
        .iter()
        .find(|(_, id, _)| *id == 26)
        .expect("SCALED_IMU");
    let scaled_x = i16::from_le_bytes([scaled[10 + 10], scaled[10 + 11]]);
    assert_eq!(scaled_x, i16::MAX, "SCALED_IMU saturates, as documented");
}
