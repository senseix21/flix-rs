//! MPU6050 register saturation is detected per axis and reported to the operator,
//! without changing what the controller receives.
use flight_core::{Quaternion, Sensor, Tick, Vec3};
use flight_esp32::Owner;
use flight_esp32::imu::{ACCEL_SATURATED, Bus, GYRO_SATURATED, Mpu6050};

struct Fake {
    registers: [u8; 256],
}
impl Bus for Fake {
    type Error = ();
    fn read(&mut self, r: u8, out: &mut [u8]) -> Result<(), ()> {
        out.copy_from_slice(&self.registers[r as usize..r as usize + out.len()]);
        Ok(())
    }
    fn write(&mut self, r: u8, v: u8) -> Result<(), ()> {
        self.registers[r as usize] = v;
        Ok(())
    }
}

/// Registers 0x3b..0x48: accel x/y/z, temperature, gyro x/y/z, big-endian i16.
fn imu(values: [i16; 7]) -> Mpu6050<Fake> {
    let mut bus = Fake {
        registers: [0; 256],
    };
    bus.registers[0x3a] = 1;
    for (i, v) in values.into_iter().enumerate() {
        bus.registers[0x3b + i * 2..0x3d + i * 2].copy_from_slice(&v.to_be_bytes());
    }
    Mpu6050::new(bus, Quaternion::IDENTITY, Vec3::ZERO, Vec3(1.0, 1.0, 1.0)).unwrap()
}

fn mask(values: [i16; 7]) -> u8 {
    let mut imu = imu(values);
    imu.sample(Tick(0), false).expect("sample");
    imu.saturation()
}

#[test]
fn each_axis_maps_to_its_bit_at_either_endpoint() {
    // (register slot, mask bit): accel 0..=2 -> bits 3..=5, gyro 4..=6 -> bits 0..=2.
    for (slot, bit) in [(0, 3), (1, 4), (2, 5), (4, 0), (5, 1), (6, 2)] {
        for limit in [i16::MAX, i16::MIN] {
            let mut v = [0; 7];
            v[slot] = limit;
            assert_eq!(mask(v), 1 << bit, "slot {slot} at {limit}");
        }
    }
    assert_eq!(GYRO_SATURATED | ACCEL_SATURATED, 0b11_1111);
}

#[test]
fn one_count_inside_the_range_and_temperature_never_saturate() {
    assert_eq!(
        mask([i16::MAX - 1, i16::MIN + 1, 0, i16::MAX, 32766, -32767, 0]),
        0
    );
    assert_eq!(mask([i16::MAX; 7]), 0b11_1111);
}

#[test]
fn saturation_does_not_change_the_sample() {
    let v = [0, 0, 8192, 0, i16::MAX, 0, 0];
    let mut saturated = imu(v);
    let sample = saturated.sample(Tick(0), false).unwrap();
    assert_eq!(saturated.saturation(), 1);
    let expected = f32::from(i16::MAX) * core::f32::consts::PI / (180.0 * 16.4);
    assert_eq!(
        sample.gyro.0, expected,
        "controller still sees the clipped value"
    );
}

fn sample(us: u64) -> Sensor {
    Sensor {
        at: Tick(us),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}

/// Tick at 1 kHz, recording `mask` each tick; returns STATUSTEXT texts with their tick.
fn run(owner: &mut Owner, from_ms: u64, ms: u64, mask: impl Fn(u64) -> u8) -> Vec<(u64, String)> {
    let mut notices = Vec::new();
    for t in from_ms..from_ms + ms {
        owner.record_imu_saturation(mask(t));
        owner.tick(Tick(t * 1000), Some(sample(t * 1000)));
        for _ in 0..4 {
            let Some(r) = owner.reply() else { break };
            let b = &r.bytes[..r.len];
            if u32::from_le_bytes([b[7], b[8], b[9], 0]) == 253 {
                assert_eq!(b[10], 4, "severity WARNING");
                let text = &b[11..b.len() - 2];
                let end = text.iter().position(|c| *c == 0).unwrap_or(text.len());
                notices.push((t, String::from_utf8(text[..end].to_vec()).unwrap()));
            }
        }
    }
    notices
}

#[test]
fn operator_notice_is_rate_limited_and_names_every_axis() {
    let mut owner = Owner::default();
    // Accel z saturates for 15 ticks (a landing), gyro x for one tick inside the same second.
    let notices = run(&mut owner, 0, 3000, |t| match t {
        100..115 => 0b100_000,
        400 => 0b000_001,
        _ => 0,
    });
    assert_eq!(notices.len(), 2, "{notices:?}");
    assert_eq!(notices[0], (100, "IMU saturated: az".to_string()));
    // Accel z kept saturating on ticks 101..=114 and gyro x at 400, all during the 1 s
    // hold-off: both are reported in the next notice, not lost.
    assert_eq!(notices[1], (1100, "IMU saturated: gx az".to_string()));
    assert_eq!(owner.counters.imu_saturated_samples, [1, 0, 0, 0, 0, 15]);
    assert_eq!(owner.counters.reply_overflow, 0);
}

#[test]
fn continuous_saturation_gives_one_notice_per_second() {
    let mut owner = Owner::default();
    let notices = run(&mut owner, 0, 5000, |_| GYRO_SATURATED);
    assert_eq!(notices.len(), 5, "{notices:?}");
    assert!(notices.windows(2).all(|w| w[1].0 - w[0].0 >= 1000));
    assert_eq!(notices[0].1, "IMU saturated: gx gy gz");
    assert_eq!(owner.counters.imu_saturated_samples[..3], [5000; 3]);
}

#[test]
fn reset_discards_pending_notices() {
    let mut owner = Owner::default();
    run(&mut owner, 0, 10, |t| u8::from(t == 5));
    owner.record_imu_saturation(ACCEL_SATURATED);
    owner.reset();
    assert!(run(&mut owner, 10, 3000, |_| 0).is_empty());
}
