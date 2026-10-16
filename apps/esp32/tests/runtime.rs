use flight_core::{Sensor, Source, Tick, Vec3};
use flight_esp32::{
    Owner,
    imu::{Bus, Mpu6050},
};
use flight_protocol::{Console, Event};
fn sample(t: u64) -> Sensor {
    Sensor {
        at: Tick(t),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}
#[test]
fn owner_roundtrip_arm_manual_disarm_and_ack() {
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(0)));
    owner.submit(Event::Select(Source::Pilot), Tick(1000));
    owner.tick(Tick(1000), Some(sample(1000)));
    let intent = |thrust| {
        flight_core::control::Intent::Stabilized(flight_core::control::Pilot {
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
            thrust,
        })
    };
    owner.submit(Event::Setpoint(Source::Pilot, intent(0.)), Tick(2000));
    owner.submit(Event::Arm, Tick(2000));
    assert_eq!(owner.tick(Tick(2000), Some(sample(2000))), [0.; 4]);
    owner.submit(Event::Setpoint(Source::Pilot, intent(0.4)), Tick(3000));
    assert!(
        owner
            .tick(Tick(3000), Some(sample(3000)))
            .iter()
            .all(|v| *v > 0.)
    );
    for _ in 0..8 {
        owner.submit(Event::Arm, Tick(4000));
    }
    assert!(owner.submit(Event::Stop, Tick(4000)));
    assert_eq!(owner.tick(Tick(4000), Some(sample(4000))), [0.; 4]);
    assert_eq!(owner.flight.snapshot().state, flight_core::State::Standby);
}
#[test]
fn maintenance_transfer_is_denied_armed_and_snapshot_offsets_work() {
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(0)));
    owner.submit(Event::Console(Console::Maintenance), Tick(1000));
    owner.tick(Tick(1000), Some(sample(1000)));
    owner.submit(
        Event::Log {
            offset: 0,
            count: 12,
        },
        Tick(2000),
    );
    owner.tick(Tick(2000), Some(sample(2000)));
    // The maintenance command's console echo is queued too; find the log chunk.
    let replies: Vec<_> = std::iter::from_fn(|| owner.reply()).collect();
    assert!(replies.iter().any(|r| &r.bytes[17..21] == b"FLXR"));
    owner.submit(Event::Console(Console::Save), Tick(3000));
    owner.tick(Tick(3000), Some(sample(3000)));
    assert!(owner.save_requested);
    owner.submit(Event::Stop, Tick(4000));
    owner.tick(Tick(4000), Some(sample(4000)));
    assert!(!owner.save_requested);
}
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
#[test]
fn imu_readback_identity_and_stationary_calibration_are_required() {
    let mut bus = Fake {
        registers: [0; 256],
    };
    bus.registers[0x75] = 0x68;
    bus.registers[0x3a] = 1;
    bus.registers[0x3f..0x41].copy_from_slice(&8192i16.to_be_bytes());
    let mut imu = Mpu6050::new(
        bus,
        flight_core::Quaternion::IDENTITY,
        Vec3::ZERO,
        Vec3(1., 1., 1.),
    )
    .unwrap();
    imu.configure().unwrap();
    assert!(!imu.sample(Tick(0), true).unwrap().calibrated);
    for i in 1..=2000 {
        let sensor = imu.sample(Tick(i * 1000), true).unwrap();
        if i == 2000 {
            assert!(sensor.calibrated);
            assert!((sensor.acceleration.2 - 9.80665).abs() < 1e-5);
        }
    }
    let bus = Fake {
        registers: [0; 256],
    };
    let mut imu = Mpu6050::new(
        bus,
        flight_core::Quaternion::IDENTITY,
        Vec3::ZERO,
        Vec3(1., 1., 1.),
    )
    .unwrap();
    assert_eq!(imu.configure(), Err(flight_esp32::imu::Error::Identity));
}
