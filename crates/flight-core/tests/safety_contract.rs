use flight_core::control::{GRAVITY, Intent, Pilot, mix};
use flight_core::{Flight, Quaternion, Reason, Sensor, Source, State, Tick, Vec3};
fn sensor(us: u64) -> Sensor {
    Sensor {
        at: Tick(us),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0.0, 0.0, GRAVITY),
        calibrated: true,
    }
}
fn pilot(thrust: f32) -> Intent {
    Intent::Stabilized(Pilot {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        thrust,
    })
}
fn armed() -> Flight {
    let mut f = Flight::default();
    f.tick(Tick(0), Some(sensor(0)));
    assert!(f.select(Source::Pilot));
    assert!(f.command(Source::Pilot, pilot(0.0), Tick(0)));
    assert!(f.arm(Tick(0)));
    f
}
#[test]
fn disarm_invalidates_cached_and_invalid_intents() {
    let mut f = armed();
    assert!(f.command(Source::Pilot, pilot(0.4), Tick(1000)));
    let cached = f.tick(Tick(1000), Some(sensor(1000)));
    assert!(f.commit(cached, Tick(1000)).iter().any(|v| *v > 0.0));
    let generation = f.snapshot().generation;
    f.disarm();
    assert!(f.snapshot().generation > generation);
    assert!(!f.command(Source::Pilot, pilot(f32::NAN), Tick(2000)));
    assert_eq!(f.commit(cached, Tick(1000)), [0.0; 4]);
    let stopped = f.tick(Tick(2000), Some(sensor(2000)));
    assert_eq!(f.commit(stopped, Tick(2000)), [0.0; 4]);
}
#[test]
fn leases_expire_without_heartbeat_or_implicit_rearm() {
    let mut f = armed();
    f.command(Source::Pilot, pilot(0.6), Tick(0));
    for i in 1..=1001 {
        let now = Tick(i * 1000);
        let frame = f.tick(now, Some(sensor(now.0)));
        if i == 1001 {
            assert_eq!(f.snapshot().reason, Reason::CommandExpired);
            assert_eq!(f.commit(frame, now), [0.0; 4]);
        }
    }
    assert_eq!(f.snapshot().state, State::Failsafe);
    assert!(!f.arm(Tick(1_001_000)));
    assert!(f.command(Source::Pilot, pilot(0.0), Tick(1_001_000)));
    assert!(!f.arm(Tick(1_001_000)));
}
#[test]
fn every_supported_intent_obeys_frame_expiry_and_disarm() {
    let intents = [
        pilot(0.4),
        Intent::Acro(Pilot {
            roll: 0.1,
            pitch: 0.0,
            yaw: 0.0,
            thrust: 0.4,
        }),
        Intent::Attitude {
            orientation: Quaternion::IDENTITY,
            feedforward: Vec3::ZERO,
            thrust: 0.4,
        },
        Intent::Rates {
            rates: Vec3(0.1, 0.0, 0.0),
            thrust: 0.4,
        },
    ];
    for intent in intents {
        let mut f = Flight::default();
        f.tick(Tick(0), Some(sensor(0)));
        let source = if intent.kind() >= 2 {
            Source::Automatic
        } else {
            Source::Pilot
        };
        f.select(source);
        let low = match intent {
            Intent::Stabilized(_) => pilot(0.0),
            Intent::Acro(p) => Intent::Acro(Pilot { thrust: 0.0, ..p }),
            Intent::Attitude {
                orientation,
                feedforward,
                ..
            } => Intent::Attitude {
                orientation,
                feedforward,
                thrust: 0.0,
            },
            Intent::Rates { rates, .. } => Intent::Rates { rates, thrust: 0.0 },
        };
        assert!(f.command(source, low, Tick(0)));
        assert!(f.arm(Tick(0)));
        assert!(f.command(source, intent, Tick(1000)));
        let frame = f.tick(Tick(1000), Some(sensor(1000)));
        assert!(f.commit(frame, Tick(1000)).iter().any(|v| *v > 0.0));
        assert_eq!(f.commit(frame, Tick(3000)), [0.0; 4]);
        f.disarm();
        assert_eq!(f.commit(frame, Tick(1000)), [0.0; 4]);
    }
}
#[test]
fn arm_requires_source_calibration_freshness_and_low_thrust() {
    let mut f = Flight::default();
    assert!(!f.arm(Tick(0)));
    f.tick(Tick(0), Some(sensor(0)));
    assert!(!f.arm(Tick(0)));
    f.select(Source::Pilot);
    f.command(Source::Pilot, pilot(0.4), Tick(0));
    assert!(!f.arm(Tick(0)));
    f.command(Source::Pilot, pilot(0.0), Tick(0));
    assert!(!f.arm(Tick(2001)));
    assert!(f.arm(Tick(0)));
    assert!(!f.select(Source::Automatic));
    let mut f = Flight::default();
    let mut s = sensor(0);
    s.calibrated = false;
    f.tick(Tick(0), Some(s));
    f.select(Source::Pilot);
    f.command(Source::Pilot, pilot(0.0), Tick(0));
    assert!(!f.arm(Tick(0)));
}
#[test]
fn invalid_sensor_clock_and_reset_never_preserve_output() {
    for case in 0..4 {
        let mut f = armed();
        f.command(Source::Pilot, pilot(0.4), Tick(1000));
        let old = f.tick(Tick(1000), Some(sensor(1000)));
        let now = if case == 1 { Tick(500) } else { Tick(2000) };
        let mut s = sensor(now.0);
        if case == 0 {
            s.gyro.0 = f32::NAN;
        }
        if case == 2 {
            s.at = Tick(0);
        }
        if case == 3 {
            s.calibrated = false;
        }
        let frame = f.tick(now, Some(s));
        assert_eq!(f.snapshot().state, State::Fault);
        assert_eq!(f.commit(frame, now), [0.0; 4]);
        assert_eq!(f.commit(old, now), [0.0; 4]);
        assert!(!f.arm(now));
        let generation = f.snapshot().generation;
        f.reset();
        assert!(f.snapshot().generation > generation);
        assert_eq!(f.snapshot().state, State::Boot);
        assert_eq!(f.commit(old, now), [0.0; 4]);
    }
}
#[test]
fn diagnostic_is_exclusive_bounded_and_expires() {
    let mut f = armed();
    assert!(!f.maintenance());
    f.disarm();
    assert!(f.maintenance());
    assert!(!f.diagnostic([0.4; 4], Tick(0), 1000));
    assert!(f.diagnostic([0.1; 4], Tick(1000), 1000));
    assert!(!f.arm(Tick(1000)));
    assert!(!f.stopped_maintenance());
    let frame = f.tick(Tick(1000), Some(sensor(1000)));
    assert_eq!(f.commit(frame, Tick(1000)), [0.1; 4]);
    assert_eq!(f.commit(frame, Tick(2000)), [0.0; 4]);
    let frame = f.tick(Tick(2000), Some(sensor(2000)));
    assert_eq!(f.commit(frame, Tick(2000)), [0.0; 4]);
    assert!(f.stopped_maintenance());
}
#[test]
fn bounded_outputs_property_and_mixer_signs() {
    let mut seed = 0x5eed_u64;
    for _ in 0..20_000 {
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) as i32) as f32 / i32::MAX as f32
        };
        let thrust = (random() + 1.0) / 2.0;
        let torque = Vec3(random() * 100.0, random() * 100.0, random() * 100.0);
        assert!(
            mix(thrust, torque)
                .unwrap()
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        );
    }
    assert_eq!(mix(0.5, Vec3(0.1, 0.0, 0.0)), Some([0.6, 0.4, 0.4, 0.6]));
    assert_eq!(mix(0.5, Vec3(0.0, 0.1, 0.0)), Some([0.6, 0.6, 0.4, 0.4]));
    assert_eq!(mix(0.5, Vec3(0.0, 0.0, 0.1)), Some([0.4, 0.6, 0.4, 0.6]));
    assert_eq!(mix(0.5, Vec3(f32::NAN, 0.0, 0.0)), None);
}
#[test]
fn quaternion_rotation_and_clock_wrap_contract() {
    let q = Quaternion::euler(Vec3(0.0, 0.0, core::f32::consts::FRAC_PI_2)).unwrap();
    let v = q.body(Vec3(1.0, 0.0, 0.0));
    assert!(v.0.abs() < 1e-6 && (v.1 + 1.0).abs() < 1e-6);
    assert!(Quaternion::new([0.0; 4]).is_none());
    assert!(
        Vec3::between(Vec3::UP, Vec3(0.0, 0.0, -1.0))
            .unwrap()
            .finite()
    );
    let mut clock = flight_core::time::Clock32::default();
    let a = clock.extend(u32::MAX - 10).unwrap();
    let b = clock.extend(20).unwrap();
    assert_eq!(b.age(a), Some(31));
    assert!(clock.extend(19).is_none());
}
#[test]
fn transition_stress_stop_dominates_at_every_seeded_step() {
    let mut f = Flight::default();
    let mut seed = 7u64;
    for tick in 0..10_000 {
        let at = Tick(tick * 1000);
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        match seed % 13 {
            0 => f.disarm(),
            1 => {
                f.clear_fault();
            }
            2 => {
                f.select(Source::Pilot);
            }
            3 => {
                f.arm(at);
            }
            4 => {
                f.maintenance();
            }
            5 => f.reset(),
            _ => {
                f.command(Source::Pilot, pilot((seed % 100) as f32 / 100.0), at);
            }
        }
        let frame = f.tick(at, Some(sensor(at.0)));
        let m = f.commit(frame, at);
        assert!(m.iter().all(|v| v.is_finite() && (0.0..=1.0).contains(v)));
        if !matches!(f.snapshot().state, State::Armed | State::Maintenance) {
            assert_eq!(m, [0.0; 4]);
        }
        f.disarm();
        assert_eq!(f.commit(frame, at), [0.0; 4]);
    }
}
