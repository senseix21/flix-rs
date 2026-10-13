use flight_core::{Sensor, Tick, Vec3};
use flight_esp32::{Owner, session::RadioSession};
use flight_protocol::wire::{self, Packet, Policy, Signing};

fn enrollment(key: u8) -> Packet {
    let mut payload = [0; 255];
    payload[..8].copy_from_slice(&100_000_000u64.to_le_bytes());
    payload[8] = 1;
    payload[9] = 1;
    payload[10..42].fill(key);
    Packet {
        id: 256,
        system: 255,
        component: 190,
        sequence: 0,
        length: 42,
        payload,
    }
}
fn heartbeat(key: u8, now: u64) -> ([u8; 280], usize) {
    let mut out = [0; 280];
    let n = wire::encode(0, &[0; 9], 0, 255, 190, &mut out).unwrap();
    let n = Signing::enrolled([key; 32], 100_000_000, now)
        .unwrap()
        .sign(&mut out, n, now, 0)
        .unwrap();
    (out, n)
}
#[test]
fn enrollment_requires_exclusive_stop_and_rotation_rejects_old_key() {
    let mut owner = Owner::default();
    let mut session = RadioSession::default();
    assert!(
        session
            .enroll(&enrollment(7), &mut owner, Tick(5_000_000))
            .is_err()
    );
    owner.tick(
        Tick(5_000_000),
        Some(Sensor {
            at: Tick(5_000_000),
            gyro: Vec3::ZERO,
            acceleration: Vec3(0., 0., 9.80665),
            calibrated: true,
        }),
    );
    assert!(owner.flight.maintenance());
    assert!(owner.flight.diagnostic([0.1; 4], Tick(5_000_000), 1000));
    assert!(
        session
            .enroll(&enrollment(7), &mut owner, Tick(5_000_000))
            .is_err()
    );
    owner.flight.disarm();
    owner.flight.maintenance();
    session
        .enroll(&enrollment(7), &mut owner, Tick(5_000_000))
        .unwrap();
    let (bytes, n) = heartbeat(7, 5_000_000);
    assert!(session.receive(&bytes[..n], &mut owner, Tick(5_001_000)));
    assert!(!session.receive(&bytes[..n], &mut owner, Tick(5_002_000))); // duplicate
    session
        .enroll(&enrollment(8), &mut owner, Tick(5_003_000))
        .unwrap();
    assert!(!session.receive(&bytes[..n], &mut owner, Tick(5_004_000)));
    let mut reboot = RadioSession::default();
    assert!(!reboot.receive(&bytes[..n], &mut owner, Tick(1_000)));
    session
        .enroll(&enrollment(0), &mut owner, Tick(5_005_000))
        .unwrap();
    assert!(!session.enabled());
}
#[test]
fn enrolled_timestamp_is_anchored_and_bidirectional_signature_is_valid() {
    let mut signer = Signing::enrolled([9; 32], 100_000_000, 900_000_000).unwrap();
    signer.advance(900_010_000);
    let (bytes, n) = heartbeat(9, 0);
    let mut policy = Policy::Signed {
        system: 255,
        component: 190,
        signing: signer,
    };
    wire::decode(&bytes[..n], &mut policy).unwrap(); // No fake 900-second enrollment offset.
    let mut out = [0; 280];
    let n = wire::encode(0, &[0; 9], 0, 1, 1, &mut out).unwrap();
    let Policy::Signed { mut signing, .. } = policy else {
        unreachable!()
    };
    let n = signing.sign(&mut out, n, 900_020_000, 1).unwrap();
    let mut observer = Policy::Signed {
        system: 1,
        component: 1,
        signing: Signing::new([9; 32], 100_000_000).unwrap(),
    };
    wire::decode(&out[..n], &mut observer).unwrap();
    out[n - 1] ^= 1;
    assert!(wire::decode(&out[..n], &mut observer).is_err());
    assert!(Signing::new([9; 32], 1 << 48).is_err());
}
#[test]
fn invalid_rotation_and_clock_rollback_revoke_without_retaining_authority() {
    let mut owner = Owner::default();
    owner.tick(
        Tick(1000),
        Some(Sensor {
            at: Tick(1000),
            gyro: Vec3::ZERO,
            acceleration: Vec3(0., 0., 9.80665),
            calibrated: true,
        }),
    );
    owner.flight.maintenance();
    let mut session = RadioSession::default();
    session
        .enroll(&enrollment(7), &mut owner, Tick(1000))
        .unwrap();
    let mut invalid = enrollment(8);
    invalid.payload[8] = 2;
    assert!(session.enroll(&invalid, &mut owner, Tick(2000)).is_err());
    assert!(!session.enabled());
    session
        .enroll(&enrollment(9), &mut owner, Tick(3000))
        .unwrap();
    let (bytes, n) = heartbeat(9, 3000);
    assert!(!session.receive(&bytes[..n], &mut owner, Tick(2999)));
    assert!(!session.enabled());
    assert!(!session.receive(&bytes[..n], &mut owner, Tick(4000)));
}
#[test]
fn console_commands_reply_with_echo_and_parameter_denials_are_announced() {
    use flight_core::{Sensor, Tick, Vec3};
    use flight_esp32::Owner;
    use flight_protocol::{Console, Event};
    let sample = |t| Sensor {
        at: Tick(t),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    };
    let text = |owner: &mut Owner| -> Vec<String> {
        std::iter::from_fn(|| owner.reply())
            .filter(|r| r.bytes[7] == 126 || r.bytes[7] == 253)
            .map(|r| {
                let payload = &r.bytes[10..r.len - 2];
                if r.bytes[7] == 126 {
                    String::from_utf8_lossy(&payload[9..9 + usize::from(payload[8])]).into_owned()
                } else {
                    String::from_utf8_lossy(&payload[1..])
                        .trim_end_matches('\0')
                        .to_owned()
                }
            })
            .collect()
    };
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(0)));
    owner.submit(Event::Console(Console::Status), Tick(1000));
    owner.tick(Tick(1000), Some(sample(1000)));
    assert!(text(&mut owner).contains(&"> status\nstopped\n".to_owned()));
    let mut name = [0u8; 16];
    name[..11].copy_from_slice(b"CTL_ATT_MAX");
    owner.submit(Event::ParameterSet { name, value: 0.5 }, Tick(2000));
    owner.tick(Tick(2000), Some(sample(2000)));
    let mut seen = Vec::new();
    for t in 3..40 {
        owner.tick(Tick(t * 1000), Some(sample(t * 1000)));
        seen.extend(text(&mut owner));
    }
    assert!(
        seen.iter().any(|t| t == "param denied: maintenance only"),
        "{seen:?}"
    );
    owner.submit(Event::Console(Console::Save), Tick(41000));
    owner.tick(Tick(41000), Some(sample(41000)));
    assert!(text(&mut owner).contains(&"> save\ndenied: maintenance only\n".to_owned()));
}
#[test]
fn console_mode_commands_select_the_source_explicitly_and_never_while_armed() {
    use flight_core::control::{Intent, Pilot};
    use flight_core::{Sensor, Source, State, Tick, Vec3};
    use flight_esp32::Owner;
    use flight_protocol::{Console, Event};
    let sample = |t| Sensor {
        at: Tick(t),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    };
    let stick = |thrust| {
        Intent::Stabilized(Pilot {
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
            thrust,
        })
    };
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(0)));
    // No source yet: manual control alone cannot arm.
    owner.submit(Event::Setpoint(Source::Pilot, stick(0.)), Tick(1000));
    owner.tick(Tick(1000), Some(sample(1000)));
    owner.submit(Event::Arm, Tick(2000));
    owner.tick(Tick(2000), Some(sample(2000)));
    assert_ne!(owner.flight.snapshot().state, State::Armed);
    owner.submit(Event::Console(Console::Mode(2)), Tick(3000));
    owner.tick(Tick(3000), Some(sample(3000)));
    assert_eq!(owner.flight.snapshot().source, Some(Source::Pilot));
    owner.submit(Event::Setpoint(Source::Pilot, stick(0.)), Tick(4000));
    owner.tick(Tick(4000), Some(sample(4000)));
    owner.submit(Event::Arm, Tick(5000));
    owner.tick(Tick(5000), Some(sample(5000)));
    assert_eq!(owner.flight.snapshot().state, State::Armed);
    // Armed: a console mode change is refused and nothing changes.
    owner.submit(Event::Console(Console::Mode(3)), Tick(6000));
    owner.tick(Tick(6000), Some(sample(6000)));
    let s = owner.flight.snapshot();
    assert_eq!((s.state, s.source), (State::Armed, Some(Source::Pilot)));
}
