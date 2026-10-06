//! A Stop traverses real signed framing, bridge overload, session admission,
//! owner ingress overload and the final motor gate. No mocked policy decisions.
use flight_core::{Sensor, Source, State, Tick, Vec3};
use flight_esp32::{Owner, session::RadioSession};
use flight_protocol::{
    Event,
    wire::{self, Packet, Signing},
};
use flight_proxy::Bridge;
fn sensor(t: u64) -> Sensor {
    Sensor {
        at: Tick(t),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}
#[test]
fn signed_stop_survives_both_queues_and_reset_requires_fresh_authority() {
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sensor(0)));
    owner.flight.maintenance();
    let mut payload = [0; 255];
    payload[..8].copy_from_slice(&100_000_000u64.to_le_bytes());
    payload[8] = 1;
    payload[9] = 1;
    payload[10..42].fill(7);
    let mut session = RadioSession::default();
    session
        .enroll(
            &Packet {
                id: 256,
                system: 255,
                component: 190,
                sequence: 0,
                length: 42,
                payload,
            },
            &mut owner,
            Tick(0),
        )
        .unwrap();
    owner.flight.disarm();
    owner.submit(Event::Select(Source::Pilot), Tick(1000));
    owner.tick(Tick(1000), Some(sensor(1000)));
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
    owner.tick(Tick(2000), Some(sensor(2000)));
    owner.submit(Event::Setpoint(Source::Pilot, intent(0.4)), Tick(3000));
    assert!(
        owner
            .tick(Tick(3000), Some(sensor(3000)))
            .iter()
            .all(|v| *v > 0.)
    );
    let mut signer = Signing::new([7; 32], 100_000_000).unwrap();
    let mut bridge = Bridge::new([2, 1, 2, 3, 4, 5]).unwrap();
    for _ in 0..8 {
        owner.submit(Event::Arm, Tick(4000));
    }
    let mut bytes = [0; 280];
    let n = wire::encode(0, &[0; 9], 0, 255, 190, &mut bytes).unwrap();
    let n = signer.sign(&mut bytes, n, 4000, 0).unwrap();
    for _ in 0..4 {
        bridge.serial(&bytes[..n], 4000).unwrap();
    }
    assert!(bridge.serial(&bytes[..n], 4000).is_err());
    let mut command = [0; 33];
    command[28..30].copy_from_slice(&400u16.to_le_bytes());
    command[30] = 1;
    command[31] = 1;
    let n = wire::encode(76, &command, 1, 255, 190, &mut bytes).unwrap();
    let n = signer.sign(&mut bytes, n, 4000, 0).unwrap();
    bridge.serial(&bytes[..n], 4000).unwrap();
    let stop = bridge.radio(4000).unwrap();
    assert!(session.receive(&stop.bytes[..stop.length], &mut owner, Tick(4000)));
    assert_eq!(owner.tick(Tick(4000), Some(sensor(4000))), [0.; 4]);
    assert_eq!(owner.flight.snapshot().state, State::Standby);
    assert!(bridge.radio(4000).is_none());
    assert!(!session.receive(&stop.bytes[..stop.length], &mut owner, Tick(5000)));
    bridge.reset();
    assert!(bridge.radio(5000).is_none());
    let mut reboot = RadioSession::default();
    assert!(!reboot.receive(&stop.bytes[..stop.length], &mut owner, Tick(5000)));
    assert_eq!(owner.tick(Tick(5000), Some(sensor(5000))), [0.; 4]);
}
