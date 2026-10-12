//! Swarm S2 position contract (SWM-05 to SWM-08, SD-10): samples, setpoints, takeoff, loss.
use flight_core::control::{GRAVITY, Intent};
use flight_core::position::{Target, Volume};
use flight_core::{Flight, Quaternion, Reason, Sensor, Source, State, Tick, Vec3};

fn sensor(us: u64) -> Sensor {
    Sensor {
        at: Tick(us),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0.0, 0.0, GRAVITY),
        calibrated: true,
    }
}
fn idle() -> Intent {
    Intent::Attitude {
        orientation: Quaternion::IDENTITY,
        feedforward: Vec3::ZERO,
        thrust: 0.0,
    }
}
fn hold(z: f32) -> Intent {
    Intent::Position(Target {
        position: Vec3(0.0, 0.0, z),
        velocity: Vec3::ZERO,
        yaw: 0.0,
    })
}
/// Standby, AUTO selected, one position sample at the origin.
fn ready() -> Flight {
    let mut f = Flight::default();
    f.tick(Tick(1000), Some(sensor(1000)));
    assert!(f.select(Source::Automatic));
    assert!(f.position(Vec3::ZERO, Tick(1000)));
    f
}
/// Armed from an idle attitude lease, then switched to a position hold at 0.5 m.
fn flying() -> Flight {
    let mut f = ready();
    assert!(f.command(Source::Automatic, idle(), Tick(1000)));
    assert!(f.arm(Tick(1000)));
    assert!(f.command(Source::Automatic, hold(0.5), Tick(2000)));
    f
}

#[test]
fn samples_are_validated_before_any_state_change() {
    let mut f = ready();
    let before = f.snapshot().position;
    let edge = Volume::DEFAULT.max + Vec3(0.6, 0.0, 0.0);
    for (value, at) in [
        (Vec3(f32::NAN, 0.0, 0.0), 2000),
        (Vec3(0.0, f32::INFINITY, 0.0), 2000),
        (edge, 2000),                  // outside volume plus margin
        (Vec3(0.1, 0.0, 0.0), 1000),   // not newer than the accepted sample
        (Vec3(0.1, 0.0, 0.0), 500),    // older than the owner's clock
        (Vec3(0.1, 0.0, 0.0), 50_000), // further ahead than the clock gap
    ] {
        assert!(!f.position(value, Tick(at)), "{value:?} at {at}");
        assert_eq!(f.snapshot().position.position, before.position);
        assert_eq!(f.snapshot().position.measured, before.measured);
    }
    // Inside the measurement margin below the floor is accepted (sensor noise at rest).
    assert!(f.position(Vec3(0.0, 0.0, -0.02), Tick(2000)));
}

#[test]
fn setpoints_outside_the_volume_or_too_fast_are_rejected() {
    let mut f = flying();
    let max = Volume::DEFAULT.max;
    for target in [
        Target {
            position: Vec3(0.0, 0.0, max.2 + 0.01),
            velocity: Vec3::ZERO,
            yaw: 0.0,
        },
        Target {
            position: Vec3(max.0 + 0.01, 0.0, 1.0),
            velocity: Vec3::ZERO,
            yaw: 0.0,
        },
        Target {
            position: Vec3(0.0, 0.0, 1.0),
            velocity: Vec3(3.0, 0.0, 0.0),
            yaw: 0.0,
        },
        Target {
            position: Vec3(0.0, 0.0, 1.0),
            velocity: Vec3::ZERO,
            yaw: f32::NAN,
        },
    ] {
        assert!(!f.command(Source::Automatic, Intent::Position(target), Tick(2000)));
    }
}

#[test]
fn position_intent_cannot_arm_and_pilot_cannot_send_it() {
    let mut f = ready();
    assert!(f.command(Source::Automatic, hold(0.0), Tick(1000)));
    assert!(!f.arm(Tick(1000)));
    let mut f = Flight::default();
    f.tick(Tick(1000), Some(sensor(1000)));
    f.select(Source::Pilot);
    assert!(!f.command(Source::Pilot, hold(0.5), Tick(1000)));
}

#[test]
fn takeoff_into_position_control_needs_an_idle_lease() {
    // From idle: allowed while armed.
    let f = flying();
    assert_eq!(f.snapshot().state, State::Armed);
    // From a non-idle attitude lease: the mode change while armed disarms.
    let mut f = ready();
    assert!(f.command(Source::Automatic, idle(), Tick(1000)));
    assert!(f.arm(Tick(1000)));
    let spinning = Intent::Attitude {
        orientation: Quaternion::IDENTITY,
        feedforward: Vec3::ZERO,
        thrust: 0.3,
    };
    assert!(f.command(Source::Automatic, spinning, Tick(1500)));
    assert!(!f.command(Source::Automatic, hold(0.5), Tick(2000)));
    assert_eq!(f.snapshot().state, State::Standby);
    // Leaving position control for another kind while armed also disarms.
    let mut f = flying();
    assert!(!f.command(Source::Automatic, idle(), Tick(3000)));
    assert_eq!(f.snapshot().state, State::Standby);
}

#[test]
fn position_hold_produces_bounded_output_through_the_gate() {
    let mut f = flying();
    let frame = f.tick(Tick(2000), Some(sensor(2000)));
    let out = f.commit(frame, Tick(2000));
    assert!(out.iter().all(|v| (0.0..=1.0).contains(v)) && out.iter().any(|v| *v > 0.1));
    f.disarm();
    assert_eq!(f.commit(frame, Tick(2000)), [0.0; 4]);
}

#[test]
fn stale_position_stops_the_motors() {
    let mut f = flying();
    // The last sample arrived at 1000; the limit is 200 ms. Keep the lease fresh meanwhile.
    let mut now = 2000;
    while now <= 201_000 {
        f.command(Source::Automatic, hold(0.5), Tick(now));
        let frame = f.tick(Tick(now), Some(sensor(now)));
        assert!(f.commit(frame, Tick(now)).iter().any(|v| *v > 0.0), "{now}");
        now += 1000;
    }
    f.command(Source::Automatic, hold(0.5), Tick(now));
    let frame = f.tick(Tick(now), Some(sensor(now)));
    assert_eq!(f.commit(frame, Tick(now)), [0.0; 4]);
    let snapshot = f.snapshot();
    assert_eq!(
        (snapshot.state, snapshot.reason),
        (State::Fault, Reason::PositionExpired)
    );
    // Late samples do not revive authority.
    assert!(!f.position(Vec3::ZERO, Tick(now)));
    assert!(!f.command(Source::Automatic, hold(0.5), Tick(now)));
}

#[test]
fn fresh_samples_keep_position_control_alive() {
    let mut f = flying();
    for k in 0..1000u64 {
        let now = 2000 + k * 1000;
        if k % 20 == 0 {
            assert!(f.position(Vec3::ZERO, Tick(now)));
            f.command(Source::Automatic, hold(0.5), Tick(now));
        }
        f.tick(Tick(now), Some(sensor(now)));
    }
    assert_eq!(f.snapshot().state, State::Armed);
}
