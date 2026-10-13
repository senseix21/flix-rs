//! Swarm S2 properties over random sequences of position samples, targets and ticks.
//! Oracles read only return values, snapshots and committed output (SWM-05 to SWM-08).
use flight_core::control::{GRAVITY, Intent};
use flight_core::position::{PositionTuning, Target, Volume, limit};
use flight_core::{Flight, Quaternion, Reason, Sensor, Source, State, Tick, Vec3};
use proptest::prelude::*;

const SAMPLE_US: u64 = PositionTuning::DEFAULT.sample_us;

fn sensor(us: u64) -> Sensor {
    Sensor {
        at: Tick(us),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0.0, 0.0, GRAVITY),
        calibrated: true,
    }
}
fn coordinate() -> impl Strategy<Value = f32> {
    prop_oneof![
        8 => -4.0f32..=4.0,
        1 => Just(f32::NAN),
        1 => Just(f32::INFINITY),
        1 => Just(-1e30f32),
    ]
}
fn vec3() -> impl Strategy<Value = Vec3> {
    (coordinate(), coordinate(), coordinate()).prop_map(|(x, y, z)| Vec3(x, y, z))
}

#[derive(Clone, Copy, Debug)]
enum Op {
    /// Advance the owner clock by this many microseconds and tick with a fresh IMU sample.
    Tick(u64),
    /// Position sample arriving this many microseconds before (or, if negative, after) now.
    Sample(Vec3, i64),
    Target(Vec3, Vec3),
    /// A valid hold target, so armed position flight lasts long enough to reach staleness.
    Hold(f32),
    Disarm,
}
fn op() -> impl Strategy<Value = Op> {
    prop_oneof![
        8 => (500u64..=10_000).prop_map(Op::Tick),
        2 => (vec3(), -2_000i64..=50_000).prop_map(|(v, d)| Op::Sample(v, d)),
        2 => (vec3(), vec3()).prop_map(|(p, v)| Op::Target(p, v)),
        3 => (0.2f32..=1.5).prop_map(Op::Hold),
        1 => Just(Op::Disarm),
    ]
}

/// Armed, flying a position hold, with one accepted sample at the origin.
fn flying() -> Flight {
    let mut f = Flight::default();
    f.tick(Tick(1000), Some(sensor(1000)));
    f.select(Source::Automatic);
    assert!(f.position(Vec3::ZERO, None, Tick(1000)));
    let idle = Intent::Attitude {
        orientation: Quaternion::IDENTITY,
        feedforward: Vec3::ZERO,
        thrust: 0.0,
    };
    assert!(f.command(Source::Automatic, idle, Tick(1000)));
    assert!(f.arm(Tick(1000)));
    let hold = Intent::Position(Target {
        position: Vec3(0.0, 0.0, 0.5),
        velocity: Vec3::ZERO,
        yaw: 0.0,
    });
    assert!(f.command(Source::Automatic, hold, Tick(1000)));
    f
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 512, ..ProptestConfig::default() })]

    /// Every committed output is finite and in [0, 1]; while flying a position intent,
    /// output is zero once the newest accepted sample is older than the freshness limit;
    /// a rejected sample or target never changes the position estimate or the lease.
    #[test]
    fn position_flight_is_bounded_and_stops_on_stale_position(ops in prop::collection::vec(op(), 1..200)) {
        let mut f = flying();
        let mut now = 1000u64;
        let mut accepted = Some(1000u64);
        for op in ops {
            match op {
                Op::Tick(step) => {
                    now += step;
                    let frame = f.tick(Tick(now), Some(sensor(now)));
                    let out = f.commit(frame, Tick(now));
                    prop_assert!(out.iter().all(|v| v.is_finite() && (0.0..=1.0).contains(v)));
                    let stale = accepted.is_none_or(|at| now.saturating_sub(at) > SAMPLE_US);
                    if stale {
                        prop_assert_eq!(out, [0.0; 4], "output after stale position at {}", now);
                    }
                    let s = f.snapshot();
                    if s.state == State::Fault && stale {
                        prop_assert!(matches!(s.reason, Reason::PositionExpired | Reason::CommandExpired | Reason::Tilt | Reason::InvalidNumeric));
                    }
                }
                Op::Sample(value, back) => {
                    let at = now.saturating_add_signed(-back);
                    let before = f.snapshot().position;
                    let ok = f.position(value, None, Tick(at));
                    let after = f.snapshot().position;
                    if ok {
                        prop_assert!(Volume::DEFAULT.contains(value, 0.5));
                        prop_assert!(accepted.is_none_or(|old| at > old));
                        accepted = Some(at);
                        prop_assert_eq!(after.measured, Some(Tick(at)));
                    } else {
                        prop_assert_eq!(after.measured, before.measured);
                        prop_assert_eq!(after.position, before.position);
                    }
                }
                Op::Target(position, velocity) => {
                    let target = Target { position, velocity, yaw: 0.0 };
                    let ok = f.command(Source::Automatic, Intent::Position(target), Tick(now));
                    if ok {
                        prop_assert!(Volume::DEFAULT.contains(position, 0.0));
                        prop_assert!(velocity.finite() && velocity.norm() <= PositionTuning::DEFAULT.max_speed);
                    }
                }
                Op::Hold(z) => {
                    let target = Target { position: Vec3(0.0, 0.0, z), velocity: Vec3::ZERO, yaw: 0.0 };
                    f.command(Source::Automatic, Intent::Position(target), Tick(now));
                }
                Op::Disarm => f.disarm(),
            }
        }
    }

    /// The limiter bounds tilt and thrust for any finite demand (the Kani harness proves
    /// the same with a square-root contract; this checks it with the real libm).
    #[test]
    fn limiter_bounds_tilt_and_thrust(x in -1e4f32..1e4, y in -1e4f32..1e4, z in -1e4f32..1e4) {
        let t = PositionTuning::DEFAULT;
        let (force, thrust) = limit(Vec3(x, y, z), t).expect("finite demand");
        prop_assert!(force.finite() && (0.0..=t.max_thrust).contains(&thrust));
        prop_assert!(force.2 >= 0.2 * GRAVITY);
        let tilt = libm::atan2f(libm::sqrtf(force.0 * force.0 + force.1 * force.1), force.2);
        prop_assert!(tilt <= t.max_tilt + 1e-5, "tilt {} > {}", tilt, t.max_tilt);
    }
}
