//! Kani model-checking harnesses: `cargo kani -p flight-core`. Each proof covers every
//! value of its symbolic inputs, unlike the sampled property tests.
use crate::control::{Intent, Pilot, mix};
use crate::time::Clock32;
use crate::{Flight, Limits, MotorFrame, Reason, Source, State, Tick, Vec3};

fn any_state() -> State {
    match kani::any::<u8>() % 6 {
        0 => State::Boot,
        1 => State::Standby,
        2 => State::Maintenance,
        3 => State::Armed,
        4 => State::Failsafe,
        _ => State::Fault,
    }
}

fn any_reason() -> Reason {
    match kani::any::<u8>() % 14 {
        0 => Reason::None,
        1 => Reason::Disarmed,
        2 => Reason::NotReady,
        3 => Reason::CommandExpired,
        4 => Reason::SensorExpired,
        5 => Reason::InvalidNumeric,
        6 => Reason::Clock,
        7 => Reason::Generation,
        8 => Reason::Tilt,
        9 => Reason::MaintenanceExpired,
        10 => Reason::Configuration,
        11 => Reason::Power,
        12 => Reason::SensorInit,
        _ => Reason::PositionExpired,
    }
}

fn any_source() -> Source {
    match kani::any::<u8>() % 3 {
        0 => Source::Pilot,
        1 => Source::Automatic,
        _ => Source::Rc,
    }
}

/// A Flight in an arbitrary lifecycle state with arbitrary authority bookkeeping.
/// Float-valued estimator and controller state are left at their defaults; the
/// properties below do not read them.
fn any_flight() -> Flight {
    let mut f = Flight::BOOT;
    f.issuer = kani::any();
    f.state = any_state();
    f.reason = any_reason();
    f.generation = kani::any();
    f.selected = if kani::any() {
        Some(any_source())
    } else {
        None
    };
    f.previous = if kani::any() {
        Some(Tick(kani::any()))
    } else {
        None
    };
    f.sample = if kani::any() {
        Some(Tick(kani::any()))
    } else {
        None
    };
    f.healthy = kani::any();
    f.calibrated = kani::any();
    if kani::any() {
        f.leases[kani::any::<usize>() % 3] = Some(crate::safety::Lease {
            intent: Intent::Stabilized(Pilot {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                thrust: kani::any(),
            }),
            at: Tick(kani::any()),
        });
    }
    if kani::any() {
        f.diagnostic = Some((
            [kani::any(), kani::any(), kani::any(), kani::any()],
            Tick(kani::any()),
            Tick(kani::any()),
        ));
    }
    f
}

fn any_frame() -> MotorFrame {
    MotorFrame {
        issuer: kani::any(),
        values: [kani::any(), kani::any(), kani::any(), kani::any()],
        generation: kani::any(),
        issued: Tick(kani::any()),
        expires: Tick(kani::any()),
    }
}

/// SAF-08: the mixer either refuses or returns four finite outputs in [0, 1].
#[kani::proof]
fn mix_outputs_are_bounded() {
    let out = mix(kani::any(), Vec3(kani::any(), kani::any(), kani::any()));
    if let Some(motors) = out {
        for m in motors {
            assert!(m.is_finite() && (0.0..=1.0).contains(&m));
        }
    }
}

/// Integer clock: a forward step within half a wrap advances monotonically by exactly
/// that step, a backward or ambiguous step is refused, and nothing panics.
#[kani::proof]
fn clock32_extension_is_monotonic() {
    let mut clock = Clock32::default();
    let first: u32 = kani::any();
    let Some(Tick(start)) = clock.extend(first) else {
        panic!("first sample always accepted");
    };
    assert_eq!(start, u64::from(first));
    let second: u32 = kani::any();
    let delta = second.wrapping_sub(first);
    match clock.extend(second) {
        Some(Tick(next)) => {
            assert!(delta <= i32::MAX as u32);
            assert_eq!(next, start + u64::from(delta));
        }
        None => assert!(delta > i32::MAX as u32),
    }
}

/// SAF-01/02/08: for any frame, state and time, the final gate returns finite values in
/// [0, 1], and exactly zero unless the owner is Armed or in Maintenance.
#[kani::proof]
#[kani::unwind(5)]
fn commit_is_bounded_and_stopped_states_are_zero() {
    let f = any_flight();
    let out = f.commit(any_frame(), Tick(kani::any()));
    for m in out {
        assert!(m.is_finite() && (0.0..=1.0).contains(&m));
    }
    if !matches!(f.state, State::Armed | State::Maintenance) {
        assert!(out == [0.0; 4]);
    }
}

/// SAF-01: a frame from another issuer or an older generation never reaches the motors.
#[kani::proof]
#[kani::unwind(5)]
fn commit_rejects_foreign_or_stale_authority() {
    let f = any_flight();
    let frame = any_frame();
    let out = f.commit(frame, Tick(kani::any()));
    if frame.issuer != f.issuer || frame.generation != f.generation || f.issuer == 0 {
        assert!(out == [0.0; 4]);
    }
}

/// SAF-01/05: disarm always leaves Armed and Maintenance, and no operation other than
/// `arm` can enter Armed. Unwind 7 covers `Tuning::valid`'s six-vector check.
#[kani::proof]
#[kani::unwind(7)]
fn only_arm_enters_armed_and_disarm_always_leaves() {
    let mut f = any_flight();
    let before = f.state;
    match kani::any::<u8>() % 7 {
        0 => {
            f.disarm();
            assert!(!matches!(f.state, State::Armed | State::Maintenance));
            return;
        }
        1 => f.fault(Reason::Tilt),
        2 => {
            f.clear_fault();
        }
        3 => {
            f.select(any_source());
        }
        4 => {
            f.maintenance();
        }
        5 => f.reset(),
        _ => {
            f.reconfigure(crate::control::Tuning::DEFAULT, Limits::DEFAULT);
        }
    }
    if before != State::Armed {
        assert!(f.state != State::Armed);
    }
}

/// ADR 003 D1: a latched fault (configuration, generation) never leaves Fault through any
/// operation, including `reset`. Recovery is rebuilding the owner, as a reboot does.
#[kani::proof]
#[kani::unwind(7)]
fn latched_faults_never_leave_fault() {
    let mut f = any_flight();
    kani::assume(f.state == State::Fault);
    kani::assume(matches!(
        f.reason,
        Reason::Configuration | Reason::Generation
    ));
    match kani::any::<u8>() % 7 {
        0 => f.disarm(),
        1 => {
            f.clear_fault();
        }
        2 => {
            f.select(any_source());
        }
        3 => {
            f.maintenance();
        }
        4 => f.reset(),
        5 => {
            f.reconfigure(crate::control::Tuning::DEFAULT, Limits::DEFAULT);
        }
        _ => {
            f.arm(Tick(kani::any()));
        }
    }
    assert!(f.state == State::Fault);
}

/// A maintenance diagnostic reaches the motors only inside its own window
/// `[start, expiry)`, for any clock value, including one earlier than the issue time.
#[kani::proof]
#[kani::unwind(5)]
fn diagnostic_only_inside_its_window() {
    let f = any_flight();
    kani::assume(f.state == State::Maintenance);
    let now = Tick(kani::any());
    let out = f.commit(any_frame(), now);
    let inside = f
        .diagnostic
        .is_some_and(|(_, start, expiry)| start <= now && now < expiry);
    if !inside {
        assert!(out == [0.0; 4]);
    }
}

/// Contract stub for `libm::sqrtf`, whose aarch64 and x86 builds use inline assembly Kani
/// cannot follow. IEEE behaviour outside the finite range (NaN for NaN or negative input,
/// +infinity for +infinity) and, for finite non-negative `x`, any non-negative finite result
/// whose square is within 2^-20 relative of `x`. No input is assumed away, so overflowed
/// intermediate values stay covered.
fn sqrtf_contract(x: f32) -> f32 {
    if x.is_nan() || x < 0.0 {
        return f32::NAN;
    }
    if x.is_infinite() {
        return f32::INFINITY;
    }
    let r: f32 = kani::any();
    kani::assume(r.is_finite() && r >= 0.0);
    kani::assume((r * r - x).abs() <= x * 9.6e-7 + f32::MIN_POSITIVE);
    r
}
/// `tanf` is only evaluated at the constant `max_tilt` (25 degrees) in the harness below.
fn tanf_contract(x: f32) -> f32 {
    assert!((x - crate::position::PositionTuning::DEFAULT.max_tilt).abs() < 1e-7);
    0.466_307_66
}

/// Swarm SWM-08: for any demand (every f32 bit pattern), the position limiter either refuses
/// a non-finite demand or returns a finite force whose vertical component is at least 0.2 g
/// and a thrust in `[0, max_thrust]`, without panicking. The tilt bound itself needs exact
/// reasoning about division and square roots that is too slow here; the property test
/// `limiter_bounds_tilt_and_thrust` checks it against the real `libm`.
#[kani::proof]
#[kani::stub(libm::sqrtf, sqrtf_contract)]
#[kani::stub(libm::tanf, tanf_contract)]
fn position_limit_bounds_thrust_and_refuses_non_finite() {
    use crate::position::{PositionTuning, limit};
    let t = PositionTuning::DEFAULT;
    let demand = Vec3(kani::any(), kani::any(), kani::any());
    match limit(demand, t) {
        None => assert!(!demand.finite()),
        Some((force, thrust)) => {
            assert!(demand.finite());
            assert!(force.finite() && thrust.is_finite());
            assert!(force.2 >= 0.2 * crate::control::GRAVITY);
            assert!((0.0..=t.max_thrust).contains(&thrust));
        }
    }
}
