//! ADR 003 fault policy at the owner: SensorInit clears only after a passing driver
//! rebuild; latched faults survive clear-fault and reset; other faults clear directly.
use flight_core::{Reason, Sensor, State, Tick, Vec3};
use flight_esp32::Owner;
use flight_protocol::{Console, Event};

fn sample(us: u64) -> Sensor {
    Sensor {
        at: Tick(us),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}

fn clear(owner: &mut Owner, ms: u64) {
    owner.submit(Event::Console(Console::ClearFault), Tick(ms * 1000));
    owner.tick(Tick(ms * 1000), Some(sample(ms * 1000)));
}

fn state(owner: &Owner) -> (State, Reason) {
    let s = owner.flight.snapshot();
    (s.state, s.reason)
}

#[test]
fn sensor_init_clears_only_after_a_passing_rebuild() {
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(0)));
    owner.flight.fault(Reason::SensorInit);

    clear(&mut owner, 1);
    assert_eq!(state(&owner), (State::Fault, Reason::SensorInit));
    assert!(
        owner.sensor_retry_requested,
        "clear-fault must request a rebuild"
    );

    owner.sensor_retry_result(false);
    assert_eq!(state(&owner), (State::Fault, Reason::SensorInit));
    assert!(!owner.sensor_retry_requested);

    clear(&mut owner, 2);
    owner.sensor_retry_result(true);
    assert_eq!(state(&owner).0, State::Boot);
    // Clearing never arms: no lease, no explicit arm.
    owner.submit(Event::Arm, Tick(3000));
    owner.tick(Tick(3000), Some(sample(3000)));
    assert_ne!(state(&owner).0, State::Armed);
}

#[test]
fn a_late_rebuild_result_cannot_clear_a_different_fault() {
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(0)));
    owner.flight.fault(Reason::SensorInit);
    clear(&mut owner, 1);
    // Before the rebuild finishes, the fault becomes a latched configuration fault.
    owner.flight.fault(Reason::Configuration);
    owner.sensor_retry_result(true);
    assert_eq!(state(&owner), (State::Fault, Reason::Configuration));
}

#[test]
fn latched_faults_survive_clear_and_reset_other_faults_clear() {
    for (reason, latched) in [
        (Reason::Configuration, true),
        (Reason::Tilt, false),
        (Reason::Clock, false),
        (Reason::Power, false),
    ] {
        let mut owner = Owner::default();
        owner.tick(Tick(0), Some(sample(0)));
        owner.flight.fault(reason);
        clear(&mut owner, 1);
        let after_clear = state(&owner);
        owner.reset();
        let after_reset = state(&owner);
        if latched {
            assert_eq!(after_clear, (State::Fault, reason), "{reason:?} clear");
            assert_eq!(after_reset, (State::Fault, reason), "{reason:?} reset");
        } else {
            assert_ne!(after_clear.0, State::Fault, "{reason:?} clear");
            assert!(!owner.sensor_retry_requested);
        }
    }
}

// ADR 003 D3: operator-facing reasons.
use flight_core::Source;
use flight_core::control::{Intent, Pilot};

/// Tick once at `ms` and return the STATUSTEXT (severity, text) replies.
fn tick_notices(owner: &mut Owner, ms: u64) -> Vec<(u8, String)> {
    owner.tick(Tick(ms * 1000), Some(sample(ms * 1000)));
    let mut out = Vec::new();
    while let Some(r) = owner.reply() {
        let b = &r.bytes[..r.len];
        if u32::from_le_bytes([b[7], b[8], b[9], 0]) == 253 {
            let text = &b[11..b.len() - 2];
            let end = text.iter().position(|c| *c == 0).unwrap_or(text.len());
            out.push((b[10], String::from_utf8(text[..end].to_vec()).unwrap()));
        }
    }
    out
}

/// Tick every millisecond in `from..=to` (the owner faults on clock gaps over 10 ms).
fn advance(owner: &mut Owner, from: u64, to: u64) -> Vec<(u8, String)> {
    (from..=to).flat_map(|ms| tick_notices(owner, ms)).collect()
}

fn texts(n: &[(u8, String)]) -> Vec<&str> {
    n.iter().map(|(_, t)| t.as_str()).collect()
}

fn pilot(thrust: f32) -> Intent {
    Intent::Stabilized(Pilot {
        roll: 0.,
        pitch: 0.,
        yaw: 0.,
        thrust,
    })
}

#[test]
fn arm_denial_names_the_first_unmet_condition_and_repeats_are_limited() {
    let mut owner = Owner::default();
    tick_notices(&mut owner, 0);
    owner.submit(Event::Arm, Tick(1000));
    assert_eq!(
        texts(&tick_notices(&mut owner, 1)),
        ["arm denied: no command"]
    );
    owner.submit(Event::Arm, Tick(2000));
    assert!(
        tick_notices(&mut owner, 2).is_empty(),
        "identical text within 1 s"
    );
    assert!(advance(&mut owner, 3, 1001).is_empty());
    owner.submit(Event::Arm, Tick(1_002_000));
    assert_eq!(
        texts(&tick_notices(&mut owner, 1002)),
        ["arm denied: no command"]
    );

    // The owner applies setpoints before queued events within a tick, so select first.
    owner.submit(Event::Select(Source::Pilot), Tick(1_003_000));
    tick_notices(&mut owner, 1003);
    owner.submit(Event::Setpoint(Source::Pilot, pilot(0.5)), Tick(1_004_000));
    tick_notices(&mut owner, 1004);
    owner.submit(Event::Arm, Tick(1_005_000));
    assert_eq!(
        texts(&tick_notices(&mut owner, 1005)),
        ["arm denied: thrust not low"]
    );
}

#[test]
fn fault_and_failsafe_entries_are_announced_once() {
    let mut owner = Owner::default();
    tick_notices(&mut owner, 0);
    owner.flight.fault(Reason::Tilt);
    let n = tick_notices(&mut owner, 1);
    assert_eq!(n, [(2, "fault: tilt".to_string())], "CRITICAL severity");
    assert!(
        tick_notices(&mut owner, 2).is_empty(),
        "no repeat while unchanged"
    );

    let mut owner = Owner::default();
    tick_notices(&mut owner, 0);
    owner.submit(Event::Select(Source::Pilot), Tick(1000));
    tick_notices(&mut owner, 1);
    owner.submit(Event::Setpoint(Source::Pilot, pilot(0.0)), Tick(2000));
    tick_notices(&mut owner, 2);
    owner.submit(Event::Arm, Tick(3000));
    tick_notices(&mut owner, 3);
    assert_eq!(owner.flight.snapshot().state, State::Armed);
    // No further setpoints: the 1 s lease expires into Failsafe.
    let seen = advance(&mut owner, 4, 1100);
    assert_eq!(texts(&seen), ["failsafe: command expired"]);
    assert_eq!(seen[0].0, 4, "WARNING severity");
}

#[test]
fn clear_denials_say_how_to_recover() {
    let mut owner = Owner::default();
    tick_notices(&mut owner, 0);
    owner.flight.fault(Reason::Configuration);
    tick_notices(&mut owner, 1);
    owner.submit(Event::Console(Console::ClearFault), Tick(2000));
    assert_eq!(
        texts(&tick_notices(&mut owner, 2)),
        ["clear denied: configuration (reflash)"]
    );

    let mut owner = Owner::default();
    tick_notices(&mut owner, 0);
    owner.flight.fault(Reason::SensorInit);
    tick_notices(&mut owner, 1);
    owner.submit(Event::Console(Console::ClearFault), Tick(2000));
    tick_notices(&mut owner, 2);
    owner.sensor_retry_result(false);
    assert_eq!(
        texts(&tick_notices(&mut owner, 3)),
        ["clear denied: sensor init"]
    );
    assert_eq!(owner.counters.reply_overflow, 0);
}

#[test]
fn failsafe_arm_denial_names_the_recovery_and_disarm_then_arm_works() {
    // R2-04 Gazebo session: after a link loss QGC sees "disarmed" (Failsafe) and offers only Arm.
    let mut owner = Owner::default();
    tick_notices(&mut owner, 0);
    owner.submit(Event::Select(Source::Pilot), Tick(1000));
    tick_notices(&mut owner, 1);
    owner.submit(Event::Setpoint(Source::Pilot, pilot(0.0)), Tick(2000));
    tick_notices(&mut owner, 2);
    owner.submit(Event::Arm, Tick(3000));
    tick_notices(&mut owner, 3);
    advance(&mut owner, 4, 1100);
    assert_eq!(owner.flight.snapshot().state, State::Failsafe);

    owner.submit(Event::Setpoint(Source::Pilot, pilot(0.0)), Tick(1_101_000));
    tick_notices(&mut owner, 1101);
    owner.submit(Event::Arm, Tick(1_102_000));
    assert_eq!(
        texts(&tick_notices(&mut owner, 1102)),
        ["arm denied: failsafe (send disarm)"]
    );
    assert_eq!(owner.flight.snapshot().state, State::Failsafe);

    owner.submit(Event::Console(Console::Disarm), Tick(1_103_000));
    tick_notices(&mut owner, 1103);
    owner.submit(Event::Select(Source::Pilot), Tick(1_104_000));
    tick_notices(&mut owner, 1104);
    owner.submit(Event::Setpoint(Source::Pilot, pilot(0.0)), Tick(1_105_000));
    tick_notices(&mut owner, 1105);
    owner.submit(Event::Arm, Tick(1_106_000));
    tick_notices(&mut owner, 1106);
    assert_eq!(owner.flight.snapshot().state, State::Armed);
}
