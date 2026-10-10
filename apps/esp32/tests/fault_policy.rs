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
