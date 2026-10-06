use flight_core::{Reason, Sensor, State, Tick, Vec3};
use flight_protocol::{Console, Event};
fn sensor(t: u64) -> Sensor {
    Sensor {
        at: Tick(t),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}
fn parameter(name: &str, value: f32) -> Event {
    let mut bytes = [0; 16];
    bytes[..name.len()].copy_from_slice(name.as_bytes());
    Event::ParameterSet { name: bytes, value }
}
#[test]
fn pending_driver_change_cannot_be_erased_by_another_parameter_or_exit() {
    let mut owner = flight_esp32::Owner::default();
    owner.tick(Tick(0), Some(sensor(0)));
    owner.submit(Event::Console(Console::Maintenance), Tick(1000));
    owner.tick(Tick(1000), Some(sensor(1000)));
    owner.submit(parameter("IMU_ROT_YAW", 0.), Tick(2000));
    owner.submit(Event::Console(Console::Exit), Tick(2000));
    owner.tick(Tick(2000), Some(sensor(2000)));
    assert!(owner.driver_restart_requested);
    assert_eq!(owner.flight.snapshot().state, State::Maintenance);
    owner.submit(parameter("CTL_RATE_R_P", 0.08), Tick(3000));
    owner.submit(Event::Console(Console::Exit), Tick(3000));
    owner.tick(Tick(3000), Some(sensor(3000)));
    assert!(owner.driver_restart_requested);
    assert_eq!(owner.flight.snapshot().state, State::Maintenance);
}
#[test]
fn invalid_storage_fault_is_not_cleared_by_console_acknowledgement() {
    let mut flight = flight_core::Flight::default();
    flight.fault(Reason::Configuration);
    assert!(!flight.clear_fault());
    assert_eq!(flight.snapshot().state, State::Fault);
}
#[test]
fn diagnostics_require_maintenance_and_expire_through_the_owner() {
    let mut owner = flight_esp32::Owner::default();
    owner.tick(Tick(0), Some(sensor(0)));
    owner.submit(
        Event::Diagnostic {
            motor: 0,
            throttle: 0.1,
            duration_us: 2000,
        },
        Tick(1000),
    );
    assert_eq!(owner.tick(Tick(1000), Some(sensor(1000))), [0.; 4]);
    owner.submit(Event::Console(Console::Maintenance), Tick(2000));
    owner.tick(Tick(2000), Some(sensor(2000)));
    owner.submit(
        Event::Diagnostic {
            motor: 0,
            throttle: 0.1,
            duration_us: 2000,
        },
        Tick(3000),
    );
    owner.tick(Tick(3000), Some(sensor(3000)));
    assert_eq!(
        owner.tick(Tick(4000), Some(sensor(4000))),
        [0.1, 0., 0., 0.]
    );
    assert_eq!(owner.tick(Tick(5000), Some(sensor(5000))), [0.; 4]);
}
