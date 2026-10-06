use flight_core::{
    Sensor, Source, State, Tick, Vec3,
    battery::{Config, Guard, Health, Reading},
    control::{Intent, Pilot},
};
use flight_esp32::Owner;
use flight_protocol::Event;
fn sample(at: Tick) -> Sensor {
    Sensor {
        at,
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}
fn config() -> Config {
    Config::new(1, 3.0, 3.5, 4.3, 0.1, 2000, 1000).unwrap()
}
#[test]
fn invalid_stale_and_debounced_voltage_have_explicit_states() {
    let mut guard = Guard::new(config());
    let reading = |volts, at| Reading::Voltage {
        volts,
        at: Tick(at),
    };
    assert_eq!(
        guard.evaluate(Reading::Unavailable, Tick(0)),
        Health::Unavailable
    );
    assert_eq!(
        guard.evaluate(reading(f32::NAN, 0), Tick(0)),
        Health::Invalid
    );
    assert_eq!(guard.evaluate(reading(3.9, 0), Tick(2001)), Health::Stale);
    assert_eq!(
        guard.evaluate(reading(3.9, 3000), Tick(3000)),
        Health::Healthy
    );
    assert_eq!(
        guard.evaluate(reading(3.5, 4000), Tick(4000)),
        Health::Warning
    );
    assert_eq!(
        guard.evaluate(reading(3.55, 5000), Tick(5000)),
        Health::Warning
    );
    assert_eq!(
        guard.evaluate(reading(3.6, 6000), Tick(6000)),
        Health::Healthy
    );
    assert_eq!(
        guard.evaluate(reading(3.0, 7000), Tick(7000)),
        Health::PendingCritical
    );
    assert_eq!(
        guard.evaluate(reading(3.0, 8000), Tick(8000)),
        Health::Critical
    );
    assert!(Config::new(0, 3., 3.5, 4.3, 0.1, 2000, 1000).is_none());
}
#[test]
fn opt_in_unknown_denies_arm_and_critical_latches_zero_output() {
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(Tick(0))));
    owner.flight.maintenance();
    assert!(owner.configure_battery(config()));
    owner.flight.disarm();
    owner.flight.select(Source::Pilot);
    let intent = |thrust| {
        Intent::Stabilized(Pilot {
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
            thrust,
        })
    };
    owner.submit(Event::Setpoint(Source::Pilot, intent(0.)), Tick(1000));
    owner.submit(Event::Arm, Tick(1000));
    owner.tick(Tick(1000), Some(sample(Tick(1000))));
    assert_eq!(owner.flight.snapshot().state, State::Standby);
    owner.battery_reading(Reading::Voltage {
        volts: 3.9,
        at: Tick(2000),
    });
    owner.submit(Event::Arm, Tick(2000));
    owner.tick(Tick(2000), Some(sample(Tick(2000))));
    assert_eq!(owner.flight.snapshot().state, State::Armed);
    owner.submit(Event::Setpoint(Source::Pilot, intent(0.4)), Tick(3000));
    owner.battery_reading(Reading::Voltage {
        volts: 3.0,
        at: Tick(3000),
    });
    assert!(
        owner
            .tick(Tick(3000), Some(sample(Tick(3000))))
            .iter()
            .any(|v| *v > 0.)
    );
    owner.battery_reading(Reading::Voltage {
        volts: 3.0,
        at: Tick(4000),
    });
    assert_eq!(owner.tick(Tick(4000), Some(sample(Tick(4000)))), [0.; 4]);
    assert_eq!(owner.flight.snapshot().reason, flight_core::Reason::Power);
    owner.battery_reading(Reading::Voltage {
        volts: 3.9,
        at: Tick(5000),
    });
    owner.submit(Event::Arm, Tick(5000));
    assert_eq!(owner.tick(Tick(5000), Some(sample(Tick(5000)))), [0.; 4]);
    assert_eq!(owner.flight.snapshot().state, State::Fault);
}
