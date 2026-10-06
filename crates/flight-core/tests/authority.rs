use flight_core::control::{Intent, Pilot};
use flight_core::{Flight, Sensor, Source, Tick, Vec3};
fn sample(t: u64) -> Sensor {
    Sensor {
        at: Tick(t),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}
fn pilot(thrust: f32) -> Intent {
    Intent::Stabilized(Pilot {
        roll: 0.,
        pitch: 0.,
        yaw: 0.,
        thrust,
    })
}
fn armed() -> Flight {
    let mut f = Flight::default();
    f.tick(Tick(0), Some(sample(0)));
    f.select(Source::Pilot);
    f.command(Source::Pilot, pilot(0.), Tick(0));
    assert!(f.arm(Tick(0)));
    f
}
#[test]
fn a_frame_from_another_live_or_replaced_owner_cannot_apply() {
    let mut a = armed();
    let mut b = armed();
    a.command(Source::Pilot, pilot(0.8), Tick(1000));
    b.command(Source::Pilot, pilot(0.2), Tick(1000));
    let fa = a.tick(Tick(1000), Some(sample(1000)));
    let fb = b.tick(Tick(1000), Some(sample(1000)));
    assert_eq!(a.commit(fb, Tick(1000)), [0.; 4]);
    assert_eq!(b.commit(fa, Tick(1000)), [0.; 4]);
    let mut replacement = armed();
    replacement.tick(Tick(1000), Some(sample(1000)));
    assert_eq!(replacement.commit(fa, Tick(1000)), [0.; 4]);
}
#[test]
fn replacing_diagnostics_revokes_the_previous_frame() {
    let mut f = Flight::default();
    f.tick(Tick(0), Some(sample(0)));
    f.maintenance();
    assert!(f.diagnostic([0.1, 0., 0., 0.], Tick(1000), 10000));
    let frame = f.tick(Tick(1000), Some(sample(1000)));
    assert_eq!(f.commit(frame, Tick(1000)), [0.1, 0., 0., 0.]);
    assert!(f.diagnostic([0., 0.1, 0., 0.], Tick(1001), 10000));
    assert_eq!(f.commit(frame, Tick(1001)), [0.; 4]);
}
