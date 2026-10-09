use flight_gazebo::*;
#[test]
fn handles_instances_nulls_versions_and_reset_contract() {
    assert_eq!(std::mem::size_of::<Input>(), 48);
    assert_eq!(std::mem::size_of::<Output>(), 48);
    let a = flix_create();
    let b = flix_create();
    assert_ne!(a, b);
    assert_ne!(a, 0);
    let input = Input {
        version: 1,
        calibrated: 1,
        now_us: 1000,
        sample_us: 1000,
        gyro: [0.; 3],
        acceleration: [0., 0., 9.80665],
    };
    let mut out = Output {
        version: 0,
        state: 0,
        reason: 0,
        reserved: 0,
        now_us: 0,
        generation: 0,
        motors: [1.; 4],
    };
    // SAFETY: locals are live, aligned and distinct for each call.
    unsafe {
        assert_eq!(flix_tick(a, &input, &mut out), 0);
        assert_eq!(out.motors, [0.; 4]);
        assert_eq!(flix_tick(b, &input, &mut out), 0);
        assert_eq!(flix_tick(a, std::ptr::null(), &mut out), -3);
    }
    assert_eq!(flix_reset(a), 0);
    let mut zero = input;
    zero.now_us = 0;
    zero.sample_us = 0;
    // SAFETY: locals are live, aligned and distinct for each call.
    unsafe {
        assert_eq!(flix_tick(a, &zero, &mut out), 0);
        assert_eq!(out.state, 1);
    }
    assert_eq!(flix_destroy(a), 0);
    assert_eq!(flix_destroy(a), -2);
    // SAFETY: locals are live, aligned and distinct for each call.
    unsafe {
        assert_eq!(flix_tick(a, &input, &mut out), -2);
        assert_eq!(out.motors, [0.; 4]);
    }
    assert_eq!(flix_destroy(b), 0);
}

#[test]
fn observation_is_read_only_and_checks_caller_contract() {
    let handle = flix_create();
    let mut observation = Observation {
        version: 0,
        source: 0,
        target_attitude_valid: 0,
        reserved: 0,
        estimate: [0.; 4],
        rates: [0.; 3],
        target_rates: [0.; 3],
        target_attitude: [0.; 4],
    };
    assert_eq!(std::mem::size_of::<Observation>(), 72);
    // SAFETY: valid aligned writable extent, no pointers retained.
    unsafe {
        assert_eq!(flix_observe(handle, &mut observation), 0);
        assert_eq!(observation.version, 1);
        assert_eq!(observation.source, u32::MAX);
        assert_eq!(observation.estimate, [1., 0., 0., 0.]);
        assert_eq!(flix_observe(handle, &mut observation), 0);
        assert_eq!(flix_observe(handle, std::ptr::null_mut()), -3);
        assert_eq!(flix_destroy(handle), 0);
        assert_eq!(flix_observe(handle, &mut observation), -2);
    }
}
