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

fn input2(now: u64, saturation: u32) -> Input2 {
    Input2 {
        version: 2,
        calibrated: 1,
        now_us: now,
        sample_us: now,
        gyro: [0.; 3],
        acceleration: [0., 0., 9.80665],
        saturation,
        reserved: 0,
    }
}

fn stopped_output() -> Output {
    Output {
        version: 0,
        state: 0,
        reason: 0,
        reserved: 0,
        now_us: 0,
        generation: 0,
        motors: [1.; 4],
    }
}

#[test]
fn version_two_layout_and_validation() {
    assert_eq!(std::mem::size_of::<Input2>(), 56);
    assert_eq!(std::mem::offset_of!(Input2, saturation), 48);
    let h = flix_create();
    let mut out = stopped_output();
    // SAFETY: locals are live, aligned and distinct for each call.
    unsafe {
        assert_eq!(flix_tick2(h, &input2(1000, 0b11_1111), &mut out), 0);
        // Undefined mask bits, nonzero reserved or a v1 version are rejected, stopped.
        for bad in [
            input2(2000, 1 << 6),
            Input2 {
                reserved: 1,
                ..input2(2000, 0)
            },
            Input2 {
                version: 1,
                ..input2(2000, 0)
            },
        ] {
            out.motors = [1.; 4];
            assert_eq!(flix_tick2(h, &bad, &mut out), -4);
            assert_eq!(out.motors, [0.; 4]);
        }
        assert_eq!(flix_tick2(h, std::ptr::null(), &mut out), -3);
    }
    assert_eq!(flix_destroy(h), 0);
}

#[test]
fn version_two_saturation_reaches_the_owner_warning() {
    let h = flix_create();
    let mut out = stopped_output();
    let mut text = Vec::new();
    for ms in 0..20u64 {
        let mask = if ms == 10 { 0b100_000 } else { 0 };
        // SAFETY: locals are live, aligned and distinct for each call.
        unsafe {
            assert_eq!(flix_tick2(h, &input2(ms * 1000, mask), &mut out), 0);
        }
        let mut reply = [0u8; 280];
        loop {
            // SAFETY: reply is a live 280-byte buffer.
            let n = unsafe { flix_reply(h, reply.as_mut_ptr(), 280) };
            if n <= 0 {
                break;
            }
            let frame = &reply[..n as usize];
            if u32::from_le_bytes([frame[7], frame[8], frame[9], 0]) == 253 {
                text.push(frame[11..frame.len() - 2].to_vec());
            }
        }
    }
    assert_eq!(text.len(), 1);
    assert!(text[0].starts_with(b"IMU saturated: az"));
    assert_eq!(flix_destroy(h), 0);
}
