//! Swarm S2 wire contract: SET_POSITION_TARGET_LOCAL_NED (84) and ATT_POS_MOCAP (138).
use flight_core::control::Intent;
use flight_core::{Source, Vec3};
use flight_protocol::wire::{self, Policy};
use flight_protocol::{Error, Event, POSITION_MASK, POSITION_MASK_WITH_VELOCITY, event};

fn decode(id: u32, payload: &[u8], local: u8) -> Result<Event, Error> {
    let mut bytes = [0; 280];
    let length = wire::encode(id, payload, 1, 255, 190, &mut bytes).unwrap();
    let mut policy = Policy::BenchUsb {
        system: 255,
        component: 190,
    };
    let packet = wire::decode(&bytes[..length], &mut policy)?;
    event(&packet, local)
}
#[allow(clippy::too_many_arguments)]
fn target(p: [f32; 3], v: [f32; 3], yaw: f32, mask: u16, system: u8, frame: u8) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&0u32.to_le_bytes());
    for x in p.iter().chain(v.iter()).chain([0.0f32; 3].iter()) {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b.extend_from_slice(&yaw.to_le_bytes());
    b.extend_from_slice(&0f32.to_le_bytes());
    b.extend_from_slice(&mask.to_le_bytes());
    b.extend_from_slice(&[system, 1, frame]);
    assert_eq!(b.len(), 53);
    b
}
fn mocap(p: [f32; 3]) -> Vec<u8> {
    mocap_q([1.0, 0.0, 0.0, 0.0], p)
}
fn mocap_q(q: [f32; 4], p: [f32; 3]) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&123u64.to_le_bytes());
    for x in q.iter().chain(p.iter()) {
        b.extend_from_slice(&x.to_le_bytes());
    }
    b
}

#[test]
fn position_target_converts_ned_to_nwu() {
    let payload = target(
        [1.0, 2.0, -0.5],
        [0.1, 0.2, -0.3],
        0.5,
        POSITION_MASK_WITH_VELOCITY,
        7,
        1,
    );
    let Ok(Event::Setpoint(Source::Automatic, Intent::Position(t))) = decode(84, &payload, 7)
    else {
        panic!("not a position setpoint");
    };
    assert_eq!(t.position, Vec3(1.0, -2.0, 0.5));
    assert_eq!(t.velocity, Vec3(0.1, -0.2, 0.3));
    assert_eq!(t.yaw, -0.5);
    // Velocity bits ignored: the feedforward is zero whatever the fields hold.
    let payload = target([1.0, 2.0, -0.5], [9.0, 9.0, 9.0], 0.0, POSITION_MASK, 7, 1);
    let Ok(Event::Setpoint(_, Intent::Position(t))) = decode(84, &payload, 7) else {
        panic!("not a position setpoint");
    };
    assert_eq!(t.velocity, Vec3::ZERO);
}

#[test]
fn position_target_rejects_other_drones_frames_masks_and_bad_numbers() {
    let ok = |mask, system, frame| target([0.0, 0.0, -1.0], [0.0; 3], 0.0, mask, system, frame);
    assert_eq!(
        decode(84, &ok(POSITION_MASK, 8, 1), 7).unwrap_err(),
        Error::Destination
    );
    for frame in [0, 7, 8, 9] {
        assert_eq!(
            decode(84, &ok(POSITION_MASK, 7, frame), 7).unwrap_err(),
            Error::Unsupported
        );
    }
    // Acceleration used, force, yaw ignored, yaw rate used, partial velocity, position ignored.
    for mask in [0x0800, 0x0BC0, 0x0DC0, 0x01C0, 0x09C8, 0x09C1] {
        assert_eq!(
            decode(84, &ok(mask, 7, 1), 7).unwrap_err(),
            Error::Unsupported,
            "{mask:#x}"
        );
    }
    let nan = target([f32::NAN, 0.0, -1.0], [0.0; 3], 0.0, POSITION_MASK, 7, 1);
    assert_eq!(decode(84, &nan, 7).unwrap_err(), Error::Numeric);
    let fast = target(
        [0.0, 0.0, -1.0],
        [20.0, 0.0, 0.0],
        0.0,
        POSITION_MASK_WITH_VELOCITY,
        7,
        1,
    );
    assert_eq!(decode(84, &fast, 7).unwrap_err(), Error::Numeric);
}

#[test]
fn mocap_converts_ned_to_nwu_and_rejects_non_finite() {
    let Ok(Event::Position { position: p, yaw }) = decode(138, &mocap([1.0, 2.0, -0.5]), 7) else {
        panic!("not a position");
    };
    assert_eq!(p, Vec3(1.0, -2.0, 0.5));
    assert_eq!(yaw, Some(0.0));
    // MAVLink 2 truncates trailing zeros: a sample at the origin is still a sample.
    let Ok(Event::Position { position: p, .. }) = decode(138, &mocap([0.0, 0.0, 0.0]), 7) else {
        panic!("not a position");
    };
    assert_eq!(p, Vec3::ZERO);
    assert_eq!(
        decode(138, &mocap([0.0, f32::INFINITY, 0.0]), 7).unwrap_err(),
        Error::Numeric
    );
}

#[test]
fn mocap_heading_converts_ned_to_nwu_and_zero_quaternion_means_none() {
    // NED yaw +0.5 rad (clockwise seen from above) is NWU yaw -0.5 rad.
    let half = 0.25f32;
    let q = [half.cos(), 0.0, 0.0, half.sin()];
    let Ok(Event::Position { yaw: Some(yaw), .. }) = decode(138, &mocap_q(q, [0.0, 0.0, -1.0]), 7)
    else {
        panic!("no heading");
    };
    assert!((yaw + 0.5).abs() < 1e-6, "{yaw}");
    for q in [[0.0; 4], [f32::NAN, 0.0, 0.0, 0.0], [2.0, 0.0, 0.0, 0.0]] {
        let Ok(Event::Position { yaw, .. }) = decode(138, &mocap_q(q, [0.0, 0.0, -1.0]), 7) else {
            panic!("position must still be used");
        };
        assert_eq!(yaw, None, "{q:?}");
    }
}
