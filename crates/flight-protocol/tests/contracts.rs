use flight_protocol::wire::{self, Policy, Signing};
use flight_protocol::{Error, Event, event};
fn bench() -> Policy {
    Policy::BenchUsb {
        system: 255,
        component: 190,
    }
}
fn frame(id: u32, payload: &[u8]) -> Vec<u8> {
    let mut bytes = [0; 280];
    let len = wire::encode(id, payload, 1, 255, 190, &mut bytes).unwrap();
    bytes[..len].to_vec()
}
fn signed(mut bytes: Vec<u8>, key: [u8; 32], timestamp: u64) -> Vec<u8> {
    use sha2::{Digest, Sha256};
    bytes[2] = 1;
    let id = u32::from(bytes[7]);
    let extra = match id {
        0 => 50,
        76 => 152,
        _ => panic!(),
    };
    let end = bytes.len() - 2;
    let c = wire::crc(&bytes[1..end], extra);
    bytes[end..].copy_from_slice(&c.to_le_bytes());
    bytes.push(0);
    bytes.extend_from_slice(&timestamp.to_le_bytes()[..6]);
    let mut d = Sha256::new();
    d.update(key);
    d.update(&bytes);
    let hash = d.finalize();
    bytes.extend_from_slice(&hash[..6]);
    bytes
}
#[test]
fn crc_signature_replay_and_source_are_enforced() {
    let unsigned = frame(0, &[0; 9]);
    let key = [0x42; 32];
    let valid = signed(unsigned.clone(), key, 1000);
    let mut policy = Policy::Signed {
        system: 255,
        component: 190,
        signing: Signing::new(key, 999).unwrap(),
    };
    assert!(wire::decode(&valid, &mut policy).is_ok());
    assert_eq!(
        wire::decode(&valid, &mut policy).unwrap_err(),
        Error::Replay
    );
    assert_eq!(
        wire::decode(&unsigned, &mut policy).unwrap_err(),
        Error::Unauthorized
    );
    let mut tampered = signed(unsigned, key, 1001);
    let end = tampered.len() - 1;
    tampered[end] ^= 1;
    assert_eq!(
        wire::decode(&tampered, &mut policy).unwrap_err(),
        Error::Unauthorized
    );
    let mut wrong = frame(0, &[0; 9]);
    wrong[5] = 1;
    let c = wire::crc(&wrong[1..19], 50);
    wrong[19..21].copy_from_slice(&c.to_le_bytes());
    assert_eq!(
        wire::decode(&wrong, &mut bench()).unwrap_err(),
        Error::Unauthorized
    );
}
#[test]
fn bad_masks_destinations_counts_and_numbers_rejected() {
    let mut p = [0; 39];
    p[4..8].copy_from_slice(&1f32.to_le_bytes());
    p[36] = 1;
    p[37] = 1;
    let valid = wire::decode(&frame(82, &p), &mut bench()).unwrap();
    assert!(matches!(event(&valid, 1), Ok(Event::Setpoint(_, _))));
    p[38] = 64;
    assert_eq!(
        event(&wire::decode(&frame(82, &p), &mut bench()).unwrap(), 1).unwrap_err(),
        Error::Unsupported
    );
    p[38] = 0;
    p[36] = 2;
    assert_eq!(
        event(&wire::decode(&frame(82, &p), &mut bench()).unwrap(), 1).unwrap_err(),
        Error::Destination
    );
    p[36] = 1;
    p[32..36].copy_from_slice(&f32::NAN.to_le_bytes());
    assert_eq!(
        event(&wire::decode(&frame(82, &p), &mut bench()).unwrap(), 1).unwrap_err(),
        Error::Numeric
    );
    let mut console = [0; 79];
    console[6] = 10;
    console[8] = 71;
    assert_eq!(
        event(
            &wire::decode(&frame(126, &console), &mut bench()).unwrap(),
            1
        )
        .unwrap_err(),
        Error::Bounds
    );
}
#[test]
fn stop_survives_full_queue_and_discards_pending_arm() {
    let mut ingress = flight_protocol::ingress::Ingress::default();
    for _ in 0..8 {
        ingress.push(Event::Arm, flight_core::Tick(0)).unwrap();
    }
    assert_eq!(
        ingress.push(Event::Arm, flight_core::Tick(0)),
        Err(Error::Full)
    );
    ingress.push(Event::Stop, flight_core::Tick(0)).unwrap();
    assert!(ingress.take_stop());
    assert!(ingress.pop().is_none());
    assert!(ingress.take_setpoints().iter().all(Option::is_none));
}
#[test]
fn truncated_zero_tail_and_concatenated_streams_are_safe() {
    let payload = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
    let bytes = frame(69, &payload);
    for length in 0..bytes.len() {
        assert!(wire::decode(&bytes[..length], &mut bench()).is_err());
    }
    let mut decoder = wire::Decoder::default();
    let mut packets = 0;
    for byte in bytes.iter().chain(bytes.iter()) {
        if let Some(result) = decoder.push(*byte, &mut bench()) {
            result.unwrap();
            packets += 1;
        }
    }
    assert_eq!(packets, 2);
    // MAVLink 2 trims zero-valued trailing fields; zero padding is required during decode.
    let short = frame(69, &[]);
    let packet = wire::decode(&short, &mut bench()).unwrap();
    assert!(event(&packet, 1).is_ok());
}
#[test]
fn seeded_packet_mutation_and_arbitrary_stream_corpus_never_panics() {
    let seed_frame = frame(82, &[0; 39]);
    let mut seed = 17u64;
    let mut decoder = wire::Decoder::default();
    let mut policy = bench();
    for _ in 0..30_000 {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let mut bytes = seed_frame.clone();
        let index = (seed as usize) % bytes.len();
        bytes[index] ^= (seed >> 32) as u8;
        if let Ok(p) = wire::decode(&bytes, &mut policy) {
            let _ = event(&p, 1);
        }
        if let Some(Ok(p)) = decoder.push((seed >> 24) as u8, &mut policy) {
            let _ = event(&p, 1);
        }
    }
}
#[test]
fn peers_are_bounded_and_require_unicast_nonzero_keys() {
    use flight_protocol::peers::{Peer, Peers};
    let mut peers = Peers::default();
    for i in 0..4 {
        assert!(
            peers
                .add(Peer {
                    mac: [2, 0, 0, 0, 0, i],
                    key: [1; 16]
                })
                .is_ok()
        );
    }
    assert_eq!(
        peers.add(Peer {
            mac: [2, 0, 0, 0, 0, 8],
            key: [1; 16]
        }),
        Err(Error::Full)
    );
    assert!(peers.remove([2, 0, 0, 0, 0, 1]));
    assert_eq!(
        peers.add(Peer {
            mac: [255; 6],
            key: [0; 16]
        }),
        Err(Error::Unauthorized)
    );
}
#[test]
fn upstream_pyflix_mode_and_console_encodings_are_accepted() {
    // DO_SET_MODE with base mode 0 (upstream pyflix) and 1 (QGC); the mode is param2.
    for base in [0.0f32, 1.0] {
        let mut p = Vec::new();
        for v in [base, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
            p.extend_from_slice(&v.to_le_bytes());
        }
        p.extend_from_slice(&176u16.to_le_bytes());
        p.extend_from_slice(&[1, 1, 0]);
        let mut policy = bench();
        let packet = wire::decode(&frame(76, &p), &mut policy).unwrap();
        assert!(
            matches!(event(&packet, 1), Ok(Event::Mode(3))),
            "base {base}"
        );
    }
    let mut p = Vec::new();
    for v in [2.0f32, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0] {
        p.extend_from_slice(&v.to_le_bytes());
    }
    p.extend_from_slice(&176u16.to_le_bytes());
    p.extend_from_slice(&[1, 1, 0]);
    let mut policy = bench();
    let packet = wire::decode(&frame(76, &p), &mut policy).unwrap();
    assert!(matches!(
        event(&packet, 1),
        Ok(Event::UnsupportedCommand(176))
    ));
    // SERIAL_CONTROL: device 0 (upstream pyflix) or 10 (shell); NUL padding counted in `count`.
    for device in [0u8, 10] {
        let mut data = [0u8; 70];
        data[..7].copy_from_slice(b"status\n");
        let mut p = vec![0u8; 6];
        p.extend_from_slice(&[device, 0, 70]);
        p.extend_from_slice(&data);
        let mut policy = bench();
        let packet = wire::decode(&frame(126, &p), &mut policy).unwrap();
        assert!(
            matches!(
                event(&packet, 1),
                Ok(Event::Console(flight_protocol::Console::Status))
            ),
            "device {device}"
        );
    }
    let mut p = vec![0u8; 6];
    p.extend_from_slice(&[1, 0, 6]);
    p.extend_from_slice(b"status");
    let mut policy = bench();
    let packet = wire::decode(&frame(126, &p), &mut policy).unwrap();
    assert_eq!(event(&packet, 1).unwrap_err(), Error::Bounds);
}
#[test]
fn wired_policy_also_accepts_component_zero_but_radio_stays_exact() {
    // wire::encode refuses a zero sender (the drone never sends as 0), so build an incoming
    // frame by patching the sender bytes and recomputing the checksum (HEARTBEAT CRC extra 50).
    let heartbeat = |system: u8, component: u8| {
        let mut bytes = frame(0, &[0, 0, 0, 0, 6, 8, 0, 4, 3]);
        bytes[5] = system;
        bytes[6] = component;
        let end = bytes.len() - 2;
        let c = wire::crc(&bytes[1..end], 50);
        bytes[end..].copy_from_slice(&c.to_le_bytes());
        bytes
    };
    for (system, component, ok) in [
        (255, 190, true),
        (255, 0, true),
        (255, 1, false),
        (254, 0, false),
        (0, 0, false),
    ] {
        let mut policy = bench();
        assert_eq!(
            wire::decode(&heartbeat(system, component), &mut policy).is_ok(),
            ok,
            "{system}/{component}"
        );
    }
    let key = [7u8; 32];
    let mut policy = Policy::Signed {
        system: 255,
        component: 190,
        signing: Signing::new(key, 1).unwrap(),
    };
    let unsigned_zero = heartbeat(255, 0);
    assert_eq!(
        wire::decode(&signed(unsigned_zero, key, 10), &mut policy).unwrap_err(),
        Error::Unauthorized
    );
}
