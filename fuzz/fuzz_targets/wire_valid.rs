//! Well-formed MAVLink frames with arbitrary payloads, so fuzzing reaches message
//! handling instead of stopping at the checksum. Optionally signed with the session key.
#![no_main]
use arbitrary::Arbitrary;
use flight_protocol::wire::{self, Policy, Signing};
use libfuzzer_sys::fuzz_target;

/// Every message ID in the frozen dialect plus one unsupported ID.
const IDS: [u32; 23] = [
    0, 20, 21, 22, 23, 26, 30, 31, 35, 65, 69, 76, 77, 82, 119, 120, 126, 140, 147, 148, 256, 300,
    9999,
];
const KEY: [u8; 32] = [7; 32];

#[derive(Arbitrary, Debug)]
struct Input {
    id: u8,
    payload: Vec<u8>,
    sequence: u8,
    system: u8,
    component: u8,
    signed: bool,
    now_us: u64,
}

fuzz_target!(|input: Input| {
    let id = IDS[usize::from(input.id) % IDS.len()];
    let mut frame = [0u8; wire::MAX_FRAME];
    let Ok(length) = wire::encode(
        id,
        &input.payload,
        input.sequence,
        input.system,
        input.component,
        &mut frame,
    ) else {
        return;
    };
    let mut policy = if input.signed {
        let mut signing = Signing::new(KEY, 1).expect("valid test key");
        let Ok(length) = signing.sign(&mut frame, length, input.now_us, 1) else {
            return;
        };
        let mut policy = Policy::Signed {
            system: input.system,
            component: input.component,
            signing: Signing::new(KEY, 1).expect("valid test key"),
        };
        if let Policy::Signed { signing, .. } = &mut policy {
            signing.advance(input.now_us);
        }
        let first = wire::decode(&frame[..length], &mut policy);
        // A replayed signed frame must never be accepted twice.
        if first.is_ok() {
            assert!(wire::decode(&frame[..length], &mut policy).is_err());
        }
        if let Ok(packet) = first {
            let _ = flight_protocol::event(&packet, 1);
        }
        return;
    } else {
        Policy::BenchUsb {
            system: input.system,
            component: input.component,
        }
    };
    if let Ok(packet) = wire::decode(&frame[..length], &mut policy) {
        assert_eq!(packet.id, id);
        assert_eq!(usize::from(packet.length), input.payload.len());
        let _ = flight_protocol::event(&packet, 1);
    }
});
