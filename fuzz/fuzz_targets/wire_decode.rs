//! Untrusted MAVLink bytes: whole-frame decode, streaming decode and console parsing.
#![no_main]
use flight_protocol::wire::{self, Decoder, Policy, Signing};
use libfuzzer_sys::fuzz_target;

const KEY: [u8; 32] = [7; 32];

fn signed() -> Policy {
    Policy::Signed {
        system: 255,
        component: 190,
        signing: Signing::new(KEY, 1).expect("valid test key"),
    }
}

fuzz_target!(|data: &[u8]| {
    let mut bench = Policy::BenchUsb {
        system: 255,
        component: 190,
    };
    if let Ok(packet) = wire::decode(data, &mut bench) {
        assert!(usize::from(packet.length) + 12 <= data.len());
        let _ = flight_protocol::event(&packet, 1);
    }
    let _ = wire::decode(data, &mut signed());
    let _ = flight_protocol::console(data);

    // At most one complete frame per byte; the decoder state must stay bounded.
    for mut policy in [bench, signed()] {
        let mut decoder = Decoder::EMPTY;
        for byte in data {
            if let Some(Ok(packet)) = decoder.push(*byte, &mut policy) {
                let _ = flight_protocol::event(&packet, 1);
            }
        }
    }
});
