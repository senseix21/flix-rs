//! Untrusted SBUS receiver frames: decoding, calibration scaling and owner ingress.
#![no_main]
use flight_core::Tick;
use flight_core::rc::{Calibration, Sbus, pilot};
use flight_esp32::Owner;
use libfuzzer_sys::fuzz_target;

const CALIBRATION: Calibration = Calibration {
    low: 172,
    center: 992,
    high: 1811,
};

fuzz_target!(|data: &[u8]| {
    if let Some(frame) = Sbus::decode(data) {
        for mapping in [[0, 1, 2, 3], [3, 2, 1, 0], [15, 15, 15, 15], [16, 0, 0, 0]] {
            if let Some(p) = pilot(frame, mapping, [CALIBRATION; 4]) {
                for v in [p.roll, p.pitch, p.yaw, p.thrust] {
                    assert!(v.is_finite(), "non-finite pilot value {p:?}");
                }
            }
        }
    }
    let mut owner = Owner::default();
    for (i, chunk) in data.chunks(25).enumerate() {
        owner.receive_rc(chunk, Tick(i as u64 * 7_000));
    }
});
