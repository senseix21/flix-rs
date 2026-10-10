//! End-to-end owner: untrusted USB MAVLink, radio enrollment, SBUS and sensor timing
//! reach the flight owner exactly as `flix_board_usb`/`flix_board_tick` deliver them.
//! Every committed motor output must be finite, in [0, 1] and zero unless armed or
//! in a maintenance diagnostic.
#![no_main]
use arbitrary::Arbitrary;
use flight_core::control::GRAVITY;
use flight_core::{Sensor, State, Tick, Vec3};
use flight_esp32::{Owner, session::RadioSession};
use flight_protocol::wire::{self, Decoder, Policy};
use libfuzzer_sys::fuzz_target;

const IDS: [u32; 22] = [
    0, 20, 21, 22, 23, 26, 30, 31, 35, 65, 69, 76, 77, 82, 119, 120, 126, 140, 147, 148, 256, 300,
];

#[derive(Arbitrary, Debug)]
enum Op {
    /// Raw serial bytes, as received over USB.
    Usb(Vec<u8>),
    /// A well-formed frame from the bench client (255/190) with an arbitrary payload.
    Message(u8, Vec<u8>),
    Radio(Vec<u8>),
    Rc(Vec<u8>),
    Tick {
        step: u16,
        stale: bool,
        gyro: [i8; 3],
    },
}

struct Harness {
    owner: Owner,
    radio: RadioSession,
    decoder: Decoder,
    usb: Policy,
    now: u64,
}

impl Harness {
    fn usb(&mut self, bytes: &[u8]) {
        let mut completed = 0;
        for byte in bytes {
            let Some(packet) = self.decoder.push(*byte, &mut self.usb) else {
                continue;
            };
            completed += 1;
            if let Ok(packet) = packet {
                if packet.id == 256 {
                    let _ = self.radio.enroll(&packet, &mut self.owner, Tick(self.now));
                } else if let Ok(event) = flight_protocol::event(&packet, 1) {
                    self.owner.submit(event, Tick(self.now));
                }
            }
            if completed >= 4 {
                self.decoder.reset();
                break;
            }
        }
    }

    fn tick(&mut self, step: u16, stale: bool, gyro: [i8; 3]) {
        self.now = self.now.saturating_add(u64::from(step));
        let at = if stale {
            self.now.saturating_sub(5_000)
        } else {
            self.now
        };
        let rates = Vec3(
            f32::from(gyro[0]) * 0.01,
            f32::from(gyro[1]) * 0.01,
            f32::from(gyro[2]) * 0.01,
        );
        let sensor = Sensor {
            at: Tick(at),
            gyro: rates,
            acceleration: Vec3(0.0, 0.0, GRAVITY),
            calibrated: true,
        };
        let out = self.owner.tick(Tick(self.now), Some(sensor));
        let state = self.owner.flight.snapshot().state;
        assert!(
            out.iter().all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
            "{state:?} output {out:?}"
        );
        if !matches!(state, State::Armed | State::Maintenance) {
            assert_eq!(out, [0.0; 4], "{state:?} produced output");
        }
        // Replies are bounded; drain them as the flight task does.
        for _ in 0..4 {
            if self.owner.reply().is_none() {
                break;
            }
        }
    }
}

fuzz_target!(|ops: Vec<Op>| {
    let mut h = Harness {
        owner: Owner::default(),
        radio: RadioSession::EMPTY,
        decoder: Decoder::EMPTY,
        usb: Policy::BenchUsb {
            system: 255,
            component: 190,
        },
        now: 0,
    };
    for op in ops.into_iter().take(512) {
        match op {
            Op::Usb(bytes) => h.usb(&bytes),
            Op::Message(id, payload) => {
                let mut frame = [0u8; wire::MAX_FRAME];
                let id = IDS[usize::from(id) % IDS.len()];
                if let Ok(length) = wire::encode(id, &payload, 0, 255, 190, &mut frame) {
                    h.usb(&frame[..length]);
                }
            }
            Op::Radio(bytes) => {
                let _ = h.radio.receive(&bytes, &mut h.owner, Tick(h.now));
            }
            Op::Rc(bytes) => {
                h.owner.receive_rc(&bytes, Tick(h.now));
            }
            Op::Tick { step, stale, gyro } => h.tick(step, stale, gyro),
        }
    }
});
