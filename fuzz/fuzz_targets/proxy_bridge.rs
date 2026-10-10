//! The USB/ESP-NOW proxy with untrusted bytes from both sides. Output frames must fit
//! one ESP-NOW payload and the bridge must never panic or grow.
#![no_main]
use arbitrary::Arbitrary;
use flight_proxy::Bridge;
use libfuzzer_sys::fuzz_target;

const PEER: [u8; 6] = [0x02, 1, 2, 3, 4, 5];

#[derive(Arbitrary, Debug)]
enum Op {
    Serial(Vec<u8>),
    Received([u8; 6], Vec<u8>),
    Radio,
    Usb,
    Reset,
    Wait(u16),
}

fuzz_target!(|ops: Vec<Op>| {
    let mut bridge = Bridge::new(PEER).expect("valid peer");
    let mut now = 0u64;
    for op in ops {
        match op {
            Op::Serial(bytes) => {
                let _ = bridge.serial(&bytes, now);
            }
            Op::Received(peer, bytes) => {
                let _ = bridge.received(peer, &bytes, now);
            }
            Op::Radio => {
                if let Some(frame) = bridge.radio(now) {
                    assert!(frame.length <= 250, "radio frame {} bytes", frame.length);
                }
            }
            Op::Usb => {
                if let Some(frame) = bridge.usb(now) {
                    assert!(frame.length <= 250, "usb frame {} bytes", frame.length);
                }
            }
            Op::Reset => bridge.reset(),
            Op::Wait(us) => now = now.saturating_add(u64::from(us) * 16),
        }
    }
});
