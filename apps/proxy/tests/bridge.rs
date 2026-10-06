use flight_protocol::wire::{self, Signing};
use flight_proxy::Bridge;
const PEER: [u8; 6] = [2, 1, 2, 3, 4, 5];
fn frame(id: u32, payload: &[u8], system: u8, component: u8) -> Vec<u8> {
    let mut bytes = [0; 280];
    let n = wire::encode(id, payload, 0, system, component, &mut bytes).unwrap();
    let n = Signing::new([7; 32], 100)
        .unwrap()
        .sign(&mut bytes, n, 0, 0)
        .unwrap();
    bytes[..n].to_vec()
}
#[test]
fn full_queue_preserves_stop_and_drops_older_arm_work() {
    let mut b = Bridge::new(PEER).unwrap();
    let heartbeat = frame(0, &[0; 9], 255, 190);
    for _ in 0..4 {
        b.serial(&heartbeat, 1).unwrap();
    }
    assert!(b.serial(&heartbeat, 2).is_err());
    let mut payload = [0; 33];
    payload[28..30].copy_from_slice(&400u16.to_le_bytes());
    payload[30] = 1;
    payload[31] = 1;
    let stop = frame(76, &payload, 255, 190);
    b.serial(&stop, 3).unwrap();
    assert_eq!(&b.radio(4).unwrap().bytes[..stop.len()], stop);
    assert!(b.radio(4).is_none());
}
#[test]
fn partial_timeout_disconnect_expiry_and_wrong_peer_are_bounded() {
    let mut b = Bridge::new(PEER).unwrap();
    let f = frame(0, &[0; 9], 255, 190);
    b.serial(&f[..5], 0).unwrap();
    b.serial(&f, 100001).unwrap();
    assert!(b.radio(100002).is_some());
    b.serial(&f, 200000).unwrap();
    assert!(b.radio(300001).is_none());
    b.serial(&f, 400000).unwrap();
    b.reset();
    assert!(b.radio(400001).is_none());
    let reply = frame(0, &[0; 9], 1, 1);
    assert!(b.received([2; 6], &reply, 1).is_err());
    b.received(PEER, &reply, 2).unwrap();
    assert_eq!(&b.usb(3).unwrap().bytes[..reply.len()], reply);
}
#[test]
fn reject_unsigned_enrollment_oversize_and_malformed_envelopes() {
    let mut b = Bridge::new(PEER).unwrap();
    let mut bytes = [0; 280];
    let n = wire::encode(0, &[0; 9], 0, 255, 190, &mut bytes).unwrap();
    assert!(b.serial(&bytes[..n], 0).is_err());
    let enrollment = frame(256, &[0; 42], 255, 190);
    assert!(b.serial(&enrollment, 1).is_err());
    let mut oversized = vec![0; 280];
    oversized[..3].copy_from_slice(&[253, 255, 1]);
    assert!(b.serial(&oversized, 2).is_err());
    let mut corrupt = frame(0, &[0; 9], 255, 190);
    corrupt[10] ^= 1;
    assert!(b.serial(&corrupt, 3).is_err());
    assert!(b.radio(4).is_none());
    assert!(Bridge::new([1; 6]).is_err());
}
