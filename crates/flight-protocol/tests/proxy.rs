use flight_protocol::{
    proxy::{Framer, admit},
    wire::{self, Policy, Signing},
};
#[test]
fn partial_frames_expire_and_unsigned_or_enrollment_is_never_relayed() {
    let mut bytes = [0; 280];
    let length = wire::encode(0, &[0; 9], 0, 255, 190, &mut bytes).unwrap();
    let mut framer = Framer::default();
    assert!(framer.push(253, 0).is_none());
    assert!(framer.push(9, 1).is_none());
    let mut frame = None;
    for b in &bytes[..length] {
        frame = framer.push(*b, 100_002).or(frame);
    }
    let frame = frame.unwrap();
    let mut policy = Policy::Signed {
        system: 255,
        component: 190,
        signing: Signing::new([7; 32], 100).unwrap(),
    };
    assert!(admit(&frame, &mut policy, 0).is_err());
    let mut p = [0; 42];
    p[..8].copy_from_slice(&100u64.to_le_bytes());
    p[8] = 1;
    p[9] = 1;
    p[10..].fill(7);
    let length = wire::encode(256, &p, 1, 255, 190, &mut bytes).unwrap();
    let length = Signing::new([7; 32], 100)
        .unwrap()
        .sign(&mut bytes, length, 0, 0)
        .unwrap();
    let mut frame = None;
    for b in &bytes[..length] {
        frame = framer.push(*b, 100_003).or(frame);
    }
    assert!(admit(&frame.unwrap(), &mut policy, 0).is_err());
}
