use flight_protocol::{
    Error, event,
    wire::{self, Policy},
};
#[test]
fn valid_crc_arbitrary_payloads_exercise_all_ingress_decoders() {
    let definitions = [
        (20, 20),
        (21, 2),
        (23, 23),
        (69, 30),
        (76, 33),
        (82, 51),
        (119, 12),
        (126, 81),
    ];
    let mut seed = 0xdec0deu64;
    let mut policy = Policy::BenchUsb {
        system: 255,
        component: 190,
    };
    for n in 0..20_000 {
        let (id, max) = definitions[n % definitions.len()];
        let length = n % (max + 1);
        let mut payload = [0; 81];
        for byte in &mut payload[..length] {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            *byte = (seed >> 32) as u8;
        }
        let mut frame = [0; 280];
        let length = wire::encode(id, &payload[..length], 0, 255, 190, &mut frame).unwrap();
        let packet = wire::decode(&frame[..length], &mut policy).unwrap();
        let _ = event(&packet, 1);
    }
}
#[test]
fn advancing_signature_floor_rejects_unseen_old_packets() {
    use sha2::{Digest, Sha256};
    let key = [1; 32];
    let mut bytes = [0; 280];
    let len = wire::encode(0, &[0; 9], 0, 255, 190, &mut bytes).unwrap();
    bytes[2] = 1;
    let crc = wire::crc(&bytes[1..len - 2], 50);
    bytes[len - 2..len].copy_from_slice(&crc.to_le_bytes());
    bytes[len] = 0;
    bytes[len + 1..len + 7].copy_from_slice(&1000u64.to_le_bytes()[..6]);
    let mut digest = Sha256::new();
    digest.update(key);
    digest.update(&bytes[..len + 7]);
    bytes[len + 7..len + 13].copy_from_slice(&digest.finalize()[..6]);
    let mut signing = wire::Signing::new(key, 1000).unwrap();
    signing.advance(70_000_000);
    let mut policy = Policy::Signed {
        system: 255,
        component: 190,
        signing,
    };
    assert_eq!(
        wire::decode(&bytes[..len + 13], &mut policy).unwrap_err(),
        Error::Replay
    );
}
