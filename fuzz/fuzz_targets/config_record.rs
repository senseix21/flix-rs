//! Untrusted persisted settings records: decode, dual-slot recovery and re-encoding.
//! Raw input exercises framing/checksum rejection; a repaired copy with a valid
//! header and checksum reaches value validation, which random bytes almost never do.
#![no_main]
use flight_config::{RECORD_SIZE, checksum, decode, encode, recover};
use libfuzzer_sys::fuzz_target;

fn check(record: &[u8]) {
    if let Ok((settings, sequence)) = decode(record) {
        // An accepted record re-encodes to a canonical record that decodes identically.
        let first = encode(&settings, sequence).expect("decoded settings re-encode");
        let (again, again_sequence) = decode(&first).expect("canonical record decodes");
        assert_eq!(again_sequence, sequence);
        assert_eq!(encode(&again, again_sequence).expect("re-encode"), first);
    }
}

fuzz_target!(|data: &[u8]| {
    check(data);
    let (a, b) = data.split_at(data.len().min(RECORD_SIZE));
    if let Ok((_, _, slot)) = recover(a, b) {
        assert!(slot < 2);
    }

    let mut repaired = [0u8; RECORD_SIZE];
    let n = data.len().min(RECORD_SIZE);
    repaired[..n].copy_from_slice(&data[..n]);
    repaired[..8].copy_from_slice(b"FLXC\x01\x00\x60\x00");
    let sum = checksum(&repaired[..RECORD_SIZE - 4]);
    repaired[RECORD_SIZE - 4..].copy_from_slice(&sum.to_le_bytes());
    check(&repaired);
});
