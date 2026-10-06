use flight_core::rc::*;
#[test]
fn sbus_calibration_flags_and_channel_bounds() {
    let mut bytes = [0u8; 25];
    bytes[0] = 15;
    let expected: [u16; 16] = core::array::from_fn(|i| 172 + i as u16 * 100);
    for (channel, value) in expected.iter().enumerate() {
        for bit in 0..11 {
            let index = channel * 11 + bit;
            bytes[1 + index / 8] |= (((value >> bit) & 1) as u8) << (index % 8);
        }
    }
    let frame = Sbus::decode(&bytes).unwrap();
    assert_eq!(frame.channels, expected);
    let calibration = Calibration {
        low: 172,
        center: 992,
        high: 1811,
    };
    assert_eq!(calibration.scale(172, false), Some(-1.0));
    assert_eq!(calibration.scale(992, false), Some(0.0));
    assert_eq!(calibration.scale(1811, true), Some(1.0));
    assert!(calibration.scale(0, false).is_none());
    assert!(pilot(frame, [0, 1, 2, 16], [calibration; 4]).is_none());
    bytes[23] = 8;
    assert!(
        pilot(
            Sbus::decode(&bytes).unwrap(),
            [0, 1, 2, 3],
            [calibration; 4]
        )
        .is_none()
    );
    bytes[23] = 4;
    assert!(Sbus::decode(&bytes).unwrap().lost);
    assert!(Sbus::decode(&bytes[..24]).is_none());
}
#[test]
fn ring_order_partial_offsets_and_zero_capacity() {
    use flight_core::log::*;
    let mut log = Log::<3>::default();
    for i in 1..=5 {
        assert!(log.push(Entry {
            micros: i,
            values: [i as f32; 14]
        }));
    }
    assert_eq!(log.len(), 3);
    assert_eq!(log.get(0).unwrap().micros, 3);
    assert_eq!(log.get(2).unwrap().micros, 5);
    assert!(log.get(3).is_none());
    let mut full = [0; 204];
    assert_eq!(log.encode(0, &mut full), 204);
    for offset in 0..204 {
        let mut chunk = [0; 17];
        let len = log.encode(offset, &mut chunk);
        assert_eq!(&chunk[..len], &full[offset..offset + len]);
    }
    assert_eq!(log.encode(204, &mut full), 0);
    assert!(!Log::<0>::default().push(Entry::default()));
}
