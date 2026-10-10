//! HIGHRES_IMU encoding contract for arbitrary samples: finite axes round-trip
//! bit-exactly in FRD (x, -y, -z); non-finite axes are 0 with their valid bit clear;
//! unmeasured fields stay zero.
use flight_core::{Sensor, Tick, Vec3};
use flight_protocol::telemetry::highres_imu;
use proptest::prelude::*;

fn axis() -> impl Strategy<Value = f32> {
    prop_oneof![
        8 => any::<f32>(),
        1 => Just(f32::NAN),
        1 => Just(f32::INFINITY),
        1 => Just(f32::NEG_INFINITY),
        1 => Just(-0.0f32),
    ]
}

proptest! {
    #[test]
    fn encodes_exact_frd_and_valid_bits(t in any::<u64>(), v in prop::array::uniform6(axis())) {
        let p = highres_imu(Sensor {
            at: Tick(t),
            acceleration: Vec3(v[0], v[1], v[2]),
            gyro: Vec3(v[3], v[4], v[5]),
            calibrated: true,
        });
        prop_assert_eq!(u64::from_le_bytes(p[..8].try_into().unwrap()), t);
        let frd = [v[0], -v[1], -v[2], v[3], -v[4], -v[5]];
        let updated = u16::from_le_bytes([p[60], p[61]]);
        for (i, want) in frd.into_iter().enumerate() {
            let got = f32::from_le_bytes(p[8 + i * 4..12 + i * 4].try_into().unwrap());
            if want.is_finite() {
                prop_assert_eq!(got.to_bits(), want.to_bits());
                prop_assert!(updated & (1 << i) != 0);
            } else {
                prop_assert_eq!(got.to_bits(), 0);
                prop_assert!(updated & (1 << i) == 0);
            }
        }
        prop_assert_eq!(updated & !0x3f, 0, "only acc/gyro bits may be set");
        prop_assert!(p[32..60].iter().all(|b| *b == 0), "unmeasured fields must be zero");
        prop_assert_eq!(p[62], 0);
    }
}
