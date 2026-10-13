use flight_config::*;
#[derive(Clone)]
struct Memory {
    slots: [[u8; RECORD_SIZE]; 2],
    cut: usize,
}
impl Storage for Memory {
    fn read(&mut self, slot: usize, out: &mut [u8; RECORD_SIZE]) -> Result<usize, Error> {
        *out = self.slots[slot];
        Ok(RECORD_SIZE)
    }
    fn write(&mut self, slot: usize, bytes: &[u8; RECORD_SIZE]) -> Result<(), Error> {
        self.slots[slot][..self.cut].copy_from_slice(&bytes[..self.cut]);
        if self.cut < RECORD_SIZE {
            Err(Error::Storage)
        } else {
            Ok(())
        }
    }
}
#[test]
fn invalid_values_and_config_aliasing_are_rejected_atomically() {
    let mut s = Settings::default();
    assert!(s.validate().is_ok());
    assert_eq!(s.set("CTL_RATE_R_P", f32::NAN), Err(Error::Bounds));
    assert_eq!(s.set("MAV_SYS_ID", 1.5), Err(Error::Bounds));
    assert_eq!(s.set("SF_DESCEND_TIME", 20.), Err(Error::Unsupported));
    assert_eq!(s.set("RC_MAX_0", 100.), Err(Error::CrossField));
    assert_eq!(s.get("RC_MAX_0"), Some(1811.));
    assert_eq!(
        s.bind_board([4, 4, -1, -1, -1, -1, -1]),
        Err(Error::CrossField)
    );
    assert!(s.bind_board([4, 5, -1, -1, -1, -1, -1]).is_ok());
}
#[test]
fn every_interruption_retains_the_last_committed_record() {
    let old = Settings::default();
    let mut new = old;
    new.set("CTL_RATE_R_P", 0.08).unwrap();
    let baseline = Memory {
        slots: [encode(&old, 7).unwrap(), [0; RECORD_SIZE]],
        cut: RECORD_SIZE,
    };
    let mut flight = flight_core::Flight::default();
    flight.disarm();
    assert!(flight.maintenance());
    for cut in 0..RECORD_SIZE {
        let mut storage = baseline.clone();
        storage.cut = cut;
        assert!(save(&mut storage, &new, &flight).is_err());
        let (recovered, sequence, _) = recover(&storage.slots[0], &storage.slots[1]).unwrap();
        assert_eq!(sequence, 7);
        assert_eq!(recovered.get("CTL_RATE_R_P"), old.get("CTL_RATE_R_P"));
    }
    let mut storage = baseline;
    assert_eq!(save(&mut storage, &new, &flight), Ok(8));
    assert_eq!(
        recover(&storage.slots[0], &storage.slots[1])
            .unwrap()
            .0
            .get("CTL_RATE_R_P"),
        Some(0.08)
    );
}
#[test]
fn flash_requires_stopped_exclusive_maintenance() {
    let settings = Settings::default();
    let mut memory = Memory {
        slots: [[0; RECORD_SIZE]; 2],
        cut: RECORD_SIZE,
    };
    let mut flight = flight_core::Flight::default();
    assert_eq!(save(&mut memory, &settings, &flight), Err(Error::Busy));
    flight.disarm();
    flight.maintenance();
    assert!(flight.diagnostic([0.1; 4], flight_core::Tick(0), 1000));
    assert_eq!(save(&mut memory, &settings, &flight), Err(Error::Busy));
}
#[test]
fn schema_inventory_and_crc_corruption_contract() {
    assert_eq!(PARAMETERS.len(), COUNT);
    assert_eq!(COUNT, 117);
    // Version 2 only appended: the first 96 names are the version 1 schema, in order.
    assert_eq!(PARAMETERS[V1_COUNT - 1].name, "SF_DISARM_TILT");
    assert!(
        PARAMETERS[V1_COUNT..]
            .iter()
            .all(|p| p.name.starts_with("POS_"))
    );
    for (i, p) in PARAMETERS.iter().enumerate() {
        assert!(p.name.len() <= 15);
        assert_eq!(Settings::index(p.name), Some(i));
    }
    let record = encode(&Settings::default(), 1).unwrap();
    for index in 0..record.len() {
        let mut corrupted = record;
        corrupted[index] ^= 1;
        assert!(decode(&corrupted).is_err());
    }
}

#[test]
fn only_absent_storage_can_initialize_defaults() {
    struct Store {
        missing: bool,
        fail: bool,
        bytes: [u8; RECORD_SIZE],
        writes: usize,
    }
    impl Storage for Store {
        fn read(&mut self, _: usize, out: &mut [u8; RECORD_SIZE]) -> Result<usize, Error> {
            if self.fail {
                return Err(Error::Storage);
            }
            if self.missing {
                return Err(Error::Missing);
            }
            *out = self.bytes;
            Ok(RECORD_SIZE)
        }
        fn write(&mut self, _: usize, bytes: &[u8; RECORD_SIZE]) -> Result<(), Error> {
            self.bytes = *bytes;
            self.missing = false;
            self.writes += 1;
            Ok(())
        }
    }
    let mut flight = flight_core::Flight::default();
    flight.disarm();
    assert!(flight.maintenance());
    let mut store = Store {
        missing: true,
        fail: false,
        bytes: [0; RECORD_SIZE],
        writes: 0,
    };
    assert_eq!(save(&mut store, &Settings::default(), &flight), Ok(1));
    store.fail = true;
    assert_eq!(
        save(&mut store, &Settings::default(), &flight),
        Err(Error::Storage)
    );
    assert_eq!(store.writes, 1);
    store.fail = false;
    store.bytes = [0; RECORD_SIZE];
    assert_eq!(
        save(&mut store, &Settings::default(), &flight),
        Err(Error::Storage)
    );
    assert_eq!(store.writes, 1);
}

/// A version 1 record as older firmware wrote it: header [1, 0, 96, 0], 96 values, CRC.
fn v1_record(values: &[f32], sequence: u64) -> Vec<u8> {
    let mut b = vec![0u8; V1_RECORD_SIZE];
    b[..4].copy_from_slice(b"FLXC");
    b[4..8].copy_from_slice(&[1, 0, 96, 0]);
    b[8..16].copy_from_slice(&sequence.to_le_bytes());
    for (i, v) in values.iter().take(V1_COUNT).enumerate() {
        b[16 + i * 4..20 + i * 4].copy_from_slice(&v.to_le_bytes());
    }
    let c = checksum(&b[..V1_RECORD_SIZE - 4]);
    b[V1_RECORD_SIZE - 4..].copy_from_slice(&c.to_le_bytes());
    b
}

#[test]
fn version_1_records_migrate_with_position_defaults() {
    let mut old = Settings::default();
    old.set("MAV_SYS_ID", 12.0).unwrap();
    let values: Vec<f32> = (0..V1_COUNT).map(|i| old.at(i).unwrap()).collect();
    let (migrated, sequence) = decode(&v1_record(&values, 41)).unwrap();
    assert_eq!(sequence, 41);
    assert_eq!(migrated.get("MAV_SYS_ID"), Some(12.0));
    for (i, p) in PARAMETERS.iter().enumerate().skip(V1_COUNT) {
        assert_eq!(migrated.at(i), Some(p.default), "{}", p.name);
    }
    // The migrated settings save as version 2 and round-trip.
    let (again, _) = decode(&encode(&migrated, 42).unwrap()).unwrap();
    assert_eq!(again.get("MAV_SYS_ID"), Some(12.0));
    assert_eq!(&encode(&migrated, 42).unwrap()[4..8], &[2, 0, 117, 0]);
}

#[test]
fn version_1_corruption_and_mixed_headers_are_rejected() {
    let values: Vec<f32> = (0..V1_COUNT)
        .map(|i| Settings::default().at(i).unwrap())
        .collect();
    let good = v1_record(&values, 3);
    let mut corrupt = good.clone();
    corrupt[20] ^= 1;
    assert!(decode(&corrupt).is_err());
    // A version 1 header on a version 2 length, and the reverse, are refused.
    let mut v2 = encode(&Settings::default(), 3).unwrap().to_vec();
    v2[4..8].copy_from_slice(&[1, 0, 96, 0]);
    assert!(decode(&v2).is_err());
    let mut v1 = good.clone();
    v1[4..8].copy_from_slice(&[2, 0, 117, 0]);
    assert!(decode(&v1).is_err());
    assert!(decode(&good[..V1_RECORD_SIZE - 1]).is_err());
}

#[test]
fn save_over_a_version_1_slot_writes_version_2_to_the_other_slot() {
    struct Mixed {
        v1: Vec<u8>,
        slots: [Option<[u8; RECORD_SIZE]>; 2],
    }
    impl Storage for Mixed {
        fn read(&mut self, slot: usize, out: &mut [u8; RECORD_SIZE]) -> Result<usize, Error> {
            match self.slots[slot] {
                Some(bytes) => {
                    *out = bytes;
                    Ok(RECORD_SIZE)
                }
                None if slot == 0 => {
                    out[..self.v1.len()].copy_from_slice(&self.v1);
                    Ok(self.v1.len())
                }
                None => Err(Error::Missing),
            }
        }
        fn write(&mut self, slot: usize, bytes: &[u8; RECORD_SIZE]) -> Result<(), Error> {
            self.slots[slot] = Some(*bytes);
            Ok(())
        }
    }
    let values: Vec<f32> = (0..V1_COUNT)
        .map(|i| Settings::default().at(i).unwrap())
        .collect();
    let mut store = Mixed {
        v1: v1_record(&values, 9),
        slots: [None, None],
    };
    let mut flight = flight_core::Flight::default();
    flight.disarm();
    assert!(flight.maintenance());
    assert_eq!(save(&mut store, &Settings::default(), &flight), Ok(10));
    assert!(
        store.slots[0].is_none(),
        "the version 1 slot is kept until the next save"
    );
    assert_eq!(&store.slots[1].unwrap()[4..8], &[2, 0, 117, 0]);
}

#[test]
fn position_parameters_reach_the_flight_tuning_and_are_validated() {
    let mut s = Settings::default();
    s.set("POS_VOL_Z_MAX", 3.0).unwrap();
    s.set("POS_P_XY", 5.0).unwrap();
    let t = s.tuning().position;
    assert_eq!(t.volume.max.2, 3.0);
    assert_eq!((t.kp.0, t.kp.1), (5.0, 5.0));
    assert_eq!(t.sample_us, 200_000);
    // An empty volume or a hover thrust above the thrust limit is refused as a whole.
    let mut bad = Settings::default();
    assert_eq!(bad.set("POS_VOL_Z_MAX", -1.0), Err(Error::CrossField));
    assert_eq!(bad.set("POS_HOVER_THR", 0.95), Err(Error::CrossField));
    assert_eq!(bad.get("POS_HOVER_THR"), Some(0.5));
}

#[test]
fn position_defaults_match_the_core_constants() {
    let from_parameters = Settings::default().tuning().position;
    let core = flight_core::position::PositionTuning::DEFAULT;
    assert_eq!(from_parameters.volume, core.volume);
    assert_eq!(from_parameters.sample_us, core.sample_us);
    for (a, b) in [
        (from_parameters.alpha, core.alpha),
        (from_parameters.beta, core.beta),
        (from_parameters.heading_weight, core.heading_weight),
        (from_parameters.integral_limit, core.integral_limit),
        (from_parameters.max_tilt, core.max_tilt),
        (from_parameters.hover_thrust, core.hover_thrust),
        (from_parameters.max_thrust, core.max_thrust),
        (from_parameters.max_speed, core.max_speed),
    ] {
        assert!((a - b).abs() < 1e-6, "{a} != {b}");
    }
    assert_eq!(
        [from_parameters.kp, from_parameters.kd, from_parameters.ki],
        [core.kp, core.kd, core.ki]
    );
}
