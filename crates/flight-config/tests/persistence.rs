use flight_config::*;
#[derive(Clone)]
struct Memory {
    slots: [[u8; RECORD_SIZE]; 2],
    cut: usize,
}
impl Storage for Memory {
    fn read(&mut self, slot: usize, out: &mut [u8; RECORD_SIZE]) -> Result<(), Error> {
        *out = self.slots[slot];
        Ok(())
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
    assert_eq!(PARAMETERS.len(), 96);
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
        fn read(&mut self, _: usize, out: &mut [u8; RECORD_SIZE]) -> Result<(), Error> {
            if self.fail {
                return Err(Error::Storage);
            }
            if self.missing {
                return Err(Error::Missing);
            }
            *out = self.bytes;
            Ok(())
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
