use flight_core::{Sensor, Tick, Vec3};
use flight_esp32::Owner;
use flight_esp32::metrics::Histogram;
use flight_protocol::{Console, Event};
fn sample(at: u64) -> Sensor {
    Sensor {
        at: Tick(at),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}
fn chunks(owner: &mut Owner) -> Vec<Vec<u8>> {
    let mut result = Vec::new();
    while let Some(reply) = owner.reply() {
        let mut policy = flight_protocol::wire::Policy::BenchUsb {
            system: 1,
            component: 1,
        };
        let packet = flight_protocol::wire::decode(&reply.bytes[..reply.len], &mut policy).unwrap();
        if packet.id == 126 {
            let data = &packet.payload[9..9 + usize::from(packet.payload[8])];
            if data.starts_with(b"FLXM") {
                result.push(data.to_vec());
            }
        }
    }
    result
}
#[test]
fn frozen_metrics_require_exclusive_maintenance_and_stop_cancels_transfer() {
    assert_eq!(
        flight_protocol::console(b"metrics 1234abcd"),
        Ok(Console::Metrics(0x1234abcd))
    );
    assert!(flight_protocol::console(b"metrics 1234abcd extra").is_err());
    let mut owner = Owner::default();
    owner.tick(Tick(0), Some(sample(0)));
    owner.submit(Event::Console(Console::Metrics(7)), Tick(1000));
    owner.tick(Tick(1000), Some(sample(1000)));
    assert_eq!(chunks(&mut owner)[0][9], 0);
    owner.flight.maintenance();
    owner.timing.acquisition.record(123);
    owner.submit(Event::Console(Console::Metrics(8)), Tick(2000));
    let mut captured = Vec::new();
    for now in (2000..10000).step_by(1000) {
        assert_eq!(owner.tick(Tick(now), Some(sample(now))), [0.; 4]);
        captured.extend(chunks(&mut owner));
        owner.timing.acquisition.record(999);
    }
    assert_eq!(captured.len(), 8);
    let mut bytes = Vec::new();
    for (i, data) in captured.iter().enumerate() {
        assert_eq!(&data[..4], b"FLXM");
        assert_eq!(&data[4..8], &8u32.to_le_bytes());
        assert_eq!(data[8], i as u8);
        assert_eq!(data[9], 8);
        bytes.extend_from_slice(&data[10..]);
    }
    assert_eq!(bytes.len(), 52 * 8);
    let values: Vec<u64> = bytes
        .chunks_exact(8)
        .map(|b| u64::from_le_bytes(b.try_into().unwrap()))
        .collect();
    assert_eq!(&values[..3], &[1, 1, 123]); // Later measurements cannot change frozen data.
    owner.submit(Event::Console(Console::Metrics(9)), Tick(10000));
    owner.tick(Tick(10000), Some(sample(10000)));
    chunks(&mut owner);
    owner.submit(Event::Stop, Tick(11000));
    owner.tick(Tick(11000), Some(sample(11000)));
    assert!(chunks(&mut owner).is_empty());
    owner.tick(Tick(12000), Some(sample(12000)));
    assert!(chunks(&mut owner).is_empty());
    owner.flight.maintenance();
    assert!(
        owner
            .flight
            .diagnostic([0.1, 0., 0., 0.], Tick(13000), 100000)
    );
    owner.submit(Event::Console(Console::Metrics(10)), Tick(13000));
    owner.tick(Tick(13000), Some(sample(13000)));
    assert_eq!(chunks(&mut owner)[0][9], 0);
}
#[test]
fn histogram_boundaries_are_inclusive_and_saturating() {
    let mut h = Histogram::default();
    for us in [
        0, 50, 51, 100, 101, 250, 251, 500, 501, 1000, 1001, 2000, 2001, 5000, 5001,
    ] {
        h.record(us);
    }
    assert_eq!(h.buckets, [2, 2, 2, 2, 2, 2, 2, 1]);
    assert_eq!(h.count, 15);
    assert_eq!(h.maximum_us, 5001);
    h.buckets[0] = u64::MAX;
    h.count = u64::MAX;
    h.record(1);
    assert_eq!(h.buckets[0], u64::MAX);
    assert_eq!(h.count, u64::MAX);
}
