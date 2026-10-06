use flight_core::{Sensor, Tick, Vec3};
use flight_esp32::Owner;
use flight_protocol::wire::{self, Policy};
#[test]
fn capabilities_keep_mavlink2_and_heartbeat_reports_selected_acro() {
    let mut owner = Owner::default();
    let mut policy = Policy::BenchUsb {
        system: 255,
        component: 190,
    };
    let mut command = [0; 33];
    command[..4].copy_from_slice(&148_f32.to_le_bytes());
    command[28..30].copy_from_slice(&512_u16.to_le_bytes());
    command[30] = 1;
    command[31] = 1;
    let mut bytes = [0; 280];
    let n = wire::encode(76, &command, 0, 255, 190, &mut bytes).unwrap();
    assert!(owner.receive(&bytes[..n], &mut policy, Tick(0)));
    owner.tick(
        Tick(0),
        Some(Sensor {
            at: Tick(0),
            gyro: Vec3::ZERO,
            acceleration: Vec3(0., 0., 9.80665),
            calibrated: true,
        }),
    );
    let mut observer = Policy::BenchUsb {
        system: 1,
        component: 1,
    };
    let mut version = false;
    while let Some(reply) = owner.reply() {
        let packet = wire::decode(&reply.bytes[..reply.len], &mut observer).unwrap();
        if packet.id == 148 {
            let capabilities = u64::from_le_bytes(packet.payload[..8].try_into().unwrap());
            assert_eq!(capabilities, 2 | 64 | 8192 | 131072);
            version = true;
        }
    }
    assert!(version);
    let snapshot = owner.flight.snapshot();
    assert_eq!(
        flight_protocol::telemetry::heartbeat(snapshot, true)[..4],
        1_u32.to_le_bytes()
    );
    assert_eq!(
        flight_protocol::telemetry::heartbeat(snapshot, false)[..4],
        2_u32.to_le_bytes()
    );
}

#[test]
fn protocol_negotiation_rejects_v1_capability_and_reports_v2_only() {
    let payload = flight_protocol::telemetry::protocol_version();
    for offset in [0, 2, 4] {
        assert_eq!(
            u16::from_le_bytes(payload[offset..offset + 2].try_into().unwrap()),
            200
        );
    }
    let mut owner = Owner::default();
    let mut policy = Policy::BenchUsb {
        system: 255,
        component: 190,
    };
    let mut command = [0; 33];
    command[..4].copy_from_slice(&300_f32.to_le_bytes());
    command[28..30].copy_from_slice(&512_u16.to_le_bytes());
    command[30] = 1;
    command[31] = 1;
    let mut bytes = [0; 280];
    let n = wire::encode(76, &command, 0, 255, 190, &mut bytes).unwrap();
    assert!(owner.receive(&bytes[..n], &mut policy, Tick(0)));
    owner.tick(
        Tick(0),
        Some(Sensor {
            at: Tick(0),
            gyro: Vec3::ZERO,
            acceleration: Vec3(0., 0., 9.80665),
            calibrated: true,
        }),
    );
    let mut observer = Policy::BenchUsb {
        system: 1,
        component: 1,
    };
    let mut found = false;
    while let Some(reply) = owner.reply() {
        let packet = wire::decode(&reply.bytes[..reply.len], &mut observer).unwrap();
        if packet.id == 300 {
            assert_eq!(&packet.payload[..22], &payload);
            found = true;
        }
    }
    assert!(found);
}
