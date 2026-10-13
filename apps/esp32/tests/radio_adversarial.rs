//! R2-05: the signed radio path under hostile or broken conditions.
//!
//! Every frame goes through `RadioSession::receive`, the same call the firmware's UDP and
//! ESP-NOW transports make. The firmware learns a reply endpoint only when that call returns
//! true, so each refusal below also means the attacker's address is not learned.
use flight_core::{Reason, Sensor, State, Tick, Vec3};
use flight_esp32::{Owner, session::RadioSession};
use flight_protocol::wire::{self, Packet, Signing};

const STAMP: u64 = 100_000_000; // Trusted enrollment time, MAVLink 10 us units.
const KEY_A: [u8; 32] = [0xa5; 32];
const KEY_B: [u8; 32] = [0x5a; 32];

fn sample(t: u64) -> Sensor {
    Sensor {
        at: Tick(t),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0., 0., 9.80665),
        calibrated: true,
    }
}
fn enrollment(key: [u8; 32], stamp: u64) -> Packet {
    let mut payload = [0; 255];
    payload[..8].copy_from_slice(&stamp.to_le_bytes());
    payload[8] = 1;
    payload[9] = 1;
    payload[10..42].copy_from_slice(&key);
    Packet {
        id: 256,
        system: 255,
        component: 190,
        sequence: 0,
        length: 42,
        payload,
    }
}
fn command(command: u16, first: f32, second: f32) -> (u32, Vec<u8>) {
    let mut p = vec![0; 33];
    p[..4].copy_from_slice(&first.to_le_bytes());
    p[4..8].copy_from_slice(&second.to_le_bytes());
    p[28..30].copy_from_slice(&command.to_le_bytes());
    p[30] = 1;
    p[31] = 1;
    (76, p)
}
fn arm() -> (u32, Vec<u8>) {
    command(400, 1., 0.)
}
fn stop() -> (u32, Vec<u8>) {
    command(400, 0., 0.)
}
fn stab() -> (u32, Vec<u8>) {
    command(176, 1., 2.)
}
fn sticks(thrust: f32) -> (u32, Vec<u8>) {
    let mut p = vec![0; 11];
    p[4..6].copy_from_slice(&((thrust * 1000.) as i16).to_le_bytes());
    p[10] = 1;
    (69, p)
}
fn heartbeat() -> (u32, Vec<u8>) {
    (0, vec![0; 9])
}

/// The operator side: a ground station holding the enrolled key and its own clock.
struct Client {
    signing: Signing,
    sequence: u8,
}
impl Client {
    fn new(key: [u8; 32], stamp: u64, now: u64) -> Self {
        Self {
            signing: Signing::enrolled(key, stamp, now).unwrap(),
            sequence: 0,
        }
    }
    fn sign(&mut self, (id, payload): (u32, Vec<u8>), now: u64, link: u8) -> Vec<u8> {
        let mut out = [0; 280];
        let n = wire::encode(id, &payload, self.sequence, 255, 190, &mut out).unwrap();
        self.sequence = self.sequence.wrapping_add(1);
        let n = self.signing.sign(&mut out, n, now, link).unwrap();
        out[..n].to_vec()
    }
}

/// The drone: owner plus radio session, ticked at 1 kHz by the test.
struct Drone {
    owner: Owner,
    session: RadioSession,
    now: u64,
}
impl Drone {
    /// Boots, enrolls `key` over the trusted wired path, and leaves maintenance.
    fn enrolled(key: [u8; 32]) -> Self {
        let mut drone = Self::booted();
        drone.enroll(key, STAMP).unwrap();
        drone
    }
    fn booted() -> Self {
        let mut owner = Owner::default();
        owner.tick(Tick(1_000), Some(sample(1_000)));
        Self {
            owner,
            session: RadioSession::default(),
            now: 1_000,
        }
    }
    fn enroll(&mut self, key: [u8; 32], stamp: u64) -> Result<(), flight_protocol::Error> {
        // The wired tool asks for maintenance first; armed, that request is refused and the
        // enrollment below meets the session's own stopped-only check.
        let entered = self.owner.flight.maintenance();
        let result = self
            .session
            .enroll(&enrollment(key, stamp), &mut self.owner, Tick(self.now));
        if entered {
            self.owner.flight.disarm();
        }
        result
    }
    fn radio(&mut self, bytes: &[u8]) -> bool {
        self.session.receive(bytes, &mut self.owner, Tick(self.now))
    }
    fn tick(&mut self, us: u64) -> [f32; 4] {
        let mut motors = [0.; 4];
        for _ in 0..us / 1_000 {
            self.now += 1_000;
            motors = self.owner.tick(Tick(self.now), Some(sample(self.now)));
        }
        motors
    }
    fn state(&self) -> State {
        self.owner.flight.snapshot().state
    }
    /// Selects STAB, arms and climbs to 40% thrust, all over the signed radio link.
    fn fly(&mut self, client: &mut Client) {
        for message in [stab(), sticks(0.), arm()] {
            let frame = client.sign(message, self.now, 0);
            assert!(self.radio(&frame));
            self.tick(1_000);
        }
        assert_eq!(self.state(), State::Armed);
        let frame = client.sign(sticks(0.4), self.now, 0);
        assert!(self.radio(&frame));
        assert!(self.tick(1_000).iter().all(|m| *m > 0.));
    }
}

#[test]
fn no_key_wrong_key_and_unsigned_frames_never_admit() {
    // Not enrolled: even a correctly signed frame is refused.
    let mut drone = Drone::booted();
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    assert!(!drone.radio(&client.sign(arm(), drone.now, 0)));

    let mut drone = Drone::enrolled(KEY_A);
    let mut stranger = Client::new(KEY_B, STAMP, drone.now);
    for message in [stab(), sticks(0.), arm(), heartbeat()] {
        assert!(!drone.radio(&stranger.sign(message, drone.now, 0)));
    }
    // Unsigned frames with the right sender: the wired policy is never reachable over radio.
    let mut unsigned = [0; 280];
    let n = wire::encode(76, &arm().1, 0, 255, 190, &mut unsigned).unwrap();
    assert!(!drone.radio(&unsigned[..n]));
    drone.tick(10_000);
    assert_eq!(drone.state(), State::Standby);
    assert_eq!(drone.owner.counters.rejected, 5);

    // Explicit wired revoke (all-zero key): the previously valid key stops working.
    drone.enroll([0; 32], STAMP).unwrap();
    assert!(!drone.session.enabled());
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    assert!(!drone.radio(&client.sign(heartbeat(), drone.now, 0)));
}

#[test]
fn every_single_bit_flip_is_refused_and_leaves_no_trace() {
    let mut drone = Drone::enrolled(KEY_A);
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    let original = client.sign(arm(), drone.now, 0);
    let mut refused = 0;
    for byte in 0..original.len() {
        for bit in 0..8 {
            let mut tampered = original.clone();
            tampered[byte] ^= 1 << bit;
            assert!(!drone.radio(&tampered), "byte {byte} bit {bit} admitted");
            refused += 1;
        }
    }
    assert_eq!(refused, original.len() * 8);
    // Truncated and extended copies are refused too.
    for length in 0..original.len() {
        assert!(!drone.radio(&original[..length]));
    }
    let mut longer = original.clone();
    longer.push(0);
    assert!(!drone.radio(&longer));
    // No refused copy advanced a stream or took a slot: the untouched original still works.
    drone.tick(1_000);
    assert_eq!(drone.state(), State::Standby);
    assert!(drone.radio(&original));
}

#[test]
fn duplicate_and_older_frames_are_dropped_not_reordered() {
    let mut drone = Drone::enrolled(KEY_A);
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    let first = client.sign(heartbeat(), drone.now, 0);
    let second = client.sign(heartbeat(), drone.now, 0);
    let third = client.sign(heartbeat(), drone.now, 0);
    assert!(drone.radio(&second));
    assert!(!drone.radio(&first), "older than the newest seen");
    assert!(!drone.radio(&second), "duplicate");
    assert!(drone.radio(&third));
    assert!(!drone.radio(&third));
    // A stop captured earlier cannot be replayed later either; a fresh one works.
    drone.fly(&mut client);
    let mut late = Client::new(KEY_A, STAMP, 1_000);
    let old_stop = late.sign(stop(), 1_000, 0);
    assert!(!drone.radio(&old_stop));
    assert!(drone.radio(&client.sign(stop(), drone.now, 0)));
    assert_eq!(drone.tick(1_000), [0.; 4]);
}

#[test]
fn stream_table_is_bounded_and_cannot_be_filled_without_the_key() {
    let mut drone = Drone::enrolled(KEY_A);
    let mut attacker = Client::new(KEY_B, STAMP, drone.now);
    // Forged frames on every link ID: none may occupy a replay-stream slot.
    for link in 0..=255 {
        assert!(!drone.radio(&attacker.sign(heartbeat(), drone.now, link)));
    }
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    for link in 0..4 {
        assert!(drone.radio(&client.sign(heartbeat(), drone.now, link)));
    }
    // A fifth stream from the key holder is refused (bounded table); the four keep working.
    assert!(!drone.radio(&client.sign(heartbeat(), drone.now, 4)));
    for link in 0..4 {
        assert!(drone.radio(&client.sign(heartbeat(), drone.now, link)));
    }
}

#[test]
fn restart_revokes_and_pre_reboot_traffic_stays_refused() {
    let drone = Drone::enrolled(KEY_A);
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    let captured: Vec<_> = [stab(), sticks(0.), arm(), sticks(0.4)]
        .into_iter()
        .map(|m| client.sign(m, drone.now, 0))
        .collect();

    // Reboot: no session at all until a new wired enrollment.
    let mut drone = Drone::booted();
    for frame in &captured {
        assert!(!drone.radio(frame));
    }
    // Fresh enrollment, new key (what the tool does): captured traffic stays refused.
    drone.enroll(KEY_B, STAMP + 500_000).unwrap();
    for frame in &captured {
        assert!(!drone.radio(frame));
    }
    // Even if an operator reused the old key, the new trusted time floor refuses the capture.
    let mut drone = Drone::booted();
    drone.enroll(KEY_A, STAMP + 500_000).unwrap();
    for frame in &captured {
        assert!(!drone.radio(frame));
    }
    drone.tick(10_000);
    assert_eq!(drone.state(), State::Standby);
}

#[test]
fn delayed_traffic_older_than_enrollment_or_sixty_seconds_is_refused() {
    let mut drone = Drone::enrolled(KEY_A);
    // Signed for a time before enrollment (a pre-enrollment capture under the same key).
    let mut early = Client::new(KEY_A, STAMP - 10, drone.now);
    assert!(!drone.radio(&early.sign(arm(), drone.now, 0)));

    let mut client = Client::new(KEY_A, STAMP, drone.now);
    let held = client.sign(stab(), drone.now, 1);
    // Held back 59 s: still inside the MAVLink 60 s window and newest on its stream.
    let mut late = Client::new(KEY_A, STAMP, drone.now);
    let held_59 = late.sign(heartbeat(), drone.now, 2);
    drone.now += 59_000_000;
    assert!(drone.radio(&held_59));
    // Held back 61 s: refused even though it was never seen.
    drone.now += 2_000_000;
    assert!(!drone.radio(&held));
    // Current traffic still works.
    assert!(drone.radio(&client.sign(heartbeat(), drone.now, 1)));
}

#[test]
fn rotation_needs_a_stopped_drone_and_drops_the_old_key() {
    let mut drone = Drone::enrolled(KEY_A);
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    drone.fly(&mut client);
    // Armed: rotation is refused and the flight continues under the current key.
    assert!(drone.enroll(KEY_B, STAMP + 1).is_err());
    assert_eq!(drone.state(), State::Armed);
    assert!(drone.session.enabled());
    assert!(drone.radio(&client.sign(sticks(0.4), drone.now, 0)));

    assert!(drone.radio(&client.sign(stop(), drone.now, 0)));
    drone.tick(1_000);
    let stamp = STAMP + drone.now / 10;
    drone.enroll(KEY_B, stamp).unwrap();
    assert!(!drone.radio(&client.sign(heartbeat(), drone.now, 0)));
    let mut rotated = Client::new(KEY_B, stamp, drone.now);
    drone.fly(&mut rotated);

    // Interrupted rotation (wrong destination): radio is off, motors stopped.
    assert!(drone.radio(&rotated.sign(stop(), drone.now, 0)));
    drone.tick(1_000);
    let mut broken = enrollment(KEY_A, stamp + 1);
    broken.payload[8] = 9;
    drone.owner.flight.maintenance();
    assert!(
        drone
            .session
            .enroll(&broken, &mut drone.owner, Tick(drone.now))
            .is_err()
    );
    drone.owner.flight.disarm();
    assert!(!drone.session.enabled());
    assert!(!drone.radio(&rotated.sign(arm(), drone.now, 0)));
    assert_eq!(drone.tick(10_000), [0.; 4]);
}

#[test]
fn transport_loss_expires_the_lease_and_reconnect_does_not_resume() {
    let mut drone = Drone::enrolled(KEY_A);
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    drone.fly(&mut client);
    // Link cut: no frames at all. The 1 s command lease runs out and the motors stop.
    assert_eq!(drone.tick(1_100_000), [0.; 4]);
    let snapshot = drone.owner.flight.snapshot();
    assert_ne!(snapshot.state, State::Armed);
    assert_eq!(snapshot.reason, Reason::CommandExpired);
    // Link back: fresh, valid control is admitted but does not spin the motors again.
    for _ in 0..100 {
        assert!(drone.radio(&client.sign(sticks(0.4), drone.now, 0)));
        assert_eq!(drone.tick(10_000), [0.; 4]);
    }
    assert_ne!(drone.state(), State::Armed);
}

#[test]
fn accepted_heartbeats_and_admission_do_not_renew_control_authority() {
    let mut drone = Drone::enrolled(KEY_A);
    let mut client = Client::new(KEY_A, STAMP, drone.now);
    drone.fly(&mut client);
    // Only heartbeats (and other non-control traffic) keep arriving, all of them accepted.
    let mut motors = [1.; 4];
    for _ in 0..120 {
        assert!(drone.radio(&client.sign(heartbeat(), drone.now, 0)));
        motors = drone.tick(10_000);
    }
    assert_eq!(motors, [0.; 4]);
    assert_eq!(drone.owner.flight.snapshot().reason, Reason::CommandExpired);
}

#[test]
fn stop_is_not_displaced_by_a_flood_of_accepted_traffic() {
    for stop_first in [false, true] {
        let mut drone = Drone::enrolled(KEY_A);
        let mut client = Client::new(KEY_A, STAMP, drone.now);
        drone.fly(&mut client);
        let flood = |drone: &mut Drone, client: &mut Client| {
            for i in 0..1_000 {
                // Idle sticks plus arm: exactly what a re-arm needs, so only the stop keeps it down.
                let message = if i % 2 == 0 { sticks(0.) } else { arm() };
                // Accepted or dropped at the full queue; either way never an error path.
                drone.radio(&client.sign(message, drone.now, 0));
            }
        };
        if !stop_first {
            flood(&mut drone, &mut client);
        }
        assert!(drone.radio(&client.sign(stop(), drone.now, 0)));
        if stop_first {
            flood(&mut drone, &mut client);
        }
        assert_eq!(drone.tick(1_000), [0.; 4], "stop_first {stop_first}");
        assert_eq!(drone.state(), State::Standby);
        // The arms queued in the flood were discarded with the stop, not replayed later.
        assert_eq!(drone.tick(100_000), [0.; 4]);
        assert_eq!(drone.state(), State::Standby);
    }
}
