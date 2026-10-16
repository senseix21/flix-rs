//! Logical-clock, loopback-only simulator for client tests. No physical motor adapter.
use flight_core::{Sensor, Tick, Vec3};
use flight_esp32::Owner;
use flight_esp32::session::RadioSession;
use flight_protocol::wire::Policy;
use std::{net::UdpSocket, time::Duration};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port: u16 = std::env::args().nth(1).unwrap_or("14550".into()).parse()?;
    let socket = UdpSocket::bind(("127.0.0.1", port))?;
    socket.set_nonblocking(true)?;
    println!("{}", socket.local_addr()?);
    use std::io::Write;
    std::io::stdout().flush()?;
    let mut owner = Owner::default();
    // Optional trusted local enrollment frame for real signed-client integration.
    // Never put keys in command-line arguments or stdout.
    let mut session = RadioSession::default();
    let signed = if let Some(path) = std::env::args().nth(2) {
        let metadata = std::fs::metadata(&path)?;
        if metadata.len() > 280 {
            return Err("enrollment frame too large".into());
        }
        let enrollment = std::fs::read(path)?;
        let mut wired = Policy::BenchUsb {
            system: 255,
            component: 190,
        };
        let packet = flight_protocol::wire::decode(&enrollment, &mut wired)
            .map_err(|_| "invalid enrollment frame")?;
        owner.tick(
            Tick(0),
            Some(Sensor {
                at: Tick(0),
                gyro: Vec3::ZERO,
                acceleration: Vec3(0., 0., 9.80665),
                calibrated: true,
            }),
        );
        owner.flight.maintenance();
        session
            .enroll(&packet, &mut owner, Tick(0))
            .map_err(|_| "enrollment denied")?;
        owner.flight.disarm();
        true
    } else {
        false
    };
    // Explicit loopback telemetry destination allows an actual GCS to discover
    // MAVLink 2 before it sends its first signed frame. Admission is unchanged.
    let mut endpoint = std::env::var("FLIX_BENCH_ENDPOINT")
        .ok()
        .map(|value| value.parse::<std::net::SocketAddr>())
        .transpose()?;
    if endpoint.is_some_and(|address| !address.ip().is_loopback()) {
        return Err("bench destination must be loopback".into());
    }
    let ticks: u64 = std::env::var("FLIX_BENCH_TICKS")
        .unwrap_or_else(|_| "5000".into())
        .parse()?;
    // Up to 15 minutes: long enough for an operator-driven client session (R2-04).
    if ticks > 900_000 {
        return Err("bench duration exceeds fifteen minutes".into());
    }
    let mut admitted = std::collections::BTreeMap::<u32, u64>::new();
    let mut rejected = 0_u64;
    let mut incoming = std::collections::BTreeMap::<(u8, u8, u32, u8), u64>::new();
    let mut outgoing = std::collections::BTreeMap::<u32, u64>::new();
    let mut policy = Policy::BenchUsb {
        system: 255,
        component: 190,
    };
    let mut bytes = [0; 281];
    for tick in 0..ticks {
        let at = Tick((tick + u64::from(signed)) * 1000);
        for _ in 0..4 {
            match socket.recv_from(&mut bytes) {
                Ok((length, source)) => {
                    if length >= 10 && bytes[0] == 253 {
                        let id = u32::from(bytes[7])
                            | (u32::from(bytes[8]) << 8)
                            | (u32::from(bytes[9]) << 16);
                        *incoming
                            .entry((bytes[5], bytes[6], id, bytes[2]))
                            .or_default() += 1;
                    }
                    let accepted = length <= 280
                        && if signed {
                            session.receive(&bytes[..length], &mut owner, at)
                        } else {
                            owner.receive(&bytes[..length], &mut policy, at)
                        };
                    if accepted {
                        endpoint = Some(source);
                        let message = u32::from(bytes[7])
                            | (u32::from(bytes[8]) << 8)
                            | (u32::from(bytes[9]) << 16);
                        *admitted.entry(message).or_default() += 1;
                    } else {
                        rejected += 1;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e.into()),
            }
        }
        owner.tick(
            at,
            Some(Sensor {
                at,
                gyro: Vec3::ZERO,
                acceleration: Vec3(0., 0., 9.80665),
                calibrated: true,
            }),
        );
        // Drain unroutable telemetry too, so a late client does not lose its
        // first ACK to a reply queue filled before endpoint admission.
        for _ in 0..4 {
            if let Some(mut reply) = owner.reply() {
                if let Some(endpoint) = endpoint {
                    let length = if signed {
                        session.sign(&mut reply.bytes, reply.len, at).unwrap_or(0)
                    } else {
                        reply.len
                    };
                    if length > 0 {
                        let id = u32::from(reply.bytes[7])
                            | (u32::from(reply.bytes[8]) << 8)
                            | (u32::from(reply.bytes[9]) << 16);
                        *outgoing.entry(id).or_default() += 1;
                        let _ = socket.send_to(&reply.bytes[..length], endpoint);
                    }
                }
            }
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    eprintln!(
        "admitted={admitted:?} rejected={rejected} incoming={incoming:?} outgoing={outgoing:?} state={:?}",
        owner.flight.snapshot().state
    );
    Ok(())
}
