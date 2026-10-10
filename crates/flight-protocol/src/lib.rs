#![no_std]
#![forbid(unsafe_code)]
// Panics in the portable core are visible, reasoned exceptions only.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented,
    clippy::unreachable,
    clippy::dbg_macro,
    clippy::print_stdout,
    clippy::print_stderr
)]
// Lossy numeric conversions and exact float equality must state why they are sound.
#![deny(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::float_cmp
)]
pub mod ingress;
pub mod peers;
pub mod proxy;
pub mod telemetry;
pub mod wire;
use flight_core::control::{Intent, Pilot};
use flight_core::{Quaternion, Source, Vec3};
use wire::Packet;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Framing,
    Checksum,
    Unsupported,
    Unauthorized,
    Replay,
    Destination,
    Bounds,
    Numeric,
    Full,
}
#[derive(Clone, Copy, Debug)]
pub enum Event {
    Heartbeat,
    Arm,
    Stop,
    Select(Source),
    Mode(u8),
    UnsupportedCommand(u16),
    VersionRequested(u16),
    ProtocolRequested(u16),
    Diagnostic {
        motor: u8,
        throttle: f32,
        duration_us: u32,
    },
    Setpoint(Source, Intent),
    ParameterRead {
        index: i16,
        name: [u8; 16],
    },
    ParameterList,
    ParameterSet {
        name: [u8; 16],
        value: f32,
    },
    Console(Console),
    Log {
        offset: u32,
        count: u32,
    },
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Console {
    Status,
    Disarm,
    Maintenance,
    Exit,
    ClearFault,
    Save,
    CalibrateGyro,
    Metrics(u32),
}
pub fn console(bytes: &[u8]) -> Result<Console, Error> {
    let text = core::str::from_utf8(bytes)
        .map_err(|_| Error::Bounds)?
        .trim();
    if let Some(nonce) = text.strip_prefix("metrics ") {
        if nonce.len() != 8 || !nonce.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(Error::Bounds);
        }
        return u32::from_str_radix(nonce, 16)
            .map(Console::Metrics)
            .map_err(|_| Error::Bounds);
    }
    match text {
        "status" => Ok(Console::Status),
        "disarm" => Ok(Console::Disarm),
        "maintenance" => Ok(Console::Maintenance),
        "exit" => Ok(Console::Exit),
        "clear-fault" => Ok(Console::ClearFault),
        "save" => Ok(Console::Save),
        "calibrate-gyro" => Ok(Console::CalibrateGyro),
        _ => Err(Error::Unsupported),
    }
}
fn f32_at(p: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes([p[offset], p[offset + 1], p[offset + 2], p[offset + 3]])
}
fn i16_at(p: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([p[offset], p[offset + 1]])
}
fn destination(p: &[u8], system: usize, component: Option<usize>, local: u8) -> Result<(), Error> {
    if p[system] != 0 && p[system] != local || component.is_some_and(|i| p[i] != 0 && p[i] != 1) {
        return Err(Error::Destination);
    }
    Ok(())
}
#[expect(
    clippy::float_cmp,
    reason = "MAVLink encodes integer command arguments as exact floats; tolerance would accept 0.5 as Stop"
)]
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "each cast follows a range and integrality check: motor 1..=4, duration <= 0.1 s, mode 1..=3"
)]
pub fn event(packet: &Packet, local: u8) -> Result<Event, Error> {
    let p = &packet.payload;
    match packet.id {
        0 => Ok(Event::Heartbeat),
        20 => {
            destination(p, 2, Some(3), local)?;
            Ok(Event::ParameterRead {
                index: i16_at(p, 0),
                name: p[4..20].try_into().map_err(|_| Error::Bounds)?,
            })
        }
        21 => {
            destination(p, 0, Some(1), local)?;
            Ok(Event::ParameterList)
        }
        23 => {
            destination(p, 4, Some(5), local)?;
            let value = f32_at(p, 0);
            if !value.is_finite() || p[22] != 9 {
                return Err(Error::Numeric);
            }
            Ok(Event::ParameterSet {
                name: p[6..22].try_into().map_err(|_| Error::Bounds)?,
                value,
            })
        }
        69 => {
            destination(p, 10, None, local)?;
            if p[11..usize::from(packet.length).max(11)]
                .iter()
                .any(|b| *b != 0)
            {
                return Err(Error::Unsupported);
            }
            let x = i16_at(p, 0);
            let y = i16_at(p, 2);
            let z = i16_at(p, 4);
            let r = i16_at(p, 6);
            let pilot = Pilot {
                roll: f32::from(y) / 1000.0,
                pitch: f32::from(x) / 1000.0,
                yaw: f32::from(r) / 1000.0,
                thrust: f32::from(z) / 1000.0,
            };
            if !pilot.valid() {
                return Err(Error::Numeric);
            }
            Ok(Event::Setpoint(Source::Pilot, Intent::Stabilized(pilot)))
        }
        76 => {
            destination(p, 30, Some(31), local)?;
            let command = u16::from_le_bytes([p[28], p[29]]);
            let args = core::array::from_fn::<_, 7, _>(|i| f32_at(p, i * 4));
            if !args.iter().all(|v| v.is_finite()) {
                return Err(Error::Numeric);
            }
            match command {
                512 if args[0] == 300.0 && args[1..].iter().all(|v| *v == 0.0) => {
                    Ok(Event::ProtocolRequested(command))
                }
                519 if args[0] == 1.0 && args[1..].iter().all(|v| *v == 0.0) => {
                    Ok(Event::ProtocolRequested(command))
                }
                512 if args[0] == 148.0 && args[1..].iter().all(|v| *v == 0.0) => {
                    Ok(Event::VersionRequested(command))
                }
                520 if args[0] == 1.0 && args[1..].iter().all(|v| *v == 0.0) => {
                    Ok(Event::VersionRequested(command))
                }
                209 if args[0] >= 1.0
                    && args[0] <= 4.0
                    && args[0] == args[0] as u8 as f32
                    && args[1] == 0.0
                    && (0.0..=15.0).contains(&args[2])
                    && (0.001..=0.1).contains(&args[3])
                    && args[4] == 1.0
                    && args[5] == 0.0
                    && args[6] == 0.0 =>
                {
                    Ok(Event::Diagnostic {
                        motor: args[0] as u8 - 1,
                        throttle: args[2] / 100.0,
                        duration_us: (args[3] * 1e6) as u32,
                    })
                }
                400 if args[1..].iter().all(|v| *v == 0.0) => match args[0] {
                    0.0 => Ok(Event::Stop),
                    1.0 => Ok(Event::Arm),
                    _ => Err(Error::Bounds),
                },
                176 if args[0] == 1.0 && args[2..].iter().all(|v| *v == 0.0) => match args[1] {
                    1.0 | 2.0 | 3.0 => Ok(Event::Mode(args[1] as u8)),
                    _ => Ok(Event::UnsupportedCommand(command)),
                },
                _ => Ok(Event::UnsupportedCommand(command)),
            }
        }
        82 => {
            destination(p, 36, Some(37), local)?;
            let mask = p[38];
            if mask & 0x78 != 0
                || p[39..usize::from(packet.length).max(39)]
                    .iter()
                    .any(|b| *b != 0)
            {
                return Err(Error::Unsupported);
            }
            let mut rate = [0.0; 3];
            for (i, v) in rate.iter_mut().enumerate() {
                if mask & (1 << i) == 0 {
                    *v = f32_at(p, 20 + i * 4);
                }
            }
            let rates = Vec3(rate[0], -rate[1], -rate[2]);
            let thrust = f32_at(p, 32);
            let intent = if mask & 128 != 0 {
                Intent::Rates { rates, thrust }
            } else {
                let qraw = [f32_at(p, 4), f32_at(p, 8), -f32_at(p, 12), -f32_at(p, 16)];
                let norm_squared = qraw.iter().map(|v| v * v).sum::<f32>();
                if !norm_squared.is_finite() || !(0.81..=1.21).contains(&norm_squared) {
                    return Err(Error::Numeric);
                }
                let q = Quaternion::new(qraw).ok_or(Error::Numeric)?;
                Intent::Attitude {
                    orientation: q,
                    feedforward: rates,
                    thrust,
                }
            };
            if !intent.valid() {
                return Err(Error::Numeric);
            }
            Ok(Event::Setpoint(Source::Automatic, intent))
        }
        119 => {
            destination(p, 10, Some(11), local)?;
            if u16::from_le_bytes([p[8], p[9]]) != 0 {
                return Err(Error::Bounds);
            }
            Ok(Event::Log {
                offset: u32::from_le_bytes(p[..4].try_into().map_err(|_| Error::Bounds)?),
                count: u32::from_le_bytes(p[4..8].try_into().map_err(|_| Error::Bounds)?),
            })
        }
        126 => {
            let count = usize::from(p[8]);
            if count > 70 || p[6] != 10 || p[7] & !6 != 0 {
                return Err(Error::Bounds);
            }
            if packet.length > 79 {
                destination(p, 79, Some(80), local)?;
            }
            Ok(Event::Console(console(&p[9..9 + count])?))
        }
        _ => Err(Error::Unsupported),
    }
}
