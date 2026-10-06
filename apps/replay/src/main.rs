use flight_core::control::{GRAVITY, Intent, Pilot};
use flight_core::{Flight, Sensor, Source, Tick, Vec3};
use std::{
    error::Error,
    io::{self, BufRead},
    time::Instant,
};
fn sensor(at: Tick) -> Sensor {
    Sensor {
        at,
        gyro: Vec3::ZERO,
        acceleration: Vec3(0.0, 0.0, GRAVITY),
        calibrated: true,
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().collect();
    let mut flight = Flight::default();
    if args.get(1).is_some_and(|v| v == "--oracle") {
        use flight_core::control::{Controller, Estimator, Tuning};
        for line in io::stdin().lock().lines() {
            let line = line?;
            let mut fields = line.split(',');
            let kind = fields.next().ok_or("kind")?;
            let v: Vec<f32> = fields.map(str::parse).collect::<Result<_, _>>()?;
            let output: Vec<f32> = match kind {
                "mix" if v.len() == 4 => flight_core::control::mix(v[0], Vec3(v[1], v[2], v[3]))
                    .ok_or("numeric")?
                    .to_vec(),
                "quaternion" if v.len() == 3 => {
                    flight_core::Quaternion::euler(Vec3(v[0], v[1], v[2]))
                        .ok_or("numeric")?
                        .array()
                        .to_vec()
                }
                "control" if v.len() == 11 => {
                    let p = Pilot {
                        roll: v[0],
                        pitch: v[1],
                        yaw: v[2],
                        thrust: v[3],
                    };
                    let intent = match v[10] {
                        1.0 => Intent::Acro(p),
                        2.0 => Intent::Stabilized(p),
                        _ => return Err("unsupported oracle mode".into()),
                    };
                    let mut e = Estimator::default();
                    e.attitude =
                        flight_core::Quaternion::euler(Vec3(v[4], v[5], v[6])).ok_or("numeric")?;
                    e.rates = Vec3(v[7], v[8], v[9]);
                    Controller::default()
                        .update(intent, e, 0.001, Tuning::default())
                        .ok_or("control")?
                        .to_vec()
                }
                _ => return Err("oracle input".into()),
            };
            println!(
                "{}",
                output
                    .iter()
                    .map(|v| format!("{v:.9}"))
                    .collect::<Vec<_>>()
                    .join(",")
            );
        }
        return Ok(());
    }
    if args.get(1).is_some_and(|v| v == "--benchmark") {
        flight.select(Source::Pilot);
        flight.command(
            Source::Pilot,
            Intent::Stabilized(Pilot {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                thrust: 0.0,
            }),
            Tick(0),
        );
        flight.tick(Tick(0), Some(sensor(Tick(0))));
        assert!(flight.arm(Tick(0)));
        let start = Instant::now();
        for i in 1..=100_000 {
            let at = Tick(i * 1000);
            flight.command(
                Source::Pilot,
                Intent::Stabilized(Pilot {
                    roll: 0.1,
                    pitch: 0.0,
                    yaw: 0.0,
                    thrust: 0.4,
                }),
                at,
            );
            let frame = flight.tick(at, Some(sensor(at)));
            std::hint::black_box(flight.commit(frame, at));
        }
        println!(
            "host_only armed_stabilized_ticks=100000 elapsed_us={} core_bytes={} owner_bytes={}",
            start.elapsed().as_micros(),
            std::mem::size_of::<Flight>(),
            std::mem::size_of::<flight_esp32::Owner>()
        );
        return Ok(());
    }
    if args.get(1).is_some_and(|v| v == "--demo") {
        for i in 0..12 {
            let at = Tick(i * 1000);
            if i == 1 {
                flight.select(Source::Pilot);
                flight.command(
                    Source::Pilot,
                    Intent::Stabilized(Pilot {
                        roll: 0.0,
                        pitch: 0.0,
                        yaw: 0.0,
                        thrust: 0.0,
                    }),
                    at,
                );
                flight.arm(at);
            }
            if i == 2 {
                flight.command(
                    Source::Pilot,
                    Intent::Stabilized(Pilot {
                        roll: 0.1,
                        pitch: 0.0,
                        yaw: 0.0,
                        thrust: 0.4,
                    }),
                    at,
                );
            }
            if i == 8 {
                flight.disarm();
            }
            let frame = flight.tick(at, Some(sensor(at)));
            println!(
                "{} {:?} {:?} {:?}",
                at.0,
                flight.snapshot().state,
                flight.snapshot().reason,
                flight.commit(frame, at)
            );
        }
        return Ok(());
    }
    eprintln!(
        "CSV stdin: us,gx,gy,gz,ax,ay,az,roll,pitch,yaw,thrust,arm (0 disarm/1 arm/2 hold); --demo or --benchmark"
    );
    println!("us,state,reason,rl,rr,fr,fl");
    flight.select(Source::Pilot);
    for line in io::stdin().lock().lines() {
        let line = line?;
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() != 12 {
            return Err("expected 12 CSV fields".into());
        }
        let at = Tick(fields[0].parse()?);
        let v: Vec<f32> = fields[1..11]
            .iter()
            .map(|v| v.parse())
            .collect::<Result<_, _>>()?;
        let sample = Sensor {
            at,
            gyro: Vec3(v[0], v[1], v[2]),
            acceleration: Vec3(v[3], v[4], v[5]),
            calibrated: true,
        };
        flight.command(
            Source::Pilot,
            Intent::Stabilized(Pilot {
                roll: v[6],
                pitch: v[7],
                yaw: v[8],
                thrust: v[9],
            }),
            at,
        );
        let action: u8 = fields[11].parse()?;
        if action == 0 {
            flight.disarm();
        } else if action > 2 {
            return Err("invalid arm action".into());
        }
        let frame = flight.tick(at, Some(sample));
        if action == 1 {
            flight.arm(at);
        }
        let m = flight.commit(frame, at);
        let s = flight.snapshot();
        println!(
            "{},{:?},{:?},{},{},{},{}",
            at.0, s.state, s.reason, m[0], m[1], m[2], m[3]
        );
    }
    Ok(())
}
