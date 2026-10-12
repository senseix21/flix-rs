//! Model-based safety properties (SAF-01/02/03/05/07/08) over random operation
//! sequences. Each oracle is stated from the public contract, not from the
//! implementation: it only reads snapshots, return values and committed output.
use flight_core::control::{Intent, Pilot, Tuning};
use flight_core::{
    Flight, Limits, MotorFrame, Quaternion, Reason, Sensor, Source, State, Tick, Vec3,
};
use proptest::prelude::*;

const DIAGNOSTIC_MAX: f32 = 0.15;

#[derive(Clone, Copy, Debug)]
enum Step {
    Normal(u64),
    Zero,
    Gap(u64),
    Backwards(u64),
}

#[derive(Clone, Copy, Debug)]
enum Sample {
    Fresh,
    Stale(u64),
    NonFinite,
    Uncalibrated,
    Missing,
}

#[derive(Clone, Copy, Debug)]
enum Op {
    Tick(Step, Sample),
    Command(Source, Intent),
    /// A well-formed command for the selected source, so armed flight is reachable.
    Fly(f32, f32),
    /// Select a source and send a low-thrust command so arming is reachable.
    Ready(Source),
    Select(Source),
    Arm,
    Disarm,
    Fault(Reason),
    ClearFault,
    Maintenance,
    Diagnostic([f32; 4], u64),
    Reset,
    /// Commit a recent frame (0 = newest) after an extra delay, as a late driver would.
    Replay(usize, u64),
}

fn scalar() -> impl Strategy<Value = f32> {
    prop_oneof![
        6 => -1.0f32..=1.0,
        1 => Just(0.0),
        1 => Just(f32::NAN),
        1 => Just(f32::INFINITY),
        1 => Just(f32::NEG_INFINITY),
        1 => Just(1e30),
    ]
}

fn thrust() -> impl Strategy<Value = f32> {
    prop_oneof![
        4 => 0.0f32..=1.0,
        2 => 0.0f32..0.05,
        1 => Just(f32::NAN),
        1 => Just(-0.5),
        1 => Just(2.0),
    ]
}

fn vec3() -> impl Strategy<Value = Vec3> {
    (scalar(), scalar(), scalar()).prop_map(|(x, y, z)| Vec3(x, y, z))
}

fn pilot() -> impl Strategy<Value = Pilot> {
    (scalar(), scalar(), scalar(), thrust()).prop_map(|(roll, pitch, yaw, thrust)| Pilot {
        roll,
        pitch,
        yaw,
        thrust,
    })
}

fn intent() -> impl Strategy<Value = Intent> {
    prop_oneof![
        pilot().prop_map(Intent::Stabilized),
        pilot().prop_map(Intent::Acro),
        (vec3(), vec3(), thrust()).prop_map(|(euler, feedforward, thrust)| Intent::Attitude {
            orientation: Quaternion::euler(euler).unwrap_or(Quaternion::IDENTITY),
            feedforward,
            thrust,
        }),
        (vec3(), thrust()).prop_map(|(rates, thrust)| Intent::Rates { rates, thrust }),
    ]
}

fn source() -> impl Strategy<Value = Source> {
    prop_oneof![
        Just(Source::Pilot),
        Just(Source::Automatic),
        Just(Source::Rc)
    ]
}

fn op() -> impl Strategy<Value = Op> {
    let step = prop_oneof![
        // Faults latch, so anomalies must be rare for armed flight to stay reachable.
        80 => (1u64..=1_500).prop_map(Step::Normal),
        1 => Just(Step::Zero),
        1 => (10_001u64..=50_000).prop_map(Step::Gap),
        1 => (1u64..=1_000).prop_map(Step::Backwards),
    ];
    let sample = prop_oneof![
        80 => Just(Sample::Fresh),
        1 => (2_001u64..=10_000).prop_map(Sample::Stale),
        1 => Just(Sample::NonFinite),
        1 => Just(Sample::Uncalibrated),
        1 => Just(Sample::Missing),
    ];
    let diagnostic = (
        prop::array::uniform4(prop_oneof![
            12 => 0.0f32..=DIAGNOSTIC_MAX,
            1 => DIAGNOSTIC_MAX..=0.3,
            1 => Just(f32::NAN),
        ]),
        prop_oneof![3 => 1u64..=5_000, 1 => 0u64..=150_000],
    );
    prop_oneof![
        30 => (step, sample).prop_map(|(s, m)| Op::Tick(s, m)),
        6 => (source(), intent()).prop_map(|(s, i)| Op::Command(s, i)),
        8 => (0.0f32..=1.0, -1.0f32..=1.0).prop_map(|(t, r)| Op::Fly(t, r)),
        4 => source().prop_map(Op::Ready),
        1 => source().prop_map(Op::Select),
        4 => Just(Op::Arm),
        2 => Just(Op::Disarm),
        1 => prop_oneof![Just(Reason::Tilt), Just(Reason::Configuration)].prop_map(Op::Fault),
        4 => Just(Op::ClearFault),
        2 => Just(Op::Maintenance),
        4 => diagnostic.prop_map(|(v, d)| Op::Diagnostic(v, d)),
        1 => Just(Op::Reset),
        6 => (prop_oneof![3 => 0usize..=1, 1 => 2usize..16], 0u64..=12_000)
            .prop_map(|(i, d)| Op::Replay(i, d)),
    ]
}

fn finite_intent(intent: Intent) -> bool {
    let all = |v: &[f32]| v.iter().all(|x| x.is_finite());
    match intent {
        Intent::Stabilized(p) | Intent::Acro(p) => all(&[p.roll, p.pitch, p.yaw, p.thrust]),
        Intent::Attitude {
            feedforward: f,
            thrust,
            ..
        } => all(&[f.0, f.1, f.2, thrust]),
        Intent::Rates { rates: r, thrust } => all(&[r.0, r.1, r.2, thrust]),
        Intent::Position(t) => all(&[
            t.position.0,
            t.position.1,
            t.position.2,
            t.velocity.0,
            t.velocity.1,
            t.velocity.2,
            t.yaw,
        ]),
    }
}

fn stopped(out: [f32; 4]) -> bool {
    out == [0.0; 4]
}

/// A frame and what the oracle knew when it was issued.
#[derive(Clone, Copy)]
struct Issued {
    frame: MotorFrame,
    at: u64,
    generation: u64,
}

fn limits() -> impl Strategy<Value = Limits> {
    // Short leases and frames make expiry reachable within one sequence.
    let command = prop_oneof![500u64..=5_000, 5_000u64..=50_000, Just(1_000_000)];
    (command, 500u64..=10_000).prop_map(|(command_us, frame_us)| Limits {
        command_us,
        frame_us,
        ..Limits::DEFAULT
    })
}

struct Model {
    flight: Flight,
    limits: Limits,
    now: u64,
    /// Latest accepted command time and intent per source (Pilot, Automatic, Rc).
    accepted: [Option<(u64, Intent)>; 3],
    /// Latest accepted diagnostic start and duration.
    diagnostic: Option<(u64, u64)>,
    history: Vec<Issued>,
}

impl Model {
    fn new(limits: Limits) -> Self {
        let mut flight = Flight::new(Tuning::default(), limits).expect("valid test limits");
        // Start in Standby so arming paths are reachable within short sequences.
        flight.tick(Tick(0), Some(sensor(0, Sample::Fresh)));
        Self {
            flight,
            limits,
            now: 0,
            accepted: [None; 3],
            diagnostic: None,
            history: Vec::new(),
        }
    }

    fn command(&mut self, source: Source, intent: Intent) -> bool {
        let armed = self.flight.snapshot().state == State::Armed;
        let previous = self.accepted[source as usize];
        let accepted = self.flight.command(source, intent, Tick(self.now));
        if armed && self.flight.snapshot().state == State::Armed {
            // A mode change while armed must stop, never take effect in flight.
            if let (true, Some((_, old))) = (accepted, previous) {
                assert_eq!(
                    core::mem::discriminant(&old),
                    core::mem::discriminant(&intent),
                    "armed mode change accepted: {old:?} -> {intent:?}"
                );
            }
        }
        if accepted {
            self.accepted[source as usize] = Some((self.now, intent));
        }
        accepted
    }

    /// Check every committed value against the contract for the current state.
    fn check_output(&self, out: [f32; 4], at: u64) -> Result<(), TestCaseError> {
        let state = self.flight.snapshot().state;
        prop_assert!(
            out.iter().all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
            "output out of range: {out:?}"
        );
        if !matches!(state, State::Armed | State::Maintenance) {
            prop_assert!(stopped(out), "{state:?} produced {out:?}");
        }
        if state == State::Maintenance {
            let active = self
                .diagnostic
                .is_some_and(|(start, duration)| at >= start && at - start < duration);
            if !active {
                prop_assert!(stopped(out), "expired diagnostic produced {out:?}");
            }
            prop_assert!(
                out.iter().all(|v| *v <= DIAGNOSTIC_MAX),
                "maintenance exceeded diagnostic limit: {out:?}"
            );
        }
        if state == State::Armed {
            let lease = self
                .flight
                .snapshot()
                .source
                .and_then(|s| self.accepted[s as usize]);
            let command_us = self.limits.command_us;
            if lease.is_none_or(|(t, _)| at.checked_sub(t).is_none_or(|age| age > command_us)) {
                prop_assert!(stopped(out), "expired lease produced {out:?}");
            }
        }
        Ok(())
    }

    fn check_replay(&self, issued: Issued, delay: u64) -> Result<(), TestCaseError> {
        let at = self.now.saturating_add(delay);
        let out = self.flight.commit(issued.frame, Tick(at));
        self.check_output(out, at)?;
        let generation = self.flight.snapshot().generation;
        let expired = at < issued.at || at - issued.at >= self.limits.frame_us;
        if expired || issued.generation != generation {
            prop_assert!(stopped(out), "stale frame produced {out:?}");
        }
        Ok(())
    }

    fn apply(&mut self, op: Op) -> Result<(), TestCaseError> {
        let before = self.flight.snapshot();
        let mut armed_explicitly = false;
        let mut fault_cleared = false;
        match op {
            Op::Tick(step, sample) => {
                self.now = match step {
                    Step::Normal(d) | Step::Gap(d) => self.now.saturating_add(d),
                    Step::Zero => self.now,
                    Step::Backwards(d) => self.now.saturating_sub(d),
                };
                let reading = match sample {
                    Sample::Missing => None,
                    _ => Some(sensor(self.now, sample)),
                };
                let frame = self.flight.tick(Tick(self.now), reading);
                let issued = Issued {
                    frame,
                    at: self.now,
                    generation: self.flight.snapshot().generation,
                };
                self.check_output(self.flight.commit(frame, Tick(self.now)), self.now)?;
                if self.history.len() == 16 {
                    self.history.remove(0);
                }
                self.history.push(issued);
            }
            Op::Command(source, intent) => {
                let accepted = self.command(source, intent);
                if !finite_intent(intent) {
                    prop_assert!(!accepted, "non-finite intent accepted: {intent:?}");
                }
            }
            Op::Fly(thrust, roll) => {
                if let Some(source) = self.flight.snapshot().source {
                    let intent = match source {
                        Source::Automatic => Intent::Rates {
                            rates: Vec3(roll, 0.0, 0.0),
                            thrust,
                        },
                        _ => Intent::Stabilized(Pilot {
                            roll,
                            pitch: 0.0,
                            yaw: 0.0,
                            thrust,
                        }),
                    };
                    self.command(source, intent);
                }
            }
            Op::Ready(source) => {
                if self.flight.select(source) {
                    let intent = match source {
                        Source::Automatic => Intent::Rates {
                            rates: Vec3::ZERO,
                            thrust: 0.0,
                        },
                        _ => Intent::Stabilized(Pilot {
                            roll: 0.0,
                            pitch: 0.0,
                            yaw: 0.0,
                            thrust: 0.0,
                        }),
                    };
                    self.command(source, intent);
                }
            }
            Op::Select(source) => {
                self.flight.select(source);
            }
            Op::Arm => {
                armed_explicitly = self.flight.arm(Tick(self.now));
                if armed_explicitly {
                    // SAF-05: arming requires a fresh, near-zero-thrust command.
                    let lease = self
                        .flight
                        .snapshot()
                        .source
                        .and_then(|s| self.accepted[s as usize]);
                    prop_assert!(
                        lease.is_some_and(|(at, intent)| self.now - at <= self.limits.command_us
                            && intent.thrust() < 0.05),
                        "armed with lease {lease:?}"
                    );
                }
            }
            Op::Disarm => {
                self.flight.disarm();
                prop_assert_ne!(self.flight.snapshot().state, State::Armed);
                for issued in &self.history {
                    let out = self.flight.commit(issued.frame, Tick(self.now));
                    prop_assert!(stopped(out), "pre-disarm frame produced {out:?}");
                }
            }
            Op::Fault(reason) => {
                self.flight.fault(reason);
                prop_assert_eq!(self.flight.snapshot().state, State::Fault);
            }
            Op::ClearFault => fault_cleared = self.flight.clear_fault(),
            Op::Maintenance => {
                self.flight.maintenance();
            }
            Op::Diagnostic(values, duration) => {
                let accepted = self.flight.diagnostic(values, Tick(self.now), duration);
                if accepted {
                    self.diagnostic = Some((self.now, duration));
                    prop_assert!(values.iter().all(|v| v.is_finite() && *v <= DIAGNOSTIC_MAX));
                    prop_assert!(duration > 0 && duration <= 100_000);
                }
            }
            Op::Reset => {
                self.flight.reset();
                fault_cleared = true;
            }
            Op::Replay(back, delay) => {
                let newest = self.history.len().checked_sub(1 + back);
                if let Some(issued) = newest.map(|i| self.history[i]) {
                    self.check_replay(issued, delay)?;
                }
            }
        }
        let after = self.flight.snapshot();
        prop_assert!(
            after.generation >= before.generation,
            "generation decreased: {} -> {}",
            before.generation,
            after.generation
        );
        if before.state != State::Armed && after.state == State::Armed {
            prop_assert!(armed_explicitly, "entered Armed without arm(): {op:?}");
        }
        if before.state == State::Fault && after.state != State::Fault {
            prop_assert!(fault_cleared, "left Fault without clear/reset: {op:?}");
        }
        Ok(())
    }
}

fn sensor(now: u64, sample: Sample) -> Sensor {
    let (at, gyro, calibrated) = match sample {
        Sample::Stale(age) => (now.saturating_sub(age), Vec3::ZERO, true),
        Sample::NonFinite => (now, Vec3(f32::NAN, 0.0, 0.0), true),
        Sample::Uncalibrated => (now, Vec3::ZERO, false),
        Sample::Fresh | Sample::Missing => (now, Vec3::ZERO, true),
    };
    Sensor {
        at: Tick(at),
        gyro,
        acceleration: Vec3(0.0, 0.0, flight_core::control::GRAVITY),
        calibrated,
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        ..ProptestConfig::default()
    })]
    #[test]
    fn safety_contract_holds_for_any_operation_sequence(
        limits in limits(),
        ops in prop::collection::vec(op(), 1..300),
    ) {
        let mut model = Model::new(limits);
        for op in ops {
            model.apply(op)?;
        }
    }
}
