use crate::control::{Controller, Estimator, Intent, Tuning};
use crate::position;
use crate::{Tick, Vec3};
use core::sync::atomic::{AtomicU32, Ordering};
static NEXT_ISSUER: AtomicU32 = AtomicU32::new(1);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum State {
    Boot = 0,
    Standby = 1,
    Maintenance = 2,
    Armed = 3,
    Failsafe = 4,
    Fault = 5,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Reason {
    None = 0,
    Disarmed = 1,
    NotReady = 2,
    CommandExpired = 3,
    SensorExpired = 4,
    InvalidNumeric = 5,
    Clock = 6,
    Generation = 7,
    Tilt = 8,
    MaintenanceExpired = 9,
    Configuration = 10,
    Power = 11,
    /// The IMU driver failed to configure (bus, identity or readback). Recoverable only
    /// after a rebuilt driver passes again (ADR 003 D2).
    SensorInit = 12,
    /// Flying under position control without a fresh external position (swarm SWM-07).
    PositionExpired = 13,
}
impl Reason {
    /// Short operator-facing name, used in STATUSTEXT (ADR 003 D3).
    pub fn label(self) -> &'static str {
        match self {
            Reason::None => "none",
            Reason::Disarmed => "disarmed",
            Reason::NotReady => "not ready",
            Reason::CommandExpired => "command expired",
            Reason::SensorExpired => "sensor expired",
            Reason::InvalidNumeric => "invalid numeric",
            Reason::Clock => "clock",
            Reason::Generation => "generation",
            Reason::Tilt => "tilt",
            Reason::MaintenanceExpired => "maintenance expired",
            Reason::Configuration => "configuration",
            Reason::Power => "power",
            Reason::SensorInit => "sensor init",
            Reason::PositionExpired => "position expired",
        }
    }
    /// Faults that only a rebuilt owner may leave (ADR 003 D1). Every exit from Fault
    /// (`clear_fault`, `reset`) consults this one predicate.
    pub fn latched(self) -> bool {
        matches!(self, Reason::Configuration | Reason::Generation)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Source {
    Pilot = 0,
    Automatic = 1,
    Rc = 2,
}
impl Source {
    fn index(self) -> usize {
        self as usize
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Sensor {
    pub at: Tick,
    pub gyro: Vec3,
    pub acceleration: Vec3,
    pub calibrated: bool,
}
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub command_us: u64,
    pub sensor_us: u64,
    pub frame_us: u64,
    pub gap_us: u64,
    pub max_tilt: f32,
}
impl Default for Limits {
    fn default() -> Self {
        Self::DEFAULT
    }
}
impl Limits {
    pub const DEFAULT: Self = Self {
        command_us: 1_000_000,
        sensor_us: 2_000,
        frame_us: 2_000,
        gap_us: 10_000,
        max_tilt: 2.0943952,
    };
}
impl Limits {
    fn valid(self) -> bool {
        self.command_us > 0
            && self.command_us <= 1_000_000
            && self.sensor_us > 0
            && self.sensor_us <= 10_000
            && self.frame_us > 0
            && self.frame_us <= 10_000
            && self.gap_us > 0
            && self.gap_us <= 100_000
            && self.max_tilt.is_finite()
            && self.max_tilt > 0.0
            && self.max_tilt <= core::f32::consts::PI
    }
}
#[derive(Clone, Copy, Debug)]
pub(crate) struct Lease {
    pub(crate) intent: Intent,
    pub(crate) at: Tick,
}
/// The first arming precondition that is not met. `Flight::arm` uses this same check,
/// so the reported cause cannot drift from the rule (ADR 003 D3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmBlock {
    NoOwner,
    State(State),
    Sensor,
    Calibration,
    SensorStale,
    NoCommand,
    Thrust,
}
impl ArmBlock {
    pub fn label(self) -> &'static str {
        match self {
            ArmBlock::NoOwner => "no owner",
            ArmBlock::State(State::Boot) => "booting",
            ArmBlock::State(State::Maintenance) => "maintenance",
            ArmBlock::State(State::Armed) => "already armed",
            // Failsafe reports as disarmed, so a ground station offers only Arm; name the
            // explicit acknowledgement that leaves it (ADR 003, R2-04 finding).
            ArmBlock::State(State::Failsafe) => "failsafe (send disarm)",
            ArmBlock::State(State::Fault) => "fault",
            ArmBlock::State(State::Standby) => "standby",
            ArmBlock::Sensor => "sensor",
            ArmBlock::Calibration => "calibration",
            ArmBlock::SensorStale => "sensor stale",
            ArmBlock::NoCommand => "no command",
            ArmBlock::Thrust => "thrust not low",
        }
    }
}
/// Only Flight can authorize frames. Copying a frame does not extend its authority.
#[derive(Clone, Copy, Debug)]
pub struct MotorFrame {
    pub(crate) issuer: u32,
    pub(crate) values: [f32; 4],
    pub(crate) generation: u64,
    pub(crate) issued: Tick,
    pub(crate) expires: Tick,
}
#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub state: State,
    pub reason: Reason,
    pub source: Option<Source>,
    pub generation: u64,
    pub estimate: Estimator,
    pub target_rates: Vec3,
    pub target_attitude: Option<crate::Quaternion>,
    pub position: position::Estimate,
}
pub struct Flight {
    pub(crate) issuer: u32,
    pub(crate) state: State,
    pub(crate) reason: Reason,
    pub(crate) generation: u64,
    pub(crate) selected: Option<Source>,
    pub(crate) leases: [Option<Lease>; 3],
    pub(crate) previous: Option<Tick>,
    pub(crate) sample: Option<Tick>,
    pub(crate) healthy: bool,
    pub(crate) calibrated: bool,
    estimate: Estimator,
    controller: Controller,
    tuning: Tuning,
    limits: Limits,
    /// Active diagnostic permit: values and its window `[start, expiry)`.
    pub(crate) diagnostic: Option<([f32; 4], Tick, Tick)>,
    last_kind: Option<u8>,
    position: position::Estimate,
    position_controller: position::Controller,
}
impl Default for Flight {
    #[expect(
        clippy::expect_used,
        reason = "built-in settings are valid; panics only after u32::MAX owners exhaust issuer IDs"
    )]
    fn default() -> Self {
        Self::new(Tuning::default(), Limits::default()).expect("valid built-in settings")
    }
}
impl Flight {
    pub const BOOT: Self = Self {
        issuer: 0,
        state: State::Boot,
        reason: Reason::NotReady,
        generation: 1,
        selected: None,
        leases: [None; 3],
        previous: None,
        sample: None,
        healthy: false,
        calibrated: false,
        estimate: Estimator::EMPTY,
        controller: Controller::EMPTY,
        tuning: Tuning::DEFAULT,
        limits: Limits::DEFAULT,
        diagnostic: None,
        last_kind: None,
        position: position::Estimate::EMPTY,
        position_controller: position::Controller::EMPTY,
    };

    pub fn new(tuning: Tuning, limits: Limits) -> Option<Self> {
        if !tuning.valid() || !limits.valid() {
            return None;
        }
        let issuer = NEXT_ISSUER
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .ok()?;
        Some(Self {
            issuer,
            tuning,
            limits,
            ..Self::BOOT
        })
    }
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            state: self.state,
            reason: self.reason,
            source: self.selected,
            generation: self.generation,
            estimate: self.estimate,
            target_rates: self.controller.target_rates,
            target_attitude: self.controller.target_attitude,
            position: self.position,
        }
    }
    pub fn stopped_maintenance(&self) -> bool {
        self.state == State::Maintenance && self.diagnostic.is_none()
    }
    /// Revoke all authority and return to Boot, except that a latched fault stays latched.
    pub fn reset(&mut self) {
        self.invalidate();
        if self.state == State::Fault && self.reason.latched() {
            return;
        }
        self.state = State::Boot;
        self.reason = Reason::NotReady;
        self.selected = None;
        self.previous = None;
        self.sample = None;
        self.estimate = Estimator::default();
        self.position = position::Estimate::EMPTY;
        self.healthy = false;
        self.calibrated = false;
    }
    pub fn reconfigure(&mut self, tuning: Tuning, limits: Limits) -> bool {
        if !self.stopped_maintenance() || !tuning.valid() || !limits.valid() {
            return false;
        }
        self.invalidate();
        self.tuning = tuning;
        self.limits = limits;
        self.estimate = Estimator::default();
        self.position = position::Estimate::EMPTY;
        self.sample = None;
        self.calibrated = false;
        self.healthy = false;
        true
    }
    fn invalidate(&mut self) {
        if let Some(next) = self.generation.checked_add(1) {
            self.generation = next;
        } else {
            self.state = State::Fault;
            self.reason = Reason::Generation;
        }
        self.controller = Controller::default();
        self.position_controller = position::Controller::EMPTY;
        self.leases = [None; 3];
        self.diagnostic = None;
        self.last_kind = None;
    }
    pub fn disarm(&mut self) {
        if self.state != State::Fault {
            self.state = State::Standby;
            self.reason = Reason::Disarmed;
        }
        self.invalidate();
    }
    pub fn fault(&mut self, reason: Reason) {
        self.state = State::Fault;
        self.reason = reason;
        self.healthy = false;
        self.invalidate();
    }
    pub fn clear_fault(&mut self) -> bool {
        if self.state != State::Fault || self.reason.latched() {
            return false;
        }
        self.state = State::Boot;
        self.reason = Reason::NotReady;
        self.previous = None;
        self.sample = None;
        self.estimate = Estimator::default();
        self.position = position::Estimate::EMPTY;
        self.healthy = false;
        self.calibrated = false;
        self.invalidate();
        true
    }
    /// Accept one external position measurement (world NWU, metres), optionally with a
    /// heading (world NWU yaw, radians), that arrived at local time `at`. Rejects non-finite
    /// or out-of-volume values and stale or reordered samples before any state changes
    /// (SWM-05). A heading corrects the attitude estimate's yaw. Does not grant authority.
    pub fn position(&mut self, measured: Vec3, yaw: Option<f32>, at: Tick) -> bool {
        let t = self.tuning.position;
        if matches!(self.state, State::Fault | State::Maintenance)
            || !t.volume.contains(measured, position::MEASUREMENT_MARGIN)
            || yaw.is_some_and(|y| !y.is_finite() || y.abs() > 2.0 * core::f32::consts::PI)
        {
            return false;
        }
        if let Some(now) = self.previous {
            if at < now || at.age(now).is_none_or(|age| age > self.limits.gap_us) {
                return false;
            }
        }
        if self.position.measured.is_some_and(|old| at <= old) {
            return false;
        }
        let mut estimate = self.estimate;
        if let Some(yaw) = yaw {
            let weight = if self.position.heading {
                t.heading_weight
            } else {
                1.0
            };
            if estimate.correct_heading(yaw, weight).is_none() {
                return false;
            }
        }
        let mut next = self.position;
        if next.correct(measured, at, t).is_none() {
            return false;
        }
        next.heading |= yaw.is_some();
        self.estimate = estimate;
        self.position = next;
        true
    }
    pub fn select(&mut self, source: Source) -> bool {
        // A source transition requires explicit rearming; never inherits another lease.
        if matches!(self.state, State::Armed | State::Maintenance | State::Fault) {
            return false;
        }
        if self.selected != Some(source) {
            self.invalidate();
            self.selected = Some(source);
        }
        true
    }
    pub fn command(&mut self, source: Source, intent: Intent, at: Tick) -> bool {
        if self.state == State::Fault
            || self.state == State::Maintenance
            || self.selected != Some(source)
            || !intent.valid()
        {
            return false;
        }
        if (source == Source::Automatic)
            != matches!(
                intent,
                Intent::Attitude { .. } | Intent::Rates { .. } | Intent::Position(_)
            )
        {
            return false;
        }
        if let Intent::Position(target) = intent {
            let t = self.tuning.position;
            if !t.volume.contains(target.position, 0.0) || target.velocity.norm() > t.max_speed {
                return false;
            }
        }
        if let Some(now) = self.previous {
            if at < now || at.age(now).is_none_or(|age| age > self.limits.gap_us) {
                return false;
            }
        }
        let index = source.index();
        if self.leases[index].is_some_and(|old| at < old.at) {
            return false;
        }
        // Armed takeoff into position control: only from an idle automatic lease, which
        // already holds the motors at idle (swarm spec SD-10).
        let takeoff = matches!(intent, Intent::Position(_))
            && self.leases[index].is_some_and(|old| {
                matches!(old.intent, Intent::Attitude { .. } | Intent::Rates { .. })
                    && old.intent.thrust() < 0.1
            });
        if takeoff && self.state == State::Armed {
            self.position_controller = position::Controller::EMPTY;
        } else if self.last_kind.is_some_and(|kind| kind != intent.kind()) {
            if self.state == State::Armed {
                self.disarm();
                return false;
            }
            self.invalidate();
        }
        self.last_kind = Some(intent.kind());
        self.leases[index] = Some(Lease { intent, at });
        true
    }
    fn lease(&self, now: Tick) -> Option<Lease> {
        let lease = self.leases[self.selected?.index()]?;
        (now.age(lease.at)? <= self.limits.command_us).then_some(lease)
    }
    /// The first unmet arming precondition at `now`, or `None` if `arm` would proceed.
    pub fn arm_blocker(&self, now: Tick) -> Option<ArmBlock> {
        let fresh = |t: Option<Tick>| {
            t.is_some_and(|t| now.age(t).is_some_and(|age| age <= self.limits.sensor_us))
        };
        if self.issuer == 0 {
            Some(ArmBlock::NoOwner)
        } else if self.state != State::Standby {
            Some(ArmBlock::State(self.state))
        } else if !self.healthy {
            Some(ArmBlock::Sensor)
        } else if !self.calibrated {
            Some(ArmBlock::Calibration)
        } else if !fresh(self.previous) || !fresh(self.sample) {
            Some(ArmBlock::SensorStale)
        } else {
            match self.lease(now) {
                None => Some(ArmBlock::NoCommand),
                // NaN thrust fails the comparison and therefore blocks arming.
                Some(l) if l.intent.thrust() < 0.05 => None,
                Some(_) => Some(ArmBlock::Thrust),
            }
        }
    }
    pub fn arm(&mut self, now: Tick) -> bool {
        if self.arm_blocker(now).is_some() {
            return false;
        }
        // Keep the just-validated lease; invalidate prior frames without erasing it.
        let Some(next) = self.generation.checked_add(1) else {
            self.fault(Reason::Generation);
            return false;
        };
        self.generation = next;
        self.controller = Controller::default();
        self.state = State::Armed;
        self.reason = Reason::None;
        true
    }
    pub fn maintenance(&mut self) -> bool {
        if self.state != State::Standby {
            return false;
        }
        self.invalidate();
        self.state = State::Maintenance;
        self.reason = Reason::None;
        true
    }
    pub fn diagnostic(&mut self, values: [f32; 4], now: Tick, duration_us: u64) -> bool {
        if self.issuer == 0
            || self.state != State::Maintenance
            || duration_us == 0
            || duration_us > 100_000
            || !values
                .iter()
                .all(|v| v.is_finite() && (0.0..=0.15).contains(v))
        {
            return false;
        }
        let Some(expires) = now.after(duration_us) else {
            return false;
        };
        self.invalidate();
        if self.state != State::Maintenance {
            return false;
        }
        self.diagnostic = Some((values, now, expires));
        true
    }
    pub fn tick(&mut self, now: Tick, sensor: Option<Sensor>) -> MotorFrame {
        let mut values = [0.0; 4];
        let elapsed = self.previous.and_then(|old| now.age(old));
        if self.previous.is_some() && elapsed.is_none_or(|age| age == 0 || age > self.limits.gap_us)
        {
            self.fault(Reason::Clock);
        }
        self.previous = Some(now);
        if self.state != State::Fault {
            let sample_ok = sensor.is_some_and(|s| {
                now.age(s.at)
                    .is_some_and(|age| age <= self.limits.sensor_us)
                    && self.sample.is_none_or(|old| s.at > old)
            });
            if !sample_ok {
                self.fault(Reason::SensorExpired);
            } else if let Some(s) = sensor {
                let dt = self.sample.and_then(|old| s.at.age(old)).unwrap_or(0) as f32 * 1e-6;
                let thrust = self.lease(now).map_or(0.0, |l| l.intent.thrust());
                if dt > self.limits.gap_us as f32 * 1e-6
                    || self
                        .estimate
                        .update(
                            s.gyro,
                            s.acceleration,
                            dt,
                            self.state == State::Armed || self.diagnostic.is_some(),
                            thrust,
                            self.tuning,
                        )
                        .is_none()
                    || self
                        .position
                        .predict(self.estimate.attitude, s.acceleration, dt)
                        .is_none()
                {
                    self.fault(Reason::InvalidNumeric);
                } else {
                    self.sample = Some(s.at);
                    self.healthy = true;
                    self.calibrated = s.calibrated;
                    if self.state == State::Boot {
                        self.state = State::Standby;
                        self.reason = Reason::Disarmed;
                    }
                }
            }
        }
        if self.state == State::Armed {
            if !self.calibrated {
                self.fault(Reason::NotReady);
            } else if let Some(l) = self.lease(now) {
                let dt = elapsed.unwrap_or(0) as f32 * 1e-6;
                let intent = match l.intent {
                    // Position control flies only on a fresh external position (SWM-07).
                    Intent::Position(target) if !self.position.fresh(now, self.tuning.position) => {
                        Err(Reason::PositionExpired)
                    }
                    Intent::Position(target) => self
                        .position_controller
                        .update(target, self.position, dt, self.tuning.position)
                        .map(|(orientation, thrust)| Intent::Attitude {
                            orientation,
                            feedforward: Vec3::ZERO,
                            thrust,
                        })
                        .ok_or(Reason::InvalidNumeric),
                    other => Ok(other),
                };
                if let Err(reason) = intent {
                    self.fault(reason);
                } else if matches!(l.intent, Intent::Stabilized(_) | Intent::Position(_))
                    && libm::acosf(self.estimate.attitude.body(Vec3::UP).2.clamp(-1.0, 1.0))
                        > self.limits.max_tilt
                {
                    self.fault(Reason::Tilt);
                } else if let Some(m) = intent.ok().and_then(|intent| {
                    self.controller
                        .update(intent, self.estimate, dt, self.tuning)
                }) {
                    values = m;
                } else {
                    self.fault(Reason::InvalidNumeric);
                }
            } else {
                self.state = State::Failsafe;
                self.reason = Reason::CommandExpired;
                self.invalidate();
            }
        } else if self.state == State::Maintenance {
            if let Some((m, start, expiry)) = self.diagnostic {
                // A clock that steps back (possible after reset) must not run it early.
                if start <= now && now < expiry {
                    values = m;
                } else {
                    self.reason = Reason::MaintenanceExpired;
                    self.diagnostic = None;
                }
            }
        }
        // Final dominance check, including faults produced after controller execution.
        if !matches!(self.state, State::Armed | State::Maintenance) {
            values = [0.0; 4];
        }
        let expires = now.after(self.limits.frame_us).unwrap_or(now);
        MotorFrame {
            issuer: self.issuer,
            values,
            generation: self.generation,
            issued: now,
            expires,
        }
    }
    /// Call immediately before physical commit. A driver must also independently expire output.
    pub fn commit(&self, frame: MotorFrame, now: Tick) -> [f32; 4] {
        if self.issuer == 0
            || frame.issuer != self.issuer
            || frame.generation != self.generation
            || now < frame.issued
            || now >= frame.expires
            || !matches!(self.state, State::Armed | State::Maintenance)
            || !frame
                .values
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        {
            return [0.0; 4];
        }
        if self.state == State::Armed
            && (self.lease(now).is_none()
                || self
                    .sample
                    .is_none_or(|t| now.age(t).is_none_or(|age| age > self.limits.sensor_us)))
        {
            return [0.0; 4];
        }
        if self.state == State::Maintenance
            && self
                .diagnostic
                .is_none_or(|(_, start, expiry)| now < start || now >= expiry)
        {
            return [0.0; 4];
        }
        frame.values
    }
}
