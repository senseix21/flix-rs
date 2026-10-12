//! External position: validated samples, a position/velocity estimate and a position controller
//! (swarm spec S2, SWM-05 to SWM-10).
//!
//! World frame is NWU: x north, y west, z up, metres, the same world the attitude quaternion maps
//! body FLU vectors into. MAVLink NED conversion happens in `flight-protocol`. Freshness is local
//! arrival time; a sender timestamp never extends it.
use crate::control::GRAVITY;
use crate::{Quaternion, Tick, Vec3};

/// Axis-aligned flight volume. Setpoints must lie inside it; measurements may lie up to
/// `MEASUREMENT_MARGIN` outside it (noise near the floor), but no further.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Volume {
    pub min: Vec3,
    pub max: Vec3,
}
pub const MEASUREMENT_MARGIN: f32 = 0.5;
impl Volume {
    pub const DEFAULT: Self = Self {
        min: Vec3(-3.0, -3.0, 0.0),
        max: Vec3(3.0, 3.0, 2.0),
    };
    pub fn valid(self) -> bool {
        self.min.finite()
            && self.max.finite()
            && self.min.0 < self.max.0
            && self.min.1 < self.max.1
            && self.min.2 < self.max.2
            && self
                .min
                .array()
                .iter()
                .chain(self.max.array().iter())
                .all(|v| v.abs() <= 100.0)
    }
    pub fn contains(self, p: Vec3, margin: f32) -> bool {
        p.finite()
            && (self.min.0 - margin..=self.max.0 + margin).contains(&p.0)
            && (self.min.1 - margin..=self.max.1 + margin).contains(&p.1)
            && (self.min.2 - margin..=self.max.2 + margin).contains(&p.2)
    }
}

/// Built-in position settings. Not operator parameters yet: adding parameters changes the
/// persistent settings record and needs a schema version migration (swarm plan S2 follow-up).
#[derive(Clone, Copy, Debug)]
pub struct PositionTuning {
    pub volume: Volume,
    /// A position sample older than this (local arrival time) is stale.
    pub sample_us: u64,
    /// Estimator correction per measurement: position fraction and velocity gain.
    pub alpha: f32,
    pub beta: f32,
    pub kp: Vec3,
    pub kd: Vec3,
    pub ki: Vec3,
    /// Bound on each integral term's contribution, m/s^2.
    pub integral_limit: f32,
    pub max_tilt: f32,
    /// Collective thrust that balances gravity; the z integral absorbs the airframe's error.
    pub hover_thrust: f32,
    pub max_thrust: f32,
    pub max_speed: f32,
}
impl PositionTuning {
    pub const DEFAULT: Self = Self {
        volume: Volume::DEFAULT,
        sample_us: 200_000,
        alpha: 0.3,
        beta: 0.1,
        kp: Vec3(6.0, 6.0, 8.0),
        kd: Vec3(4.0, 4.0, 5.0),
        ki: Vec3(1.0, 1.0, 6.0),
        integral_limit: 2.0,
        max_tilt: 0.436_332_3, // 25 degrees
        hover_thrust: 0.5,
        max_thrust: 0.9,
        max_speed: 2.0,
    };
    pub fn valid(self) -> bool {
        let gains = [self.kp, self.kd, self.ki].iter().all(|v| {
            v.array()
                .iter()
                .all(|x| x.is_finite() && (0.0..=50.0).contains(x))
        });
        self.volume.valid()
            && gains
            && self.sample_us > 0
            && self.sample_us <= 1_000_000
            && [self.alpha, self.beta]
                .iter()
                .all(|x| x.is_finite() && *x > 0.0 && *x < 1.0)
            && self.integral_limit.is_finite()
            && (0.0..=GRAVITY).contains(&self.integral_limit)
            && self.max_tilt.is_finite()
            && self.max_tilt > 0.0
            && self.max_tilt < core::f32::consts::FRAC_PI_2
            && self.hover_thrust.is_finite()
            && self.hover_thrust > 0.05
            && self.hover_thrust < self.max_thrust
            && self.max_thrust.is_finite()
            && self.max_thrust <= 1.0
            && self.max_speed.is_finite()
            && self.max_speed > 0.0
            && self.max_speed <= 10.0
    }
}
impl Default for PositionTuning {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// A position target. Velocity is an optional feedforward (zero when the sender omits it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Target {
    pub position: Vec3,
    pub velocity: Vec3,
    pub yaw: f32,
}
impl Target {
    /// Value checks that need no configuration; the volume is checked by `Flight::command`.
    pub fn valid(self) -> bool {
        self.position.finite()
            && self.velocity.finite()
            && self.velocity.array().iter().all(|v| v.abs() <= 10.0)
            && self.yaw.is_finite()
            && self.yaw.abs() <= 2.0 * core::f32::consts::PI
    }
}

/// Position and velocity in the world frame: IMU prediction, external position correction.
#[derive(Clone, Copy, Debug, Default)]
pub struct Estimate {
    pub position: Vec3,
    pub velocity: Vec3,
    /// Local arrival time of the last accepted measurement; `None` until the first one.
    pub measured: Option<Tick>,
}
impl Estimate {
    pub const EMPTY: Self = Self {
        position: Vec3::ZERO,
        velocity: Vec3::ZERO,
        measured: None,
    };
    /// Integrate one IMU sample: body specific force rotated to world, minus gravity.
    pub fn predict(&mut self, attitude: Quaternion, specific_force: Vec3, dt: f32) -> Option<()> {
        if self.measured.is_none() {
            return Some(());
        }
        let acceleration = attitude.world(specific_force) - Vec3::UP * GRAVITY;
        let velocity = self.velocity + acceleration * dt;
        let position = self.position + velocity * dt;
        (velocity.finite() && position.finite()).then(|| {
            self.velocity = velocity;
            self.position = position;
        })
    }
    /// Apply a measurement that the caller already validated and ordered.
    pub fn correct(&mut self, measured: Vec3, at: Tick, t: PositionTuning) -> Option<()> {
        let Some(previous) = self.measured else {
            self.position = measured;
            self.velocity = Vec3::ZERO;
            self.measured = Some(at);
            return Some(());
        };
        let elapsed = at.age(previous)?;
        if elapsed == 0 || elapsed > t.sample_us {
            // A gap longer than the freshness limit restarts the estimate from the measurement.
            self.position = measured;
            self.velocity = Vec3::ZERO;
            self.measured = Some(at);
            return Some(());
        }
        #[expect(
            clippy::cast_precision_loss,
            reason = "elapsed is at most sample_us <= 1 s in microseconds"
        )]
        let period = elapsed as f32 * 1e-6;
        let error = measured - self.position;
        let position = self.position + error * t.alpha;
        let velocity = self.velocity + error * (t.beta / period);
        if !position.finite() || !velocity.finite() {
            return None;
        }
        self.position = position;
        self.velocity = velocity;
        self.measured = Some(at);
        Some(())
    }
    pub fn fresh(&self, now: Tick, t: PositionTuning) -> bool {
        self.measured
            .is_some_and(|at| now.age(at).is_some_and(|age| age <= t.sample_us))
    }
}

/// Position loop: target and estimate to an attitude and a collective thrust for the existing
/// attitude controller. Output tilt and thrust are bounded by the tuning.
#[derive(Clone, Copy, Debug, Default)]
pub struct Controller {
    integral: Vec3,
}
impl Controller {
    pub const EMPTY: Self = Self {
        integral: Vec3::ZERO,
    };
    pub fn update(
        &mut self,
        target: Target,
        estimate: Estimate,
        dt: f32,
        t: PositionTuning,
    ) -> Option<(Quaternion, f32)> {
        if !target.valid() || !estimate.position.finite() || !estimate.velocity.finite() {
            return None;
        }
        let error = target.position - estimate.position;
        let velocity_error = target.velocity - estimate.velocity;
        // Integrate only once airborne, where the integral corrects hover thrust and drift.
        if estimate.position.2 > t.volume.min.2 + 0.05 && dt > 0.0 && dt < 0.1 {
            let mut next = self.integral + error * dt;
            for (value, ki) in [
                (&mut next.0, t.ki.0),
                (&mut next.1, t.ki.1),
                (&mut next.2, t.ki.2),
            ] {
                let bound = if ki > 0.0 { t.integral_limit / ki } else { 0.0 };
                *value = value.clamp(-bound, bound);
            }
            self.integral = next;
        }
        let axis = |e: f32, v: f32, i: f32, kp: f32, kd: f32, ki: f32| kp * e + kd * v + ki * i;
        let demand = Vec3(
            axis(
                error.0,
                velocity_error.0,
                self.integral.0,
                t.kp.0,
                t.kd.0,
                t.ki.0,
            ),
            axis(
                error.1,
                velocity_error.1,
                self.integral.1,
                t.kp.1,
                t.kd.1,
                t.ki.1,
            ),
            axis(
                error.2,
                velocity_error.2,
                self.integral.2,
                t.kp.2,
                t.kd.2,
                t.ki.2,
            ),
        );
        let vertical = (demand.2 + GRAVITY).max(0.2 * GRAVITY);
        let horizontal = libm::sqrtf(demand.0 * demand.0 + demand.1 * demand.1);
        let limit = vertical * libm::tanf(t.max_tilt);
        let scale = if horizontal > limit {
            limit / horizontal
        } else {
            1.0
        };
        let force = Vec3(demand.0 * scale, demand.1 * scale, vertical);
        let thrust = (t.hover_thrust * force.norm() / GRAVITY).clamp(0.0, t.max_thrust);
        let attitude = attitude_for(force, target.yaw)?;
        thrust.is_finite().then_some((attitude, thrust))
    }
}

/// Attitude whose body z points along `force` and whose body x faces `yaw` (world NWU).
pub fn attitude_for(force: Vec3, yaw: f32) -> Option<Quaternion> {
    let z = force.unit()?;
    let y = z
        .cross(Vec3(libm::cosf(yaw), libm::sinf(yaw), 0.0))
        .unit()?;
    let x = y.cross(z);
    Quaternion::from_axes(x, y, z)
}
