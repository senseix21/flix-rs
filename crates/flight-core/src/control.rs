use crate::math::wrap_angle;
use crate::position::{PositionTuning, Target};
use crate::{Quaternion, Vec3};
pub const GRAVITY: f32 = 9.80665;

#[derive(Clone, Copy, Debug)]
pub struct Tuning {
    pub rate_p: Vec3,
    pub rate_i: Vec3,
    pub rate_d: Vec3,
    pub windup: Vec3,
    pub derivative_alpha: Vec3,
    pub attitude_p: Vec3,
    pub max_rate: Vec3,
    pub tilt: f32,
    pub gyro_alpha: f32,
    pub acc_weight: f32,
    pub level_weight: f32,
    /// External position estimation and position control (swarm S2).
    pub position: PositionTuning,
}
impl Default for Tuning {
    fn default() -> Self {
        Self::DEFAULT
    }
}
impl Tuning {
    pub const DEFAULT: Self = Self {
        rate_p: Vec3(0.05, 0.05, 0.3),
        rate_i: Vec3(0.2, 0.2, 0.0),
        rate_d: Vec3(0.001, 0.001, 0.0),
        windup: Vec3(0.3, 0.3, 0.3),
        derivative_alpha: Vec3(0.2, 0.2, 1.0),
        attitude_p: Vec3(6.0, 6.0, 3.0),
        max_rate: Vec3(
            2.0 * core::f32::consts::PI,
            2.0 * core::f32::consts::PI,
            2.0 * core::f32::consts::PI,
        ),
        tilt: core::f32::consts::PI / 6.0,
        gyro_alpha: 0.2,
        acc_weight: 0.003,
        level_weight: 0.0002,
        position: PositionTuning::DEFAULT,
    };
}
impl Tuning {
    pub fn valid(self) -> bool {
        [
            self.rate_p,
            self.rate_i,
            self.rate_d,
            self.windup,
            self.attitude_p,
            self.max_rate,
        ]
        .iter()
        .all(|v| {
            v.array()
                .iter()
                .all(|x| x.is_finite() && *x >= 0.0 && *x <= 100.0)
        }) && self
            .derivative_alpha
            .array()
            .iter()
            .all(|x| x.is_finite() && (0.0..=1.0).contains(x))
            && [self.gyro_alpha, self.acc_weight, self.level_weight]
                .iter()
                .all(|x| x.is_finite() && (0.0..=1.0).contains(x))
            && self.tilt.is_finite()
            && (0.0..=core::f32::consts::FRAC_PI_2).contains(&self.tilt)
            && self.position.valid()
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Pid {
    integral: f32,
    previous: Option<f32>,
    derivative: f32,
}
#[derive(Clone, Copy, Debug)]
pub struct PidGains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub limit: f32,
    pub alpha: f32,
}
impl Pid {
    pub fn update(&mut self, error: f32, dt: f32, gains: PidGains) -> f32 {
        let PidGains {
            p,
            i,
            d,
            limit,
            alpha,
        } = gains;
        if let Some(old) = self.previous {
            if dt > 0.0 && dt < 0.1 {
                // Bound stored integral, unlike the legacy output-only clamp.
                self.integral = if i > 0.0 {
                    (self.integral + error * dt).clamp(-limit / i, limit / i)
                } else {
                    0.0
                };
                self.derivative += alpha * ((error - old) / dt - self.derivative);
            } else {
                self.integral = 0.0;
                self.derivative = 0.0;
            }
        }
        self.previous = Some(error);
        p * error + i * self.integral + d * self.derivative
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Estimator {
    pub attitude: Quaternion,
    pub rates: Vec3,
    initialized: bool,
}
impl Estimator {
    pub const EMPTY: Self = Self {
        attitude: Quaternion::IDENTITY,
        rates: Vec3::ZERO,
        initialized: false,
    };
    pub fn update(
        &mut self,
        gyro: Vec3,
        acc: Vec3,
        dt: f32,
        active: bool,
        thrust: f32,
        t: Tuning,
    ) -> Option<()> {
        if !gyro.finite() || !acc.finite() || gyro.norm() > 40.0 || acc.norm() > 160.0 {
            return None;
        }
        if self.initialized {
            self.rates = self.rates + (gyro - self.rates) * t.gyro_alpha;
        } else {
            self.rates = gyro;
            self.initialized = true;
        }
        self.attitude = self
            .attitude
            .compose(Quaternion::rotation(self.rates * dt)?)?;
        let up = self.attitude.body(Vec3::UP);
        let landed = !active && (acc.norm() - GRAVITY).abs() < GRAVITY * 0.1;
        let correction = if landed {
            Vec3::between(acc, up)? * t.acc_weight
        } else if thrust >= 0.1 {
            Vec3::between(Vec3::UP, up)? * t.level_weight
        } else {
            Vec3::ZERO
        };
        self.attitude = self.attitude.compose(Quaternion::rotation(correction)?)?;
        Some(())
    }
    /// Turn the attitude about world up by `weight` of the error to an external heading
    /// (world NWU yaw, radians). Roll and pitch are unchanged.
    pub fn correct_heading(&mut self, yaw: f32, weight: f32) -> Option<()> {
        if !yaw.is_finite() || !(0.0..=1.0).contains(&weight) {
            return None;
        }
        let error = wrap_angle(yaw - self.attitude.angles().2);
        self.attitude =
            Quaternion::rotation(Vec3::UP * (error * weight))?.compose(self.attitude)?;
        Some(())
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Pilot {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub thrust: f32,
}
impl Pilot {
    pub fn valid(self) -> bool {
        [self.roll, self.pitch, self.yaw]
            .iter()
            .all(|v| v.is_finite() && (-1.0..=1.0).contains(v))
            && self.thrust.is_finite()
            && (0.0..=1.0).contains(&self.thrust)
    }
}
#[derive(Clone, Copy, Debug)]
pub enum Intent {
    Stabilized(Pilot),
    Acro(Pilot),
    Attitude {
        orientation: Quaternion,
        feedforward: Vec3,
        thrust: f32,
    },
    Rates {
        rates: Vec3,
        thrust: f32,
    },
    /// Automatic only; `Flight` turns it into an attitude and thrust each tick (swarm S2).
    Position(Target),
}
impl Intent {
    /// Commanded collective thrust. A position intent chooses its own thrust, so it reports
    /// full scale: it can never satisfy the low-thrust arming precondition or idle the motors.
    pub fn thrust(self) -> f32 {
        match self {
            Self::Stabilized(p) | Self::Acro(p) => p.thrust,
            Self::Attitude { thrust, .. } | Self::Rates { thrust, .. } => thrust,
            Self::Position(_) => 1.0,
        }
    }
    pub fn valid(self) -> bool {
        match self {
            Self::Stabilized(p) | Self::Acro(p) => p.valid(),
            Self::Position(target) => target.valid(),
            Self::Attitude {
                feedforward,
                thrust,
                ..
            }
            | Self::Rates {
                rates: feedforward,
                thrust,
            } => {
                feedforward.finite()
                    && feedforward.array().iter().all(|v| v.abs() <= 40.0)
                    && thrust.is_finite()
                    && (0.0..=1.0).contains(&thrust)
            }
        }
    }
    pub fn kind(self) -> u8 {
        match self {
            Self::Stabilized(_) => 0,
            Self::Acro(_) => 1,
            Self::Attitude { .. } => 2,
            Self::Rates { .. } => 3,
            Self::Position(_) => 4,
        }
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Controller {
    pid: [Pid; 3],
    yaw: Option<f32>,
    pub target_rates: Vec3,
    pub target_attitude: Option<Quaternion>,
}
impl Controller {
    pub const EMPTY: Self = Self {
        pid: [Pid {
            integral: 0.0,
            previous: None,
            derivative: 0.0,
        }; 3],
        yaw: None,
        target_rates: Vec3::ZERO,
        target_attitude: None,
    };
    pub fn update(
        &mut self,
        intent: Intent,
        estimate: Estimator,
        dt: f32,
        t: Tuning,
    ) -> Option<[f32; 4]> {
        self.target_attitude = None;
        // Position intents are resolved by `Flight` before reaching the attitude controller.
        if !intent.valid() || matches!(intent, Intent::Position(_)) {
            return None;
        }
        if intent.thrust() < 0.1 {
            self.pid = [Pid::default(); 3];
            self.yaw = None;
            return Some([0.1; 4]);
        }
        let target = match intent {
            Intent::Acro(p) => Vec3(
                p.roll * t.max_rate.0,
                p.pitch * t.max_rate.1,
                -(if p.yaw.abs() < 0.1 { 0.0 } else { p.yaw }) * t.max_rate.2,
            ),
            Intent::Rates { rates, .. } => rates,
            Intent::Stabilized(p) => {
                let yaw = if p.yaw.abs() < 0.1 { 0.0 } else { p.yaw };
                if self.yaw.is_none() || yaw != 0.0 {
                    self.yaw = Some(estimate.attitude.angles().2);
                }
                self.attitude_rates(
                    Quaternion::euler(Vec3(p.roll * t.tilt, p.pitch * t.tilt, self.yaw?))?,
                    Vec3(0.0, 0.0, -yaw * t.max_rate.2),
                    estimate,
                    t,
                )?
            }
            Intent::Attitude {
                orientation,
                feedforward,
                ..
            } => self.attitude_rates(orientation, feedforward, estimate, t)?,
            Intent::Position(_) => return None,
        };
        self.target_rates = target;
        let error = (target - estimate.rates).array();
        let mut torque = [0.0; 3];
        for axis in 0..3 {
            torque[axis] = self.pid[axis].update(
                error[axis],
                dt,
                PidGains {
                    p: t.rate_p.array()[axis],
                    i: t.rate_i.array()[axis],
                    d: t.rate_d.array()[axis],
                    limit: t.windup.array()[axis],
                    alpha: t.derivative_alpha.array()[axis],
                },
            );
        }
        mix(intent.thrust(), Vec3(torque[0], torque[1], torque[2]))
    }
    fn attitude_rates(&mut self, q: Quaternion, ff: Vec3, e: Estimator, t: Tuning) -> Option<Vec3> {
        self.target_attitude = Some(q);
        let error = Vec3::between(q.body(Vec3::UP), e.attitude.body(Vec3::UP))?;
        Some(Vec3(
            error.0 * t.attitude_p.0 + ff.0,
            error.1 * t.attitude_p.1 + ff.1,
            wrap_angle(q.angles().2 - e.attitude.angles().2) * t.attitude_p.2 + ff.2,
        ))
    }
}
/// Rear-left, rear-right, front-right, front-left. Shift then clip like hardware.
pub fn mix(thrust: f32, torque: Vec3) -> Option<[f32; 4]> {
    if !thrust.is_finite() || !torque.finite() {
        return None;
    }
    let Vec3(x, y, z) = torque;
    let mut motors = [
        thrust + x + y - z,
        thrust - x + y + z,
        thrust - x - y - z,
        thrust + x - y + z,
    ];
    if !motors.iter().all(|v| v.is_finite()) {
        return None;
    }
    let max = motors.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    let min = motors.iter().copied().fold(f32::INFINITY, f32::min);
    let correction = if max > 1.0 {
        max - 1.0
    } else if min < 0.0 {
        min
    } else {
        0.0
    };
    for motor in &mut motors {
        *motor = (*motor - correction).clamp(0.0, 1.0);
    }
    Some(motors)
}
