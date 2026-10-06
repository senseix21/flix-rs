//! FLU body axes; quaternion maps body to world, inverse rotation maps world to body.
use core::ops::{Add, Mul, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3(pub f32, pub f32, pub f32);
impl Vec3 {
    pub const ZERO: Self = Self(0.0, 0.0, 0.0);
    pub const UP: Self = Self(0.0, 0.0, 1.0);
    pub fn finite(self) -> bool {
        self.0.is_finite() && self.1.is_finite() && self.2.is_finite()
    }
    pub fn dot(self, b: Self) -> f32 {
        self.0 * b.0 + self.1 * b.1 + self.2 * b.2
    }
    pub fn cross(self, b: Self) -> Self {
        Self(
            self.1 * b.2 - self.2 * b.1,
            self.2 * b.0 - self.0 * b.2,
            self.0 * b.1 - self.1 * b.0,
        )
    }
    pub fn norm(self) -> f32 {
        libm::sqrtf(self.dot(self))
    }
    pub fn unit(self) -> Option<Self> {
        let n = self.norm();
        (self.finite() && n.is_finite() && n > 1e-6).then(|| self * (1.0 / n))
    }
    pub fn between(a: Self, b: Self) -> Option<Self> {
        let a = a.unit()?;
        let b = b.unit()?;
        let axis = a.cross(b);
        let n = axis.norm();
        let dot = a.dot(b).clamp(-1.0, 1.0);
        if n < 1e-6 {
            if dot > 0.0 {
                return Some(Self::ZERO);
            }
            // Deterministic antiparallel axis, rather than division by zero.
            let basis = if a.0.abs() < 0.9 {
                Self(1.0, 0.0, 0.0)
            } else {
                Self(0.0, 1.0, 0.0)
            };
            return Some(a.cross(basis).unit()? * core::f32::consts::PI);
        }
        Some(axis * (libm::atan2f(n, dot) / n))
    }
    pub fn array(self) -> [f32; 3] {
        [self.0, self.1, self.2]
    }
}
impl Add for Vec3 {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self(self.0 + b.0, self.1 + b.1, self.2 + b.2)
    }
}
impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self(self.0 - b.0, self.1 - b.1, self.2 - b.2)
    }
}
impl Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, b: f32) -> Self {
        Self(self.0 * b, self.1 * b, self.2 * b)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion([f32; 4]);
impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}
impl Quaternion {
    pub const IDENTITY: Self = Self([1.0, 0.0, 0.0, 0.0]);
    pub fn new(v: [f32; 4]) -> Option<Self> {
        if !v.iter().all(|x| x.is_finite()) {
            return None;
        }
        let n = libm::sqrtf(v.iter().map(|x| x * x).sum());
        if !n.is_finite() || n < 1e-6 {
            return None;
        }
        Some(Self(v.map(|x| x / n)))
    }
    pub fn array(self) -> [f32; 4] {
        self.0
    }
    pub fn rotation(v: Vec3) -> Option<Self> {
        if !v.finite() {
            return None;
        }
        let angle = v.norm();
        if angle < 1e-6 {
            return Self::new([1.0, v.0 * 0.5, v.1 * 0.5, v.2 * 0.5]);
        }
        let s = libm::sinf(angle * 0.5) / angle;
        Self::new([libm::cosf(angle * 0.5), v.0 * s, v.1 * s, v.2 * s])
    }
    pub fn euler(v: Vec3) -> Option<Self> {
        let (sr, cr) = (libm::sinf(v.0 / 2.0), libm::cosf(v.0 / 2.0));
        let (sp, cp) = (libm::sinf(v.1 / 2.0), libm::cosf(v.1 / 2.0));
        let (sy, cy) = (libm::sinf(v.2 / 2.0), libm::cosf(v.2 / 2.0));
        Self::new([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ])
    }
    pub fn compose(self, b: Self) -> Option<Self> {
        let [w, x, y, z] = self.0;
        let [a, i, j, k] = b.0;
        Self::new([
            w * a - x * i - y * j - z * k,
            w * i + x * a + y * k - z * j,
            w * j - x * k + y * a + z * i,
            w * k + x * j - y * i + z * a,
        ])
    }
    pub fn body(self, world: Vec3) -> Vec3 {
        let [w, x, y, z] = self.0;
        let v = Vec3(-x, -y, -z);
        let t = v.cross(world) * 2.0;
        world + t * w + v.cross(t)
    }
    pub fn angles(self) -> Vec3 {
        let [w, x, y, z] = self.0;
        Vec3(
            libm::atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            libm::asinf((2.0 * (w * y - z * x)).clamp(-1.0, 1.0)),
            libm::atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
        )
    }
}
pub fn wrap_angle(v: f32) -> f32 {
    libm::atan2f(libm::sinf(v), libm::cosf(v))
}
