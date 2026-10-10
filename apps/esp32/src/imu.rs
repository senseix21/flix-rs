//! MPU6050 register protocol. Bus implementations must bound each transaction.
use flight_core::control::GRAVITY;
use flight_core::{Quaternion, Sensor, Tick, Vec3};
pub trait Bus {
    type Error;
    fn read(&mut self, register: u8, out: &mut [u8]) -> Result<(), Self::Error>;
    fn write(&mut self, register: u8, value: u8) -> Result<(), Self::Error>;
}
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    Identity,
    Readback,
    NotReady,
    Calibration,
}
pub struct Mpu6050<B> {
    bus: B,
    rotation: Quaternion,
    acc_bias: Vec3,
    acc_scale: Vec3,
    gyro_bias: Vec3,
    count: u32,
    sum: Vec3,
    start: Option<Tick>,
    calibrated: bool,
    saturation: u8,
}
/// Per-axis register saturation of the last sample, in the sensor frame: gyro x/y/z in
/// bits 0..=2 and accelerometer x/y/z in bits 3..=5 (the Gazebo trace layout). A
/// saturated axis read its register endpoint, so the true value may be larger.
pub const GYRO_SATURATED: u8 = 0b000_111;
pub const ACCEL_SATURATED: u8 = 0b111_000;
impl<B: Bus> Mpu6050<B> {
    pub fn new(
        bus: B,
        rotation: Quaternion,
        acc_bias: Vec3,
        acc_scale: Vec3,
    ) -> Result<Self, Error<B::Error>> {
        if !acc_bias.finite()
            || !acc_scale.finite()
            || acc_scale.array().iter().any(|x| *x < 0.1 || *x > 10.0)
        {
            return Err(Error::Calibration);
        }
        Ok(Self {
            bus,
            rotation,
            acc_bias,
            acc_scale,
            gyro_bias: Vec3::ZERO,
            count: 0,
            sum: Vec3::ZERO,
            start: None,
            calibrated: false,
            saturation: 0,
        })
    }
    pub fn configure(&mut self) -> Result<(), Error<B::Error>> {
        let mut id = [0];
        self.bus.read(0x75, &mut id).map_err(Error::Bus)?;
        if id[0] & 0x7e != 0x68 {
            return Err(Error::Identity);
        }
        // Wake PLL X gyro; DLPF=1 => 1kHz gyro sampling, divider=0; ±4g, ±2000dps.
        for (register, value) in [
            (0x6b, 1),
            (0x1a, 1),
            (0x19, 0),
            (0x1b, 24),
            (0x1c, 8),
            (0x38, 1),
        ] {
            self.bus.write(register, value).map_err(Error::Bus)?;
            let mut actual = [0];
            self.bus.read(register, &mut actual).map_err(Error::Bus)?;
            if actual[0] != value {
                return Err(Error::Readback);
            }
        }
        Ok(())
    }
    pub fn sample(&mut self, at: Tick, stopped: bool) -> Result<Sensor, Error<B::Error>> {
        let mut status = [0];
        self.bus.read(0x3a, &mut status).map_err(Error::Bus)?;
        if status[0] & 1 == 0 {
            return Err(Error::NotReady);
        }
        let mut raw = [0; 14];
        self.bus.read(0x3b, &mut raw).map_err(Error::Bus)?;
        let register = |i: usize| i16::from_be_bytes([raw[i], raw[i + 1]]);
        let at_limit = |i: usize| matches!(register(i), i16::MIN | i16::MAX);
        // Registers 0x3b..: accel x/y/z, temperature, gyro x/y/z (two bytes each).
        self.saturation = [8, 10, 12, 0, 2, 4]
            .into_iter()
            .enumerate()
            .fold(0, |mask, (bit, i)| mask | (u8::from(at_limit(i)) << bit));
        let signed = |i: usize| f32::from(register(i));
        let acceleration = Vec3(signed(0), signed(2), signed(4)) * (GRAVITY / 8192.0);
        let gyro =
            Vec3(signed(8), signed(10), signed(12)) * (core::f32::consts::PI / (180.0 * 16.4));
        if stopped && !self.calibrated {
            if gyro.norm() < 0.1 && (acceleration.norm() - GRAVITY).abs() < GRAVITY * 0.1 {
                let start = *self.start.get_or_insert(at);
                self.sum = self.sum + gyro;
                self.count = self.count.saturating_add(1);
                if at.age(start).is_some_and(|age| age >= 2_000_000) && self.count >= 1000 {
                    self.gyro_bias = self.sum * (1.0 / self.count as f32);
                    self.calibrated = true;
                }
            } else {
                self.start = None;
                self.sum = Vec3::ZERO;
                self.count = 0;
            }
        }
        let a = acceleration - self.acc_bias;
        let a = Vec3(
            a.0 / self.acc_scale.0,
            a.1 / self.acc_scale.1,
            a.2 / self.acc_scale.2,
        );
        // rotation is sensor -> body, therefore use its inverse for body() convention.
        let [w, x, y, z] = self.rotation.array();
        let inverse = Quaternion::new([w, -x, -y, -z]).ok_or(Error::Calibration)?;
        Ok(Sensor {
            at,
            gyro: inverse.body(gyro - self.gyro_bias),
            acceleration: inverse.body(a),
            calibrated: self.calibrated,
        })
    }
    /// Saturation mask of the most recent successful sample (see [`GYRO_SATURATED`]).
    pub fn saturation(&self) -> u8 {
        self.saturation
    }
    pub fn reset_calibration(&mut self) {
        self.calibrated = false;
        self.start = None;
        self.sum = Vec3::ZERO;
        self.count = 0;
        self.gyro_bias = Vec3::ZERO;
    }
}
