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
use flight_core::control::Tuning;
use flight_core::{Limits, Vec3};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Unknown,
    Unsupported,
    Bounds,
    CrossField,
    Storage,
    Missing,
    Busy,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Implemented,
    Deferred,
    Obsolete,
}
pub struct Parameter {
    pub name: &'static str,
    pub default: f32,
    pub min: f32,
    pub max: f32,
    pub integer: bool,
    pub status: Status,
}
include!("schema.rs");
pub const COUNT: usize = 96;
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    values: [f32; COUNT],
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            values: core::array::from_fn(|i| PARAMETERS[i].default),
        }
    }
}
impl Settings {
    pub fn from_values(values: [f32; COUNT]) -> Result<Self, Error> {
        let settings = Self { values };
        settings.validate()?;
        Ok(settings)
    }
    pub const DEFAULT: Self = Self {
        values: DEFAULT_VALUES,
    };
    pub fn bind_board(&mut self, pins: [i32; 7]) -> Result<(), Error> {
        let names = [
            "IMU_PIN_SDA",
            "IMU_PIN_SCL",
            "MOT_PIN_RL",
            "MOT_PIN_RR",
            "MOT_PIN_FR",
            "MOT_PIN_FL",
            "RC_RX_PIN",
        ];
        let mut next = *self;
        for (name, pin) in names.into_iter().zip(pins) {
            let i = Self::index(name).ok_or(Error::Unknown)?;
            if !Self::scalar(i, pin as f32) {
                return Err(Error::Bounds);
            }
            next.values[i] = pin as f32;
        }
        for name in [
            "MOT_PWM_FREQ",
            "MOT_PWM_RES",
            "MOT_PWM_STOP",
            "MOT_PWM_MIN",
            "MOT_PWM_MAX",
            "IMU_MODEL",
            "IMU_BUS",
        ] {
            let index = Self::index(name).ok_or(Error::Unknown)?;
            next.values[index] = PARAMETERS[index].default;
        }
        next.validate()?;
        *self = next;
        Ok(())
    }
    pub fn index(name: &str) -> Option<usize> {
        PARAMETERS
            .iter()
            .position(|p| p.name.eq_ignore_ascii_case(name))
    }
    pub fn get(&self, name: &str) -> Option<f32> {
        Self::index(name).map(|i| self.values[i])
    }
    pub fn at(&self, index: usize) -> Option<f32> {
        self.values.get(index).copied()
    }
    fn scalar(index: usize, value: f32) -> bool {
        let p = &PARAMETERS[index];
        value.is_finite()
            && (p.min..=p.max).contains(&value)
            && (!p.integer || value == value as i32 as f32)
    }
    pub fn set(&mut self, name: &str, value: f32) -> Result<(), Error> {
        let index = Self::index(name).ok_or(Error::Unknown)?;
        if PARAMETERS[index].status != Status::Implemented {
            return Err(Error::Unsupported);
        }
        if !Self::scalar(index, value) {
            return Err(Error::Bounds);
        }
        let mut candidate = *self;
        candidate.values[index] = value;
        candidate.validate()?;
        *self = candidate;
        Ok(())
    }
    pub fn validate(&self) -> Result<(), Error> {
        if self
            .values
            .iter()
            .enumerate()
            .any(|(i, v)| !Self::scalar(i, *v))
        {
            return Err(Error::Bounds);
        }
        let pins = [
            "MOT_PIN_RL",
            "MOT_PIN_RR",
            "MOT_PIN_FR",
            "MOT_PIN_FL",
            "IMU_PIN_SDA",
            "IMU_PIN_SCL",
            "RC_RX_PIN",
        ];
        for (i, name) in pins.iter().enumerate() {
            let pin = self.get(name).ok_or(Error::Unknown)?;
            if pin >= 0.0 {
                if [
                    0.0, 19.0, 20.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0,
                    32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 43.0, 44.0, 45.0, 46.0,
                ]
                .contains(&pin)
                {
                    return Err(Error::CrossField);
                }
                if pins[..i].iter().any(|p| self.get(p) == Some(pin)) {
                    return Err(Error::CrossField);
                }
            }
        }
        for i in 0..8 {
            if self.values[Self::index(RC_ZERO_NAMES[i]).ok_or(Error::Unknown)?]
                >= self.values[Self::index(RC_MAX_NAMES[i]).ok_or(Error::Unknown)?]
            {
                return Err(Error::CrossField);
            }
        }
        let resolution = self.get("MOT_PWM_RES").ok_or(Error::Unknown)? as u32;
        if resolution == 0 {
            return Err(Error::CrossField);
        }
        let max = (1u32 << resolution) - 1;
        let configured = self.get("MOT_PWM_MAX").ok_or(Error::Unknown)?;
        let maximum = if configured < 0.0 {
            max as f32
        } else {
            configured
        };
        let minimum = self.get("MOT_PWM_MIN").ok_or(Error::Unknown)?;
        let stop = self.get("MOT_PWM_STOP").ok_or(Error::Unknown)?;
        if maximum > max as f32
            || minimum > maximum
            || stop != 0.0
            || self.get("MOT_PWM_FREQ").ok_or(Error::Unknown)? < 1000.0
            || self.get("MOT_PWM_FREQ").ok_or(Error::Unknown)? * (1u32 << resolution) as f32
                > 80_000_000.0
        {
            return Err(Error::CrossField);
        }
        if !self.tuning().valid() {
            return Err(Error::CrossField);
        }
        Ok(())
    }
    #[expect(
        clippy::expect_used,
        reason = "names come from the generated schema; a missing key is a build defect"
    )]
    pub fn tuning(&self) -> Tuning {
        let g = |name: &str| {
            self.get(name)
                .expect("generated schema contains tuning keys")
        };
        Tuning {
            rate_p: Vec3(g("CTL_RATE_R_P"), g("CTL_RATE_P_P"), g("CTL_RATE_Y_P")),
            rate_i: Vec3(g("CTL_RATE_R_I"), g("CTL_RATE_P_I"), g("CTL_RATE_Y_I")),
            rate_d: Vec3(g("CTL_RATE_R_D"), g("CTL_RATE_P_D"), g("CTL_RATE_Y_D")),
            windup: Vec3(g("CTL_RATE_R_WU"), g("CTL_RATE_P_WU"), g("CTL_RATE_Y_WU")),
            derivative_alpha: Vec3(
                g("CTL_RATE_R_D_A"),
                g("CTL_RATE_P_D_A"),
                g("CTL_RATE_Y_D_A"),
            ),
            attitude_p: Vec3(g("CTL_ATT_R_P"), g("CTL_ATT_P_P"), g("CTL_ATT_Y_P")),
            max_rate: Vec3(
                g("CTL_RATE_R_MAX"),
                g("CTL_RATE_P_MAX"),
                g("CTL_RATE_Y_MAX"),
            ),
            tilt: g("CTL_ATT_MAX"),
            gyro_alpha: g("EST_RATES_LPF_A"),
            acc_weight: g("EST_ACC_WEIGHT"),
            level_weight: g("EST_LVL_WEIGHT"),
        }
    }
    #[expect(
        clippy::expect_used,
        reason = "names come from the generated schema; a missing key is a build defect"
    )]
    pub fn limits(&self) -> Limits {
        Limits {
            command_us: (self.get("SF_RC_LOSS_TIME").expect("schema") * 1e6) as u64,
            max_tilt: self.get("SF_DISARM_TILT").expect("schema"),
            ..Limits::default()
        }
    }
}
const RC_ZERO_NAMES: [&str; 8] = [
    "RC_ZERO_0",
    "RC_ZERO_1",
    "RC_ZERO_2",
    "RC_ZERO_3",
    "RC_ZERO_4",
    "RC_ZERO_5",
    "RC_ZERO_6",
    "RC_ZERO_7",
];
const RC_MAX_NAMES: [&str; 8] = [
    "RC_MAX_0", "RC_MAX_1", "RC_MAX_2", "RC_MAX_3", "RC_MAX_4", "RC_MAX_5", "RC_MAX_6", "RC_MAX_7",
];
pub const RECORD_SIZE: usize = 16 + COUNT * 4 + 4;
pub fn checksum(bytes: &[u8]) -> u32 {
    let mut c = !0u32;
    for b in bytes {
        c ^= u32::from(*b);
        for _ in 0..8 {
            c = (c >> 1) ^ if c & 1 != 0 { 0xedb88320 } else { 0 };
        }
    }
    !c
}
pub fn encode(settings: &Settings, sequence: u64) -> Result<[u8; RECORD_SIZE], Error> {
    settings.validate()?;
    let mut b = [0; RECORD_SIZE];
    b[..4].copy_from_slice(b"FLXC");
    b[4..6].copy_from_slice(&1u16.to_le_bytes());
    b[6..8].copy_from_slice(&(COUNT as u16).to_le_bytes());
    b[8..16].copy_from_slice(&sequence.to_le_bytes());
    for (i, v) in settings.values.iter().enumerate() {
        b[16 + i * 4..20 + i * 4].copy_from_slice(&v.to_le_bytes());
    }
    let c = checksum(&b[..RECORD_SIZE - 4]);
    b[RECORD_SIZE - 4..].copy_from_slice(&c.to_le_bytes());
    Ok(b)
}
pub fn decode(bytes: &[u8]) -> Result<(Settings, u64), Error> {
    if bytes.len() != RECORD_SIZE
        || &bytes[..4] != b"FLXC"
        || bytes[4..8] != [1, 0, 96, 0]
        || checksum(&bytes[..RECORD_SIZE - 4])
            != u32::from_le_bytes(
                bytes[RECORD_SIZE - 4..]
                    .try_into()
                    .map_err(|_| Error::Storage)?,
            )
    {
        return Err(Error::Storage);
    }
    let values = core::array::from_fn(|i| {
        let at = 16 + i * 4;
        f32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
    });
    let s = Settings { values };
    s.validate()?;
    Ok((
        s,
        u64::from_le_bytes(bytes[8..16].try_into().map_err(|_| Error::Storage)?),
    ))
}
pub fn recover(a: &[u8], b: &[u8]) -> Result<(Settings, u64, usize), Error> {
    match (decode(a), decode(b)) {
        (Ok((a, sa)), Ok((b, sb))) => {
            if sa >= sb {
                Ok((a, sa, 0))
            } else {
                Ok((b, sb, 1))
            }
        }
        (Ok((a, s)), Err(_)) => Ok((a, s, 0)),
        (Err(_), Ok((b, s))) => Ok((b, s, 1)),
        _ => Err(Error::Storage),
    }
}
pub trait Storage {
    fn read(&mut self, slot: usize, bytes: &mut [u8; RECORD_SIZE]) -> Result<(), Error>;
    fn write(&mut self, slot: usize, bytes: &[u8; RECORD_SIZE]) -> Result<(), Error>;
}
pub fn save(
    storage: &mut impl Storage,
    settings: &Settings,
    flight: &flight_core::Flight,
) -> Result<u64, Error> {
    if !flight.stopped_maintenance() {
        return Err(Error::Busy);
    }
    let mut a = [0; RECORD_SIZE];
    let mut b = [0; RECORD_SIZE];
    let read_a = storage.read(0, &mut a);
    let read_b = storage.read(1, &mut b);
    if read_a.is_err() && read_a != Err(Error::Missing)
        || read_b.is_err() && read_b != Err(Error::Missing)
    {
        return Err(Error::Storage);
    }
    let (sequence, active) = match recover(&a, &b) {
        Ok((_, s, i)) => (s, i),
        Err(_) if read_a == Err(Error::Missing) && read_b == Err(Error::Missing) => (0, 1),
        Err(_) => return Err(Error::Storage),
    };
    let next = sequence.checked_add(1).ok_or(Error::Storage)?;
    let record = encode(settings, next)?;
    storage.write(1 - active, &record)?;
    let mut verify = [0; RECORD_SIZE];
    storage.read(1 - active, &mut verify)?;
    if verify != record {
        return Err(Error::Storage);
    }
    Ok(next)
}
