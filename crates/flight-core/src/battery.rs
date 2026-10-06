//! Opt-in bench power policy. Hardware supplies calibrated pack voltage/time.
use crate::Tick;

#[derive(Clone, Copy, Debug)]
pub enum Reading {
    Unavailable,
    Invalid,
    Voltage { volts: f32, at: Tick },
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Health {
    Healthy,
    Warning,
    PendingCritical,
    Critical,
    Unavailable,
    Invalid,
    Stale,
}
impl Health {
    pub fn permits_arm(self) -> bool {
        self == Self::Healthy
    }
    pub fn requires_stop(self) -> bool {
        matches!(
            self,
            Self::Critical | Self::Unavailable | Self::Invalid | Self::Stale
        )
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Config {
    cells: u8,
    critical: f32,
    warning: f32,
    maximum: f32,
    hysteresis: f32,
    age_us: u64,
    debounce_us: u64,
}
impl Config {
    /// Thresholds are per cell. No built-in flight battery defaults.
    pub fn new(
        cells: u8,
        critical: f32,
        warning: f32,
        maximum: f32,
        hysteresis: f32,
        age_us: u64,
        debounce_us: u64,
    ) -> Option<Self> {
        (cells > 0
            && cells <= 10
            && [critical, warning, maximum, hysteresis]
                .iter()
                .all(|v| v.is_finite())
            && critical > 0.0
            && critical < warning
            && hysteresis >= 0.0
            && warning + hysteresis < maximum
            && maximum <= 10.0
            && age_us > 0
            && age_us <= 1_000_000
            && debounce_us <= 1_000_000)
            .then_some(Self {
                cells,
                critical,
                warning,
                maximum,
                hysteresis,
                age_us,
                debounce_us,
            })
    }
    pub fn cells(self) -> u8 {
        self.cells
    }
}
pub struct Guard {
    config: Config,
    low_since: Option<Tick>,
    last_sample: Option<Tick>,
    warning: bool,
}
impl Guard {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            low_since: None,
            last_sample: None,
            warning: false,
        }
    }
    pub fn config(&self) -> Config {
        self.config
    }
    pub fn reset(&mut self) {
        self.low_since = None;
        self.last_sample = None;
        self.warning = false;
    }
    pub fn evaluate(&mut self, reading: Reading, now: Tick) -> Health {
        let (volts, at) = match reading {
            Reading::Unavailable => {
                self.low_since = None;
                return Health::Unavailable;
            }
            Reading::Invalid => {
                self.low_since = None;
                return Health::Invalid;
            }
            Reading::Voltage { volts, at } => (volts, at),
        };
        if !volts.is_finite()
            || volts <= 0.0
            || volts > self.config.maximum * f32::from(self.config.cells)
            || self.last_sample.is_some_and(|old| at < old)
        {
            self.low_since = None;
            return Health::Invalid;
        }
        if now.age(at).is_none_or(|age| age > self.config.age_us) {
            self.low_since = None;
            return Health::Stale;
        }
        self.last_sample = Some(at);
        let voltage = volts / f32::from(self.config.cells);
        if voltage <= self.config.critical {
            let since = *self.low_since.get_or_insert(now);
            return if now
                .age(since)
                .is_none_or(|age| age >= self.config.debounce_us)
            {
                Health::Critical
            } else {
                Health::PendingCritical
            };
        }
        self.low_since = None;
        if voltage <= self.config.warning {
            self.warning = true;
        } else if voltage >= self.config.warning + self.config.hysteresis {
            self.warning = false;
        }
        if self.warning {
            Health::Warning
        } else {
            Health::Healthy
        }
    }
}
