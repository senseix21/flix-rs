use crate::control::Pilot;
#[derive(Clone, Copy, Debug)]
pub struct Sbus {
    pub channels: [u16; 16],
    pub lost: bool,
    pub failsafe: bool,
}
impl Sbus {
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 25
            || bytes[0] != 0x0f
            || ![0, 4, 0x14, 0x24, 0x34].contains(&bytes[24])
            || bytes[23] & 0xf0 != 0
        {
            return None;
        }
        let mut channels = [0u16; 16];
        for (channel, value) in channels.iter_mut().enumerate() {
            for bit in 0..11 {
                let position = channel * 11 + bit;
                *value |= u16::from((bytes[1 + position / 8] >> (position % 8)) & 1) << bit;
            }
        }
        Some(Self {
            channels,
            lost: bytes[23] & 4 != 0,
            failsafe: bytes[23] & 8 != 0,
        })
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    pub low: u16,
    pub center: u16,
    pub high: u16,
}
impl Calibration {
    pub fn valid(self) -> bool {
        self.low < self.center && self.center < self.high && self.high <= 2047
    }
    pub fn scale(self, value: u16, throttle: bool) -> Option<f32> {
        if !self.valid() || value < self.low || value > self.high {
            return None;
        }
        Some(if throttle {
            f32::from(value - self.low) / f32::from(self.high - self.low)
        } else if value >= self.center {
            f32::from(value - self.center) / f32::from(self.high - self.center)
        } else {
            -f32::from(self.center - value) / f32::from(self.center - self.low)
        })
    }
}
pub fn pilot(frame: Sbus, mapping: [usize; 4], calibration: [Calibration; 4]) -> Option<Pilot> {
    if frame.lost || frame.failsafe || mapping.iter().any(|i| *i >= 16) {
        return None;
    }
    for i in 0..4 {
        for j in i + 1..4 {
            if mapping[i] == mapping[j] {
                return None;
            }
        }
    }
    let p = Pilot {
        roll: calibration[0].scale(frame.channels[mapping[0]], false)?,
        pitch: calibration[1].scale(frame.channels[mapping[1]], false)?,
        yaw: calibration[2].scale(frame.channels[mapping[2]], false)?,
        thrust: calibration[3].scale(frame.channels[mapping[3]], true)?,
    };
    p.valid().then_some(p)
}
