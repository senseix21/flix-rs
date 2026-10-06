//! Fixed-memory measurements. Histogram counts are approximate, not raw trace proof.
pub const UPPER_US: [u64; 7] = [50, 100, 250, 500, 1000, 2000, 5000];

/// Frozen transfer; at most one 66-byte SERIAL_CONTROL data chunk per tick.
pub(crate) struct Transfer {
    values: [u64; 52],
    nonce: u32,
    index: u8,
}
impl Transfer {
    pub fn new(values: [u64; 52], nonce: u32) -> Self {
        Self {
            values,
            nonce,
            index: 0,
        }
    }
    pub fn next(&mut self) -> ([u8; 66], usize, bool) {
        let mut bytes = [0; 66];
        bytes[..4].copy_from_slice(b"FLXM");
        bytes[4..8].copy_from_slice(&self.nonce.to_le_bytes());
        bytes[8] = self.index;
        bytes[9] = 8;
        let start = usize::from(self.index) * 7;
        let count = (52 - start).min(7);
        for (i, value) in self.values[start..start + count].iter().enumerate() {
            bytes[10 + i * 8..18 + i * 8].copy_from_slice(&value.to_le_bytes());
        }
        self.index += 1;
        (bytes, 10 + count * 8, self.index == 8)
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Histogram {
    pub buckets: [u64; 8],
    pub maximum_us: u64,
    pub count: u64,
}
impl Default for Histogram {
    fn default() -> Self {
        Self::EMPTY
    }
}
impl Histogram {
    pub const EMPTY: Self = Self {
        buckets: [0; 8],
        maximum_us: 0,
        count: 0,
    };
    pub fn record(&mut self, us: u64) {
        let i = UPPER_US.iter().position(|upper| us <= *upper).unwrap_or(7);
        self.buckets[i] = self.buckets[i].saturating_add(1);
        self.count = self.count.saturating_add(1);
        self.maximum_us = self.maximum_us.max(us);
    }
}
#[derive(Default)]
pub struct Timing {
    pub acquisition: Histogram,
    pub sample_age: Histogram,
    pub core: Histogram,
    pub owner: Histogram,
    pub jitter: Histogram,
    pub invalid_sample_age: u64,
}
impl Timing {
    pub const EMPTY: Self = Self {
        acquisition: Histogram::EMPTY,
        sample_age: Histogram::EMPTY,
        core: Histogram::EMPTY,
        owner: Histogram::EMPTY,
        jitter: Histogram::EMPTY,
        invalid_sample_age: 0,
    };
    /// v1, then acquisition/sample-age/core/owner/jitter: count,max,8 buckets;
    /// final field counts impossible future sample timestamps. No pointers/secrets.
    pub fn snapshot(&self) -> [u64; 52] {
        let mut out = [0; 52];
        out[0] = 1;
        for (i, h) in [
            &self.acquisition,
            &self.sample_age,
            &self.core,
            &self.owner,
            &self.jitter,
        ]
        .into_iter()
        .enumerate()
        {
            let start = 1 + i * 10;
            out[start] = h.count;
            out[start + 1] = h.maximum_us;
            out[start + 2..start + 10].copy_from_slice(&h.buckets);
        }
        out[51] = self.invalid_sample_age;
        out
    }
}
