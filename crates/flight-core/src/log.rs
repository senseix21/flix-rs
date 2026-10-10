//! Heap-free ring storage. Snapshot is borrowed exclusively: acquisition cannot mutate it.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Entry {
    pub micros: u64,
    pub values: [f32; 14],
}
impl Default for Entry {
    fn default() -> Self {
        Self {
            micros: 0,
            values: [0.0; 14],
        }
    }
}
pub struct Log<const N: usize> {
    entries: [Entry; N],
    next: usize,
    len: usize,
}
impl<const N: usize> Default for Log<N> {
    fn default() -> Self {
        Self {
            entries: [Entry::default(); N],
            next: 0,
            len: 0,
        }
    }
}
impl<const N: usize> Log<N> {
    pub const EMPTY: Self = Self {
        entries: [Entry {
            micros: 0,
            values: [0.0; 14],
        }; N],
        next: 0,
        len: 0,
    };
    pub fn push(&mut self, entry: Entry) -> bool {
        if N == 0 || !entry.values.iter().all(|v| v.is_finite()) {
            return false;
        }
        self.entries[self.next] = entry;
        self.next = (self.next + 1) % N;
        self.len = (self.len + 1).min(N);
        true
    }
    pub fn len(&self) -> usize {
        self.len
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub fn get(&self, index: usize) -> Option<Entry> {
        if index >= self.len || N == 0 {
            return None;
        }
        let start = if self.len == N { self.next } else { 0 };
        Some(self.entries[(start + index) % N])
    }
    #[expect(
        clippy::expect_used,
        reason = "offset < total = 12 + len * 64, so every record index is < len"
    )]
    #[expect(
        clippy::cast_possible_truncation,
        reason = "len <= N, a small compile-time ring capacity"
    )]
    pub fn encode(&self, offset: usize, out: &mut [u8]) -> usize {
        // Format: FLXR, version u16=1, record size u16=64, count u32, then LE records.
        let total = 12 + self.len * 64;
        if offset >= total {
            return 0;
        }
        let count = out.len().min(total - offset);
        let mut header = [0u8; 12];
        header[..4].copy_from_slice(b"FLXR");
        header[4..6].copy_from_slice(&1u16.to_le_bytes());
        header[6..8].copy_from_slice(&64u16.to_le_bytes());
        header[8..12].copy_from_slice(&(self.len as u32).to_le_bytes());
        for (i, byte) in out[..count].iter_mut().enumerate() {
            let position = offset + i;
            *byte = if position < 12 {
                header[position]
            } else {
                let position = position - 12;
                let entry = self.get(position / 64).expect("bounded record");
                let field = position % 64;
                if field < 8 {
                    entry.micros.to_le_bytes()[field]
                } else {
                    let field = field - 8;
                    entry.values[field / 4].to_le_bytes()[field % 4]
                }
            };
        }
        count
    }
}
