#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Tick(pub u64);
impl Tick {
    pub fn age(self, then: Self) -> Option<u64> {
        self.0.checked_sub(then.0)
    }
    pub fn after(self, us: u64) -> Option<Self> {
        self.0.checked_add(us).map(Self)
    }
}
/// Extends a 32-bit microsecond clock. Poll at least every half-wrap (~35 minutes).
#[derive(Default)]
pub struct Clock32 {
    previous: Option<u32>,
    elapsed: u64,
}
impl Clock32 {
    pub fn extend(&mut self, raw: u32) -> Option<Tick> {
        if let Some(last) = self.previous {
            let delta = raw.wrapping_sub(last);
            if delta > i32::MAX as u32 {
                return None;
            }
            self.elapsed = self.elapsed.checked_add(u64::from(delta))?;
        } else {
            self.elapsed = u64::from(raw);
        }
        self.previous = Some(raw);
        Some(Tick(self.elapsed))
    }
}
