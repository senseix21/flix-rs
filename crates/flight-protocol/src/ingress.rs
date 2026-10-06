use crate::{Error, Event};
use flight_core::{Flight, Tick};
/// Owned by one task. Callbacks hand off bytes; they never hold &mut Ingress concurrently.
#[derive(Default)]
pub struct Ingress {
    queue: [Option<(Event, Tick)>; 8],
    head: usize,
    len: usize,
    latest: [Option<(Event, Tick)>; 3],
    stop: bool,
    pub rejected: u64,
}
impl Ingress {
    pub const EMPTY: Self = Self {
        queue: [None; 8],
        head: 0,
        len: 0,
        latest: [None; 3],
        stop: false,
        rejected: 0,
    };
    pub fn push(&mut self, event: Event, at: Tick) -> Result<(), Error> {
        match event {
            Event::Stop | Event::Console(crate::Console::Disarm) => {
                self.stop = true;
                Ok(())
            }
            Event::Heartbeat => Ok(()),
            Event::Setpoint(source, _) => {
                let slot = &mut self.latest[source as usize];
                if slot.is_some_and(|(_, old)| at < old) {
                    return Err(Error::Replay);
                }
                *slot = Some((event, at));
                Ok(())
            }
            _ => {
                if self.len == 8 {
                    self.rejected = self.rejected.saturating_add(1);
                    return Err(Error::Full);
                }
                self.queue[(self.head + self.len) % 8] = Some((event, at));
                self.len += 1;
                Ok(())
            }
        }
    }
    pub fn take_stop(&mut self) -> bool {
        if !self.stop {
            return false;
        }
        self.stop = false;
        self.queue = [None; 8];
        self.head = 0;
        self.len = 0;
        self.latest = [None; 3];
        true
    }
    pub fn take_setpoints(&mut self) -> [Option<(Event, Tick)>; 3] {
        core::mem::take(&mut self.latest)
    }
    pub fn pop(&mut self) -> Option<(Event, Tick)> {
        if self.len == 0 {
            return None;
        }
        let result = self.queue[self.head].take();
        self.head = (self.head + 1) % 8;
        self.len -= 1;
        result
    }
    /// Control-only runner. Rich services use the same stop-first draining policy.
    pub fn apply(&mut self, flight: &mut Flight, now: Tick) {
        if self.take_stop() {
            flight.disarm();
            return;
        }
        for (event, at) in self.take_setpoints().into_iter().flatten() {
            if let Event::Setpoint(source, intent) = event {
                flight.command(source, intent, at);
            }
        }
        for _ in 0..2 {
            if let Some((event, _)) = self.pop() {
                match event {
                    Event::Arm => {
                        flight.arm(now);
                    }
                    Event::Select(source) => {
                        flight.select(source);
                    }
                    _ => {}
                }
            }
        }
    }
}
