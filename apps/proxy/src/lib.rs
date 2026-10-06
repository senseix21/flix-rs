#![no_std]
//! One-owner transparent serial/radio bridge. No keys or flight authority here.
use flight_protocol::{
    Error,
    proxy::{Frame, Framer, relayable},
    wire::MAX_FRAME,
};
pub const MAX_AGE_US: u64 = 100_000;
#[derive(Clone, Copy)]
struct Pending {
    frame: Frame,
    at: u64,
}
struct Queue {
    frames: [Option<Pending>; 4],
    stop: Option<Pending>,
}
impl Queue {
    const EMPTY: Self = Self {
        frames: [None; 4],
        stop: None,
    };
    fn push(&mut self, frame: Frame, at: u64, stop: bool) -> Result<(), Error> {
        if stop {
            // Do not forward queued older pilot/arm work after a Stop.
            self.frames.fill(None);
            self.stop = Some(Pending { frame, at });
            return Ok(());
        }
        let slot = self
            .frames
            .iter_mut()
            .find(|f| f.is_none())
            .ok_or(Error::Full)?;
        *slot = Some(Pending { frame, at });
        Ok(())
    }
    fn pop(&mut self, now: u64) -> Option<Frame> {
        for _ in 0..5 {
            let next = self.stop.take().or_else(|| {
                let frame = self.frames[0].take();
                self.frames.rotate_left(1);
                frame
            });
            if let Some(p) = next {
                if now.checked_sub(p.at).is_some_and(|age| age <= MAX_AGE_US) {
                    return Some(p.frame);
                }
            }
        }
        None
    }
}
pub struct Bridge {
    framer: Framer,
    radio: Queue,
    usb: Queue,
    peer: [u8; 6],
}
impl Bridge {
    pub fn new(peer: [u8; 6]) -> Result<Self, Error> {
        if peer[0] & 1 != 0 || peer == [0; 6] {
            return Err(Error::Bounds);
        }
        Ok(Self {
            framer: Framer::EMPTY,
            radio: Queue::EMPTY,
            usb: Queue::EMPTY,
            peer,
        })
    }
    pub fn reset(&mut self) {
        self.framer.reset();
        self.radio = Queue::EMPTY;
        self.usb = Queue::EMPTY;
    }
    pub fn serial(&mut self, bytes: &[u8], now: u64) -> Result<(), Error> {
        if bytes.len() > 1120 {
            return Err(Error::Bounds);
        }
        let mut error = None;
        for byte in bytes {
            if let Some(frame) = self.framer.push(*byte, now) {
                match relayable(&frame, 255, 190).and_then(|stop| self.radio.push(frame, now, stop))
                {
                    Ok(()) => {}
                    Err(e) => error = Some(e),
                }
            }
        }
        error.map_or(Ok(()), Err)
    }
    pub fn received(&mut self, peer: [u8; 6], bytes: &[u8], now: u64) -> Result<(), Error> {
        if peer != self.peer {
            return Err(Error::Unauthorized);
        }
        if bytes.len() > 250 {
            return Err(Error::Bounds);
        }
        let mut frame = Frame {
            bytes: [0; MAX_FRAME],
            length: bytes.len(),
        };
        frame.bytes[..bytes.len()].copy_from_slice(bytes);
        relayable(&frame, 1, 1)?;
        self.usb.push(frame, now, false)
    }
    pub fn radio(&mut self, now: u64) -> Option<Frame> {
        self.radio.pop(now)
    }
    pub fn usb(&mut self, now: u64) -> Option<Frame> {
        self.usb.pop(now)
    }
}
