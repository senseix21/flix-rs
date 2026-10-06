//! Bounded serial framing for a provisioned radio proxy. No flight state lives here.
use crate::{
    Error,
    wire::{self, MAX_FRAME, Policy},
};
#[derive(Clone, Copy)]
pub struct Frame {
    pub bytes: [u8; MAX_FRAME],
    pub length: usize,
}
pub struct Framer {
    bytes: [u8; MAX_FRAME],
    length: usize,
    needed: usize,
    started: Option<u64>,
}
impl Default for Framer {
    fn default() -> Self {
        Self::EMPTY
    }
}
impl Framer {
    pub const EMPTY: Self = Self {
        bytes: [0; MAX_FRAME],
        length: 0,
        needed: 0,
        started: None,
    };
    pub fn reset(&mut self) {
        self.bytes.fill(0);
        self.length = 0;
        self.needed = 0;
        self.started = None;
    }
    /// Incomplete frames expire after 100 ms (280-byte frame is ~25 ms at 115200).
    pub fn push(&mut self, byte: u8, now_us: u64) -> Option<Frame> {
        if self
            .started
            .is_some_and(|start| now_us < start || now_us - start > 100_000)
        {
            self.reset();
        }
        if self.length == 0 {
            if byte != 253 {
                return None;
            }
            self.started = Some(now_us);
        }
        self.bytes[self.length] = byte;
        self.length += 1;
        if self.length == 3 {
            self.needed = usize::from(self.bytes[1]) + 12 + if byte & 1 != 0 { 13 } else { 0 };
        }
        if self.needed > MAX_FRAME {
            self.reset();
            return None;
        }
        if self.needed > 0 && self.length == self.needed {
            let frame = Frame {
                bytes: self.bytes,
                length: self.length,
            };
            self.reset();
            Some(frame)
        } else {
            None
        }
    }
}
/// Authenticate before forwarding or reserving queue capacity. Enrollment is never relayed.
pub fn admit(frame: &Frame, policy: &mut Policy, now_us: u64) -> Result<(), Error> {
    let Policy::Signed { signing, .. } = policy else {
        return Err(Error::Unauthorized);
    };
    signing.advance(now_us);
    let packet = wire::decode(
        frame.bytes.get(..frame.length).ok_or(Error::Bounds)?,
        policy,
    )?;
    if packet.id == 256 {
        return Err(Error::Unauthorized);
    }
    Ok(())
}

/// A transparent proxy has no session key. Require the signed envelope and CRC;
/// the flight owner/client still authenticate every forwarded signature.
/// Returns true only for a complete addressed disarm command, to reserve capacity.
pub fn relayable(frame: &Frame, system: u8, component: u8) -> Result<bool, Error> {
    let bytes = frame.bytes.get(..frame.length).ok_or(Error::Bounds)?;
    let id = wire::signed_envelope(bytes, system, component)?;
    if id == 256 || bytes.len() > 250 {
        return Err(Error::Unauthorized);
    }
    let stop = id == 76
        && bytes[1] >= 32
        && bytes[38..40] == 400u16.to_le_bytes()
        && bytes[40] == 1
        && bytes[41] == 1
        && bytes[10..38]
            .chunks_exact(4)
            .all(|v| f32::from_le_bytes([v[0], v[1], v[2], v[3]]) == 0.0);
    Ok(stop)
}
