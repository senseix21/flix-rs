//! Bounded MAVLink 2 subset. Table is generated from the frozen common dialect inventory.
use crate::Error;
use sha2::{Digest, Sha256};
pub const MAX_FRAME: usize = 280;
#[derive(Clone, Copy, Debug)]
pub struct Packet {
    pub id: u32,
    pub system: u8,
    pub component: u8,
    pub sequence: u8,
    pub length: u8,
    pub payload: [u8; 255],
}
include!("dialect.rs");
pub fn crc(bytes: &[u8], extra: u8) -> u16 {
    let mut result = 0xffffu16;
    for byte in bytes.iter().chain(core::iter::once(&extra)) {
        let mut tmp = *byte ^ result.to_le_bytes()[0];
        tmp ^= tmp << 4;
        result =
            (result >> 8) ^ (u16::from(tmp) << 8) ^ (u16::from(tmp) << 3) ^ (u16::from(tmp) >> 4);
    }
    result
}
pub fn encode(
    id: u32,
    payload: &[u8],
    sequence: u8,
    system: u8,
    component: u8,
    out: &mut [u8],
) -> Result<usize, Error> {
    let (extra, _, max) = dialect(id).ok_or(Error::Unsupported)?;
    if payload.len() > usize::from(max)
        || system == 0
        || component == 0
        || out.len() < payload.len() + 12
    {
        return Err(Error::Bounds);
    }
    // max <= 255 and payload.len() <= max (checked above), so the length fits a byte.
    let length = u8::try_from(payload.len()).map_err(|_| Error::Bounds)?;
    let [id0, id1, id2, _] = id.to_le_bytes();
    out[..10].copy_from_slice(&[
        0xfd, length, 0, 0, sequence, system, component, id0, id1, id2,
    ]);
    out[10..10 + payload.len()].copy_from_slice(payload);
    let checksum = crc(&out[1..10 + payload.len()], extra);
    out[10 + payload.len()..12 + payload.len()].copy_from_slice(&checksum.to_le_bytes());
    Ok(payload.len() + 12)
}
#[derive(Clone, Copy)]
struct Stream {
    system: u8,
    component: u8,
    link: u8,
    timestamp: u64,
}
pub const MAX_TIMESTAMP: u64 = (1 << 48) - 1;
/// Trusted enrollment anchors MAVLink time to monotonic time; reboot revokes the session.
pub struct Signing {
    key: [u8; 32],
    floor: u64,
    boot_timestamp: u64,
    anchor_us: u64,
    outgoing: u64,
    streams: [Option<Stream>; 4],
}
impl Signing {
    pub fn new(key: [u8; 32], floor: u64) -> Result<Self, Error> {
        Self::enrolled(key, floor, 0)
    }
    pub fn enrolled(key: [u8; 32], timestamp: u64, monotonic_us: u64) -> Result<Self, Error> {
        if timestamp == 0 || timestamp > MAX_TIMESTAMP || key.iter().all(|v| *v == 0) {
            return Err(Error::Unauthorized);
        }
        Ok(Self {
            key,
            floor: timestamp,
            boot_timestamp: timestamp,
            anchor_us: monotonic_us,
            outgoing: timestamp,
            streams: [None; 4],
        })
    }
    /// Timestamp floor is trusted MAVLink time at monotonic boot zero. Retain a 60s lag allowance.
    pub fn advance(&mut self, monotonic_us: u64) {
        self.floor = self.floor.max(
            self.boot_timestamp
                .saturating_add(monotonic_us.saturating_sub(self.anchor_us) / 10)
                .saturating_sub(6_000_000),
        );
    }
    /// Sign an already encoded unsigned frame, including its updated CRC.
    /// Call only from the owner. No allocation; failure leaves no valid signed frame.
    pub fn sign(
        &mut self,
        frame: &mut [u8],
        length: usize,
        now_us: u64,
        link: u8,
    ) -> Result<usize, Error> {
        if length < 12
            || length + 13 > frame.len()
            || frame[0] != 253
            || frame[2] != 0
            || usize::from(frame[1]) + 12 != length
        {
            return Err(Error::Framing);
        }
        let id = u32::from(frame[7]) | (u32::from(frame[8]) << 8) | (u32::from(frame[9]) << 16);
        let (extra, _, maximum) = dialect(id).ok_or(Error::Unsupported)?;
        if frame[1] > maximum
            || crc(&frame[1..length - 2], extra)
                != u16::from_le_bytes([frame[length - 2], frame[length - 1]])
        {
            return Err(Error::Checksum);
        }
        let timestamp = self
            .boot_timestamp
            .checked_add(now_us.checked_sub(self.anchor_us).ok_or(Error::Replay)? / 10)
            .ok_or(Error::Replay)?
            .max(self.outgoing.checked_add(1).ok_or(Error::Replay)?);
        if timestamp > MAX_TIMESTAMP {
            return Err(Error::Replay);
        }
        frame[2] = 1;
        let checksum = crc(&frame[1..length - 2], extra);
        frame[length - 2..length].copy_from_slice(&checksum.to_le_bytes());
        frame[length] = link;
        frame[length + 1..length + 7].copy_from_slice(&timestamp.to_le_bytes()[..6]);
        let mut digest = Sha256::new();
        digest.update(self.key);
        digest.update(&frame[..length + 7]);
        frame[length + 7..length + 13].copy_from_slice(&digest.finalize()[..6]);
        self.outgoing = timestamp;
        Ok(length + 13)
    }
    fn verify(&mut self, bytes: &[u8], system: u8, component: u8) -> Result<(), Error> {
        let signature = &bytes[bytes.len() - 13..];
        let link = signature[0];
        let mut time = [0u8; 8];
        time[..6].copy_from_slice(&signature[1..7]);
        let timestamp = u64::from_le_bytes(time);
        if timestamp < self.floor {
            return Err(Error::Replay);
        }
        let existing = self.streams.iter().position(|entry| {
            entry.is_some_and(|s| s.system == system && s.component == component && s.link == link)
        });
        if existing.is_some_and(|i| self.streams[i].is_some_and(|s| timestamp <= s.timestamp)) {
            return Err(Error::Replay);
        }
        let slot = existing
            .or_else(|| self.streams.iter().position(Option::is_none))
            .ok_or(Error::Full)?;
        let mut digest = Sha256::new();
        digest.update(self.key);
        digest.update(&bytes[..bytes.len() - 6]);
        let expected = digest.finalize();
        let mismatch = expected[..6]
            .iter()
            .zip(&signature[7..])
            .fold(0u8, |a, (b, c)| a | (b ^ c));
        if mismatch != 0 {
            return Err(Error::Unauthorized);
        }
        self.streams[slot] = Some(Stream {
            system,
            component,
            link,
            timestamp,
        });
        Ok(())
    }
}
pub enum Policy {
    BenchUsb {
        system: u8,
        component: u8,
    },
    Signed {
        system: u8,
        component: u8,
        signing: Signing,
    },
}
pub fn decode(bytes: &[u8], policy: &mut Policy) -> Result<Packet, Error> {
    if bytes.len() < 12 || bytes.len() > MAX_FRAME || bytes[0] != 0xfd || bytes[2] & !1 != 0 {
        return Err(Error::Framing);
    }
    let length = usize::from(bytes[1]);
    let signed = bytes[2] & 1 != 0;
    if bytes.len() != length + 12 + if signed { 13 } else { 0 } {
        return Err(Error::Framing);
    }
    let id = u32::from(bytes[7]) | (u32::from(bytes[8]) << 8) | (u32::from(bytes[9]) << 16);
    let (extra, _, max) = dialect(id).ok_or(Error::Unsupported)?;
    if length > usize::from(max) {
        return Err(Error::Bounds);
    }
    if crc(&bytes[1..10 + length], extra)
        != u16::from_le_bytes([bytes[10 + length], bytes[11 + length]])
    {
        return Err(Error::Checksum);
    }
    let (system, component) = match policy {
        Policy::BenchUsb { system, component }
        | Policy::Signed {
            system, component, ..
        } => (*system, *component),
    };
    // The wired bench path also accepts component 0, pymavlink's default sender, so upstream
    // pyflix works unmodified (R2-04, decided 2026-10-13). On this unsigned trusted link the
    // sender ID is hygiene, not authentication; the signed radio policy stays exact.
    let wired_default = matches!(policy, Policy::BenchUsb { .. }) && bytes[6] == 0;
    if system == 0
        || component == 0
        || bytes[5] != system
        || (bytes[6] != component && !wired_default)
    {
        return Err(Error::Unauthorized);
    }
    match policy {
        Policy::BenchUsb { .. } if signed => return Err(Error::Unauthorized),
        Policy::Signed { signing, .. } => {
            if !signed {
                return Err(Error::Unauthorized);
            }
            signing.verify(bytes, system, component)?;
        }
        _ => {}
    }
    let mut payload = [0; 255];
    payload[..length].copy_from_slice(&bytes[10..10 + length]);
    Ok(Packet {
        id,
        system,
        component,
        sequence: bytes[4],
        length: bytes[1],
        payload,
    })
}
/// UART stream accumulator; length bounded before accepting a complete packet.
pub struct Decoder {
    bytes: [u8; MAX_FRAME],
    len: usize,
    needed: usize,
}
impl Default for Decoder {
    fn default() -> Self {
        Self {
            bytes: [0; MAX_FRAME],
            len: 0,
            needed: 0,
        }
    }
}
impl Decoder {
    pub const EMPTY: Self = Self {
        bytes: [0; MAX_FRAME],
        len: 0,
        needed: 0,
    };
    pub fn reset(&mut self) {
        self.bytes.fill(0);
        self.len = 0;
        self.needed = 0;
    }
    pub fn push(&mut self, byte: u8, policy: &mut Policy) -> Option<Result<Packet, Error>> {
        if self.len == 0 && byte != 0xfd {
            return None;
        }
        self.bytes[self.len] = byte;
        self.len += 1;
        if self.len == 3 {
            self.needed = usize::from(self.bytes[1]) + 12 + if byte & 1 != 0 { 13 } else { 0 };
        }
        if self.needed != 0 && self.len == self.needed {
            let result = decode(&self.bytes[..self.len], policy);
            self.reset();
            return Some(result);
        }
        None
    }
}

/// Inspect a signed transport envelope without authenticating it. Only endpoints
/// possessing the enrolled session key may authorize the returned packet.
pub(crate) fn signed_envelope(bytes: &[u8], system: u8, component: u8) -> Result<u32, Error> {
    if bytes.len() < 25 || bytes.len() > MAX_FRAME || bytes[0] != 253 || bytes[2] != 1 {
        return Err(Error::Framing);
    }
    let length = usize::from(bytes[1]);
    if bytes.len() != length + 25 {
        return Err(Error::Framing);
    }
    let id = u32::from(bytes[7]) | (u32::from(bytes[8]) << 8) | (u32::from(bytes[9]) << 16);
    let (extra, _, maximum) = dialect(id).ok_or(Error::Unsupported)?;
    if bytes[1] > maximum {
        return Err(Error::Bounds);
    }
    if bytes[5] != system || bytes[6] != component || system == 0 || component == 0 {
        return Err(Error::Unauthorized);
    }
    if crc(&bytes[1..10 + length], extra)
        != u16::from_le_bytes([bytes[10 + length], bytes[11 + length]])
    {
        return Err(Error::Checksum);
    }
    Ok(id)
}
