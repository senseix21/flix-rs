use crate::Error;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Peer {
    pub mac: [u8; 6],
    pub key: [u8; 16],
}
/// Explicit provisioning only. Unsolicited discovery cannot consume the peer pool.
#[derive(Default)]
pub struct Peers {
    slots: [Option<Peer>; 4],
}
impl Peers {
    pub fn add(&mut self, peer: Peer) -> Result<usize, Error> {
        if peer.mac == [0; 6] || peer.mac[0] & 1 != 0 || peer.key == [0; 16] {
            return Err(Error::Unauthorized);
        }
        if let Some(i) = self
            .slots
            .iter()
            .position(|p| p.is_some_and(|p| p.mac == peer.mac))
        {
            self.slots[i] = Some(peer);
            return Ok(i);
        }
        let slot = self
            .slots
            .iter()
            .position(Option::is_none)
            .ok_or(Error::Full)?;
        self.slots[slot] = Some(peer);
        Ok(slot)
    }
    pub fn remove(&mut self, mac: [u8; 6]) -> bool {
        if let Some(slot) = self
            .slots
            .iter_mut()
            .find(|p| p.is_some_and(|p| p.mac == mac))
        {
            *slot = None;
            true
        } else {
            false
        }
    }
    pub fn get(&self, mac: [u8; 6]) -> Option<Peer> {
        self.slots.iter().flatten().find(|p| p.mac == mac).copied()
    }
}
