//! POSIX serial bridge with loopback UDP standing in for ESP-NOW in host tests.
use flight_proxy::Bridge;
use std::os::fd::{AsRawFd, FromRawFd};
use std::{
    fs::{File, OpenOptions},
    io::{Read, Write},
    net::UdpSocket,
    time::{Duration, Instant},
};
unsafe extern "C" {
    fn fcntl(fd: i32, cmd: i32, ...) -> i32;
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().ok_or("serial path required")?;
    let mut serial = if path == "--fd" {
        let fd: i32 = args.next().ok_or("inherited serial fd required")?.parse()?;
        // SAFETY: F_GETFL takes no pointer argument; an invalid fd only returns -1.
        if fd < 3 || unsafe { fcntl(fd, 3) } < 0 {
            return Err("invalid inherited serial fd".into());
        }
        // SAFETY: validated open descriptor inherited at process startup. This
        // program has no other owner; File now closes it exactly once. Taking
        // ownership avoids opening a new PTY through /dev/fd on Linux.
        unsafe { File::from_raw_fd(fd) }
    } else {
        OpenOptions::new().read(true).write(true).open(path)?
    };
    let port: u16 = args.next().ok_or("loopback port required")?.parse()?;
    #[cfg(target_os = "macos")]
    const NONBLOCK: i32 = 4;
    #[cfg(not(target_os = "macos"))]
    const NONBLOCK: i32 = 2048;
    // SAFETY: valid owned fd; POSIX F_GETFL=3/F_SETFL=4 use integer arguments.
    let flags = unsafe { fcntl(serial.as_raw_fd(), 3) };
    // SAFETY: as above.
    if flags < 0 || unsafe { fcntl(serial.as_raw_fd(), 4, flags | NONBLOCK) } < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    let socket = UdpSocket::bind(("127.0.0.1", 0))?;
    socket.connect(("127.0.0.1", port))?;
    socket.set_nonblocking(true)?;
    let peer = [2, 1, 2, 3, 4, 5];
    let mut bridge = Bridge::new(peer).map_err(|_| "invalid peer")?;
    let start = Instant::now();
    let mut bytes = [0; 1120];
    let mut pending = None;
    println!("ready");
    std::io::stdout().flush()?;
    loop {
        let now = start.elapsed().as_micros() as u64;
        match serial.read(&mut bytes) {
            Ok(0) => {
                bridge.reset();
                return Ok(());
            }
            Ok(n) => {
                let _ = bridge.serial(&bytes[..n], now);
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) => {
                bridge.reset();
                return Err(e.into());
            }
        }
        for _ in 0..4 {
            match socket.recv(&mut bytes) {
                Ok(n) => {
                    let _ = bridge.received(peer, &bytes[..n], now);
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e.into()),
            }
        }
        if let Some(frame) = bridge.radio(now) {
            socket.send(&frame.bytes[..frame.length])?;
        }
        if pending.is_none() {
            pending = bridge.usb(now).map(|f| (f, 0, now));
        }
        if let Some((frame, offset, at)) = pending.as_mut() {
            if now.saturating_sub(*at) > 100000 {
                bridge.reset();
                return Err("serial output expired".into());
            }
            match serial.write(&frame.bytes[*offset..frame.length]) {
                Ok(n) => {
                    *offset += n;
                    if *offset == frame.length {
                        pending = None;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => return Err(e.into()),
            }
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}
