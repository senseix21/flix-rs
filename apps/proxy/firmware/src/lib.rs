#![no_std]
//! The SDK bridge task is the sole caller. Callbacks only use C queue handoffs.
use flight_proxy::Bridge;
static mut BRIDGE: Option<Bridge> = None;
unsafe fn bridge() -> Option<&'static mut Bridge> {
    // SAFETY: exactly one SDK owner task calls all exports, serialized for life.
    unsafe { (&mut *core::ptr::addr_of_mut!(BRIDGE)).as_mut() }
}
/// # Safety
/// Owner-only; peer references six readable bytes for this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_proxy_init(peer: *const u8) -> i32 {
    if peer.is_null() {
        return -1;
    }
    let mut mac = [0; 6];
    mac.copy_from_slice(unsafe { core::slice::from_raw_parts(peer, 6) });
    match Bridge::new(mac) {
        Ok(b) => {
            unsafe { *core::ptr::addr_of_mut!(BRIDGE) = Some(b) };
            0
        }
        Err(_) => -1,
    }
}
/// # Safety
/// Owner-only; data is readable for len bytes (max 1120) and not retained.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_proxy_input(data: *const u8, len: u32, now: u64) -> i32 {
    if data.is_null() || len > 1120 {
        return -1;
    }
    let Some(b) = (unsafe { bridge() }) else {
        return -1;
    };
    if b.serial(
        unsafe { core::slice::from_raw_parts(data, len as usize) },
        now,
    )
    .is_ok()
    {
        0
    } else {
        -2
    }
}
/// # Safety
/// Owner-only; peer/data are readable for six/len bytes; len <=250.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_proxy_received(
    peer: *const u8,
    data: *const u8,
    len: u32,
    now: u64,
) -> i32 {
    if peer.is_null() || data.is_null() || len > 250 {
        return -1;
    }
    let Some(b) = (unsafe { bridge() }) else {
        return -1;
    };
    let mut mac = [0; 6];
    mac.copy_from_slice(unsafe { core::slice::from_raw_parts(peer, 6) });
    if b.received(
        mac,
        unsafe { core::slice::from_raw_parts(data, len as usize) },
        now,
    )
    .is_ok()
    {
        0
    } else {
        -2
    }
}
/// # Safety
/// Owner-only; out references 250 writable bytes. No pointer retained.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_proxy_output(out: *mut u8, radio: bool, now: u64) -> u32 {
    if out.is_null() {
        return 0;
    }
    let Some(b) = (unsafe { bridge() }) else {
        return 0;
    };
    let Some(frame) = (if radio { b.radio(now) } else { b.usb(now) }) else {
        return 0;
    };
    unsafe { core::ptr::copy_nonoverlapping(frame.bytes.as_ptr(), out, frame.length) };
    frame.length as u32
}
/// # Safety
/// Owner-only. Clear both queues and partial framing on disconnect or overflow.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_proxy_reset() {
    if let Some(b) = unsafe { bridge() } {
        b.reset();
    }
}
#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
