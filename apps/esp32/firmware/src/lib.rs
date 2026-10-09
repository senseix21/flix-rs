#![no_std]
//! ESP-IDF owns startup and memory. Exactly one flight task calls these exports.
use core::panic::PanicInfo;
use flight_core::{Quaternion, State, Tick, Vec3};
use flight_esp32::session::RadioSession;
use flight_esp32::{
    Owner,
    imu::{Bus, Mpu6050},
};
use flight_protocol::wire::{Decoder, Policy};
// SAFETY: signatures match `idf/main/main.c`. The `safe` functions take no
// pointers, validate their own state and serialize output access with a lock.
unsafe extern "C" {
    fn flix_hal_read(register: u8, out: *mut u8, len: u32) -> i32;
    safe fn flix_hal_write(register: u8, value: u8) -> i32;
    safe fn flix_hal_stop();
    safe fn flix_hal_now() -> u64;
    fn flix_hal_storage_read(slot: u32, out: *mut u8, len: u32) -> i32;
    fn flix_hal_storage_write(slot: u32, data: *const u8, len: u32) -> i32;
}
struct I2c;
impl Bus for I2c {
    type Error = i32;
    fn read(&mut self, register: u8, out: &mut [u8]) -> Result<(), i32> {
        // SAFETY: driver writes exactly out.len() bytes and does not retain the pointer.
        let result = unsafe { flix_hal_read(register, out.as_mut_ptr(), out.len() as u32) };
        if result == 0 { Ok(()) } else { Err(result) }
    }
    fn write(&mut self, register: u8, value: u8) -> Result<(), i32> {
        let result = flix_hal_write(register, value);
        if result == 0 { Ok(()) } else { Err(result) }
    }
}
struct Nvs;
impl flight_config::Storage for Nvs {
    fn read(
        &mut self,
        slot: usize,
        out: &mut [u8; flight_config::RECORD_SIZE],
    ) -> Result<(), flight_config::Error> {
        // SAFETY: out is a live exclusive RECORD_SIZE buffer; C writes at most len bytes.
        let result =
            unsafe { flix_hal_storage_read(slot as u32, out.as_mut_ptr(), out.len() as u32) };
        if result == 1 {
            return Err(flight_config::Error::Missing);
        }
        if result == 0 {
            Ok(())
        } else {
            Err(flight_config::Error::Storage)
        }
    }
    fn write(
        &mut self,
        slot: usize,
        data: &[u8; flight_config::RECORD_SIZE],
    ) -> Result<(), flight_config::Error> {
        // SAFETY: data is a live RECORD_SIZE buffer; C only reads len bytes.
        let result =
            unsafe { flix_hal_storage_write(slot as u32, data.as_ptr(), data.len() as u32) };
        if result == 0 {
            Ok(())
        } else {
            Err(flight_config::Error::Storage)
        }
    }
}
struct Firmware {
    owner: Owner,
    imu: Option<Mpu6050<I2c>>,
    decoder: Decoder,
    usb: Policy,
    radio: RadioSession,
    ready: bool,
}
static mut FIRMWARE: Firmware = Firmware {
    owner: Owner::BOOT,
    imu: None,
    decoder: Decoder::EMPTY,
    usb: Policy::BenchUsb {
        system: 255,
        component: 190,
    },
    radio: RadioSession::EMPTY,
    ready: false,
};
static mut INITIALIZED: bool = false;
/// app_main initializes once; only the flight task calls board exports afterwards.
/// ESP-NOW callbacks and timer callbacks exclusively use C handoffs, never this storage.
/// # Safety
/// Caller is the flight task and holds no other reference into `FIRMWARE`.
unsafe fn firmware() -> Option<&'static mut Firmware> {
    // SAFETY: single-task access per this function's contract.
    if !unsafe { INITIALIZED } {
        return None;
    }
    // SAFETY: as above; the returned borrow is the only live reference.
    Some(unsafe { &mut *core::ptr::addr_of_mut!(FIRMWARE) })
}
/// # Safety
/// pins points to seven readable aligned i32 values: SDA, SCL, RL, RR, FR, FL, SBUS RX.
/// Called once from app_main before the flight task starts.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_init(pins: *const i32) -> i32 {
    flix_hal_stop();
    if pins.is_null() || !pins.is_aligned() {
        return -1;
    }
    // SAFETY: app_main is the only caller and the flight task has not started.
    if unsafe { INITIALIZED } {
        return -1;
    }
    let mut a = [0; flight_config::RECORD_SIZE];
    let mut b = [0; flight_config::RECORD_SIZE];
    use flight_config::Storage;
    let read_a = Nvs.read(0, &mut a);
    let read_b = Nvs.read(1, &mut b);
    // Fresh erased storage may use safe unwired defaults. Corrupt existing storage latches fault.
    let recovered = flight_config::recover(&a, &b);
    let mut settings = recovered
        .as_ref()
        .map_or_else(|_| flight_config::Settings::default(), |(s, _, _)| *s);
    // SAFETY: non-null and aligned (checked); caller guarantees seven readable values.
    let board_pins = unsafe { pins.cast::<[i32; 7]>().read() };
    if settings.bind_board(board_pins).is_err() {
        return -2;
    }
    let Some(flight) = flight_core::Flight::new(settings.tuning(), settings.limits()) else {
        return -2;
    };
    // Static log storage avoids constructing a 64 KiB owner on the SDK startup stack.
    // SAFETY: INITIALIZED is false, so no export can hold a reference yet.
    let f = unsafe { &mut *core::ptr::addr_of_mut!(FIRMWARE) };
    f.owner.settings = settings;
    f.owner.flight = flight;
    if recovered.is_err()
        && !(read_a == Err(flight_config::Error::Missing)
            && read_b == Err(flight_config::Error::Missing))
    {
        f.owner.flight.fault(flight_core::Reason::Configuration);
    }
    let g = |name| settings.get(name).unwrap_or(0.0);
    let rotation = Quaternion::euler(Vec3(
        g("IMU_ROT_ROLL"),
        g("IMU_ROT_PITCH"),
        g("IMU_ROT_YAW"),
    ))
    .unwrap_or(Quaternion::IDENTITY);
    let Ok(mut imu) = Mpu6050::new(
        I2c,
        rotation,
        Vec3(
            g("IMU_ACC_BIAS_X"),
            g("IMU_ACC_BIAS_Y"),
            g("IMU_ACC_BIAS_Z"),
        ),
        Vec3(
            g("IMU_ACC_SCALE_X"),
            g("IMU_ACC_SCALE_Y"),
            g("IMU_ACC_SCALE_Z"),
        ),
    ) else {
        return -3;
    };
    let ready = imu.configure().is_ok();
    if !ready {
        f.owner.flight.fault(flight_core::Reason::SensorExpired);
    }
    f.imu = Some(imu);
    f.ready = ready;
    // SAFETY: still the sole caller; publishing happens before the task starts.
    unsafe {
        INITIALIZED = true;
    }
    if ready { 0 } else { -4 }
}
/// # Safety
/// data must point to len readable bytes. Called only by the owner task.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_usb(data: *const u8, len: u32, now: u64) {
    if data.is_null() || len > 1120 {
        return;
    }
    // SAFETY: owner-task caller per this export's contract.
    let Some(f) = (unsafe { firmware() }) else {
        return;
    };
    // SAFETY: non-null; caller guarantees len readable bytes for this call only.
    let bytes = unsafe { core::slice::from_raw_parts(data, len as usize) };
    let mut completed = 0;
    for byte in bytes {
        if let Some(packet) = f.decoder.push(*byte, &mut f.usb) {
            completed += 1;
            if let Ok(packet) = packet {
                if packet.id == 256 {
                    let accepted = f.radio.enroll(&packet, &mut f.owner, Tick(now)).is_ok();
                    f.owner.status_text(if accepted {
                        if f.radio.enabled() {
                            b"radio-enrolled"
                        } else {
                            b"radio-revoked"
                        }
                    } else {
                        b"radio-denied"
                    });
                } else if let Ok(event) = flight_protocol::event(
                    &packet,
                    f.owner.settings.get("MAV_SYS_ID").unwrap_or(1.0) as u8,
                ) {
                    f.owner.submit(event, Tick(now));
                }
            }
            if completed >= 4 {
                f.decoder.reset();
                break;
            }
        }
    }
}
/// Sign a queued USB reply for wireless transmission. Only the owner task calls it.
/// # Safety
/// out points to capacity writable bytes containing length encoded bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_sign_reply(
    out: *mut u8,
    length: u32,
    capacity: u32,
    now: u64,
) -> u32 {
    if out.is_null() || capacity > 280 || length > capacity {
        return 0;
    }
    // SAFETY: owner-task caller per this export's contract.
    let Some(f) = (unsafe { firmware() }) else {
        return 0;
    };
    // SAFETY: non-null; caller guarantees capacity (<=280) exclusive writable bytes.
    let bytes = unsafe { core::slice::from_raw_parts_mut(out, capacity as usize) };
    f.radio.sign(bytes, length as usize, Tick(now)).unwrap_or(0) as u32
}
/// # Safety
/// Only the owner task calls this export.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_radio_epoch() -> u64 {
    // SAFETY: owner-task caller per this export's contract.
    unsafe { firmware() }.map_or(0, |f| f.radio.epoch())
}
/// Bounded stopped-only timing snapshot for an owner-task instrumentation harness.
/// # Safety
/// out points to capacity writable aligned u64 values. Called only by the owner task.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_metrics(out: *mut u64, capacity: u32) -> u32 {
    if out.is_null() || out.addr() % core::mem::align_of::<u64>() != 0 || capacity < 52 {
        return 0;
    }
    // SAFETY: owner-task caller per this export's contract.
    let Some(f) = (unsafe { firmware() }) else {
        return 0;
    };
    if !f.owner.flight.stopped_maintenance() {
        return 0;
    }
    let snapshot = f.owner.timing.snapshot();
    // SAFETY: non-null, aligned (checked) and capacity >= 52 writable values.
    let out = unsafe { &mut *out.cast::<[u64; 52]>() };
    *out = snapshot;
    52
}
/// # Safety
/// data points to len readable bytes. Only the owner task may call this function.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_radio(data: *const u8, len: u32, now: u64) -> i32 {
    if data.is_null() || len > 280 {
        return -1;
    }
    // SAFETY: owner-task caller per this export's contract.
    let Some(f) = (unsafe { firmware() }) else {
        return -2;
    };
    // SAFETY: non-null; caller guarantees len (<=280) readable bytes.
    let bytes = unsafe { core::slice::from_raw_parts(data, len as usize) };
    if f.radio.receive(bytes, &mut f.owner, Tick(now)) {
        0
    } else {
        -4
    }
}
/// # Safety
/// Only the owner task calls this export.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_stop() {
    // SAFETY: owner-task caller per this export's contract.
    if let Some(f) = unsafe { firmware() } {
        f.owner.flight.disarm();
        f.owner.ingress = Default::default();
    }
    flix_hal_stop();
}
/// # Safety
/// Only the owner task calls this export.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_maintenance() -> u32 {
    // SAFETY: owner-task caller per this export's contract.
    unsafe { firmware() }.is_some_and(|f| f.owner.flight.stopped_maintenance()) as u32
}
/// # Safety
/// data points to exactly 25 readable bytes; caller is the flight owner task.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_rc(data: *const u8, now: u64) {
    if data.is_null() {
        return;
    }
    // SAFETY: owner-task caller per this export's contract.
    if let Some(f) = unsafe { firmware() } {
        // SAFETY: non-null; caller guarantees 25 readable bytes.
        let bytes = unsafe { core::slice::from_raw_parts(data, 25) };
        f.owner.receive_rc(bytes, Tick(now));
    }
}
/// # Safety
/// motors points to four writable aligned floats; only the flight task calls this export.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_tick(motors: *mut f32) -> u32 {
    if motors.is_null() || !motors.is_aligned() {
        flix_hal_stop();
        return 5;
    }
    // SAFETY: non-null, aligned (checked); caller guarantees four exclusive floats.
    let motors = unsafe { &mut *motors.cast::<[f32; 4]>() };
    *motors = [0.0; 4];
    // SAFETY: owner-task caller per this export's contract.
    let Some(f) = (unsafe { firmware() }) else {
        return 5;
    };
    let start = flix_hal_now();
    if f.owner.driver_restart_requested && f.owner.flight.stopped_maintenance() {
        flix_hal_stop();
        let g = |name| f.owner.settings.get(name).unwrap_or(0.0);
        let rebuilt = Quaternion::euler(Vec3(
            g("IMU_ROT_ROLL"),
            g("IMU_ROT_PITCH"),
            g("IMU_ROT_YAW"),
        ))
        .and_then(|q| {
            Mpu6050::new(
                I2c,
                q,
                Vec3(
                    g("IMU_ACC_BIAS_X"),
                    g("IMU_ACC_BIAS_Y"),
                    g("IMU_ACC_BIAS_Z"),
                ),
                Vec3(
                    g("IMU_ACC_SCALE_X"),
                    g("IMU_ACC_SCALE_Y"),
                    g("IMU_ACC_SCALE_Z"),
                ),
            )
            .ok()
        });
        if let Some(mut imu) = rebuilt {
            f.ready = imu.configure().is_ok();
            f.imu = Some(imu);
        } else {
            f.ready = false;
        }
        if !f.ready {
            f.owner.flight.fault(flight_core::Reason::Configuration);
        }
        f.owner.driver_restart_requested = false;
    }
    if f.owner.calibration_requested && f.owner.flight.stopped_maintenance() {
        if let Some(imu) = &mut f.imu {
            imu.reset_calibration();
        }
        f.owner.calibration_requested = false;
    }
    let stopped = matches!(
        f.owner.flight.snapshot().state,
        State::Boot | State::Standby | State::Failsafe | State::Fault
    ) || f.owner.flight.stopped_maintenance();
    let acquisition_start = flix_hal_now();
    let sensor = if f.ready {
        f.imu
            .as_mut()
            .and_then(|imu| imu.sample(Tick(flix_hal_now()), stopped).ok())
    } else {
        None
    };
    let now = Tick(flix_hal_now());
    f.owner
        .timing
        .acquisition
        .record(now.0.saturating_sub(acquisition_start));
    let values = f.owner.tick(now, sensor);
    f.owner
        .timing
        .core
        .record(flix_hal_now().saturating_sub(now.0));
    if f.owner.save_requested {
        flix_hal_stop();
        let _ = flight_config::save(&mut Nvs, &f.owner.settings, &f.owner.flight);
        f.owner.save_requested = false;
    }
    f.owner
        .record_execution(flix_hal_now().saturating_sub(start));
    *motors = values;
    f.owner.flight.snapshot().state as u32
}
/// # Safety
/// out points to capacity writable bytes. Only the owner task calls this export.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_board_reply(out: *mut u8, capacity: u32) -> u32 {
    if out.is_null() || capacity < 280 {
        return 0;
    }
    // SAFETY: owner-task caller per this export's contract.
    let Some(f) = (unsafe { firmware() }) else {
        return 0;
    };
    let Some(reply) = f.owner.reply() else {
        return 0;
    };
    let Some(reply) = reply.bytes.get(..reply.len) else {
        return 0;
    };
    // SAFETY: non-null; caller guarantees capacity (>=280) exclusive writable bytes.
    let out = unsafe { core::slice::from_raw_parts_mut(out, capacity as usize) };
    out[..reply.len()].copy_from_slice(reply);
    reply.len() as u32
}
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    flix_hal_stop();
    loop {
        core::hint::spin_loop();
    }
}
