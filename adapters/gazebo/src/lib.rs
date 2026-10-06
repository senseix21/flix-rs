//! Numeric handles prevent stale/double-free handles from dereferencing freed memory.
//! Pointers remain C caller obligations: valid, aligned, writable, nonoverlapping and live.
use flight_core::{Sensor, Tick, Vec3};
use flight_esp32::Owner;
use std::sync::{LazyLock, Mutex};
struct Registry {
    next: u64,
    slots: [Option<(u64, Box<Owner>)>; 8],
}
static REGISTRY: LazyLock<Mutex<Registry>> = LazyLock::new(|| {
    Mutex::new(Registry {
        next: 1,
        slots: core::array::from_fn(|_| None),
    })
});
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Input {
    pub version: u32,
    pub calibrated: u32,
    pub now_us: u64,
    pub sample_us: u64,
    pub gyro: [f32; 3],
    pub acceleration: [f32; 3],
}
#[repr(C)]
pub struct Output {
    pub version: u32,
    pub state: u32,
    pub reason: u32,
    pub reserved: u32,
    pub now_us: u64,
    pub generation: u64,
    pub motors: [f32; 4],
}
#[unsafe(no_mangle)]
pub extern "C" fn flix_create() -> u64 {
    let Ok(mut registry) = REGISTRY.lock() else {
        return 0;
    };
    let Some(slot) = registry.slots.iter().position(Option::is_none) else {
        return 0;
    };
    let id = registry.next;
    let Some(next) = id.checked_add(1) else {
        return 0;
    };
    registry.next = next;
    registry.slots[slot] = Some((id, Box::new(Owner::default())));
    id
}
#[unsafe(no_mangle)]
pub extern "C" fn flix_destroy(handle: u64) -> i32 {
    let Ok(mut registry) = REGISTRY.lock() else {
        return -1;
    };
    if let Some(slot) = registry
        .slots
        .iter_mut()
        .find(|s| s.as_ref().is_some_and(|(id, _)| *id == handle))
    {
        *slot = None;
        0
    } else {
        -2
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn flix_reset(handle: u64) -> i32 {
    let Ok(mut registry) = REGISTRY.lock() else {
        return -1;
    };
    let Some((_, owner)) = registry
        .slots
        .iter_mut()
        .flatten()
        .find(|(id, _)| *id == handle)
    else {
        return -2;
    };
    owner.reset();
    0
}
/// # Safety
/// input/output must be valid aligned objects for the call; they must not alias.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_tick(handle: u64, input: *const Input, output: *mut Output) -> i32 {
    if input.is_null()
        || output.is_null()
        || (input as usize) % align_of::<Input>() != 0
        || (output as usize) % align_of::<Output>() != 0
    {
        return -3;
    }
    // SAFETY: validated alignment/null; object lifetime/extent/nonaliasing are caller contract.
    let input = unsafe { *input };
    // Initialize stopped output before checking handle or input version.
    unsafe {
        output.write(Output {
            version: 1,
            state: 5,
            reason: 5,
            reserved: 0,
            now_us: input.now_us,
            generation: 0,
            motors: [0.0; 4],
        });
    }
    if input.version != 1 || input.calibrated > 1 {
        return -4;
    }
    let Ok(mut registry) = REGISTRY.lock() else {
        return -1;
    };
    let Some((_, owner)) = registry
        .slots
        .iter_mut()
        .flatten()
        .find(|(id, _)| *id == handle)
    else {
        return -2;
    };
    let sample = Sensor {
        at: Tick(input.sample_us),
        gyro: Vec3(input.gyro[0], input.gyro[1], input.gyro[2]),
        acceleration: Vec3(
            input.acceleration[0],
            input.acceleration[1],
            input.acceleration[2],
        ),
        calibrated: input.calibrated != 0,
    };
    let motors = owner.tick(Tick(input.now_us), Some(sample));
    let snapshot = owner.flight.snapshot();
    // SAFETY: same output object contract as above. No borrowed pointer is stored.
    unsafe {
        output.write(Output {
            version: 1,
            state: snapshot.state as u32,
            reason: snapshot.reason as u32,
            reserved: 0,
            now_us: input.now_us,
            generation: snapshot.generation,
            motors,
        });
    }
    0
}
/// Simulation transport is explicitly unauthenticated bench traffic. Never link this host ABI into firmware.
/// # Safety
/// bytes must reference length readable bytes for this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_receive(
    handle: u64,
    bytes: *const u8,
    length: u32,
    at_us: u64,
) -> i32 {
    if bytes.is_null() || length > 280 {
        return -3;
    }
    // SAFETY: caller promises readable extent; length checked and slice does not escape.
    let data = unsafe { core::slice::from_raw_parts(bytes, length as usize) };
    let Ok(mut registry) = REGISTRY.lock() else {
        return -1;
    };
    let Some((_, owner)) = registry
        .slots
        .iter_mut()
        .flatten()
        .find(|(id, _)| *id == handle)
    else {
        return -2;
    };
    let mut policy = flight_protocol::wire::Policy::BenchUsb {
        system: 255,
        component: 190,
    };
    if owner.receive(data, &mut policy, Tick(at_us)) {
        0
    } else {
        -4
    }
}
/// # Safety
/// out must reference capacity writable bytes; it must not alias registry state.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_reply(handle: u64, out: *mut u8, capacity: u32) -> i32 {
    if out.is_null() || capacity < 280 {
        return -3;
    }
    let Ok(mut registry) = REGISTRY.lock() else {
        return -1;
    };
    let Some((_, owner)) = registry
        .slots
        .iter_mut()
        .flatten()
        .find(|(id, _)| *id == handle)
    else {
        return -2;
    };
    let Some(reply) = owner.reply() else {
        return 0;
    };
    // SAFETY: caller's output extent >=280; reply length <=280, regions cannot alias.
    unsafe {
        core::ptr::copy_nonoverlapping(reply.bytes.as_ptr(), out, reply.len);
    }
    reply.len as i32
}

/// Read-only controller observation. No allocation, mutation or borrowed pointers.
#[repr(C)]
pub struct Observation {
    pub version: u32,
    pub source: u32,
    pub target_attitude_valid: u32,
    pub reserved: u32,
    pub estimate: [f32; 4],
    pub rates: [f32; 3],
    pub target_rates: [f32; 3],
    pub target_attitude: [f32; 4],
}
/// # Safety
/// out must be a valid aligned writable Observation for this call and must not
/// alias registry-owned storage.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flix_observe(handle: u64, out: *mut Observation) -> i32 {
    if out.is_null() || (out as usize) % align_of::<Observation>() != 0 {
        return -3;
    }
    let Ok(registry) = REGISTRY.lock() else {
        return -1;
    };
    let Some((_, owner)) = registry
        .slots
        .iter()
        .flatten()
        .find(|(id, _)| *id == handle)
    else {
        return -2;
    };
    let snapshot = owner.flight.snapshot();
    let r = snapshot.estimate.rates;
    let target = snapshot.target_rates;
    // SAFETY: caller supplies writable aligned extent; no pointer escapes.
    unsafe {
        out.write(Observation {
            version: 1,
            source: snapshot.source.map_or(u32::MAX, |v| v as u32),
            target_attitude_valid: u32::from(snapshot.target_attitude.is_some()),
            reserved: 0,
            estimate: snapshot.estimate.attitude.array(),
            rates: [r.0, r.1, r.2],
            target_rates: [target.0, target.1, target.2],
            target_attitude: snapshot.target_attitude.map_or([0.; 4], |q| q.array()),
        });
    }
    0
}
