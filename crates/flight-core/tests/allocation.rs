use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
// Count only this thread: the test harness may allocate on other threads.
thread_local! {
    static RECORD: Cell<bool> = const { Cell::new(false) };
    static COUNT: Cell<usize> = const { Cell::new(0) };
}
struct Allocator;
// SAFETY: every request is forwarded unchanged to `System`; counting only
// touches thread-local cells and never allocates.
unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = RECORD.try_with(|record| {
            if record.get() {
                let _ = COUNT.try_with(|count| count.set(count.get() + 1));
            }
        });
        // SAFETY: forwards this method's own contract to `System`.
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        // SAFETY: forwards this method's own contract to `System`.
        unsafe { System.dealloc(p, l) }
    }
}
#[global_allocator]
static ALLOCATOR: Allocator = Allocator;
#[test]
fn flight_ticks_do_not_allocate() {
    use flight_core::{Flight, Sensor, Tick, Vec3};
    let mut flight = Flight::default();
    let sample = Sensor {
        at: Tick(0),
        gyro: Vec3::ZERO,
        acceleration: Vec3(0.0, 0.0, 9.80665),
        calibrated: true,
    };
    flight.tick(Tick(0), Some(sample));
    flight.select(flight_core::Source::Pilot);
    let mut p = flight_core::control::Pilot {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        thrust: 0.0,
    };
    flight.command(
        flight_core::Source::Pilot,
        flight_core::control::Intent::Stabilized(p),
        Tick(0),
    );
    assert!(flight.arm(Tick(0)));
    p.thrust = 0.4;
    COUNT.set(0);
    RECORD.set(true);
    for i in 1..10_000 {
        let at = Tick(i * 1000);
        flight.command(
            flight_core::Source::Pilot,
            flight_core::control::Intent::Stabilized(p),
            at,
        );
        let frame = flight.tick(
            at,
            Some(Sensor {
                at,
                gyro: Vec3::ZERO,
                acceleration: Vec3(0.0, 0.0, 9.80665),
                calibrated: true,
            }),
        );
        std::hint::black_box(flight.commit(frame, at));
    }
    RECORD.set(false);
    assert_eq!(COUNT.get(), 0);
    assert!(std::mem::size_of::<Flight>() <= 8192);
}
