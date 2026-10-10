#![no_std]
#![forbid(unsafe_code)]
// Panics in the portable core are visible, reasoned exceptions only.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented,
    clippy::unreachable,
    clippy::dbg_macro,
    clippy::print_stdout,
    clippy::print_stderr
)]

pub mod battery;
pub mod control;
pub mod log;
pub mod math;
pub mod rc;
pub mod safety;
pub mod time;
pub use math::{Quaternion, Vec3};
pub use safety::*;
pub use time::Tick;
