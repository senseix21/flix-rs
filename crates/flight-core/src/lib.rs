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
// Lossy numeric conversions and exact float equality must state why they are sound.
#![deny(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::float_cmp
)]

pub mod battery;
pub mod control;
pub mod log;
pub mod math;
pub mod position;
#[cfg(kani)]
mod proofs;
pub mod rc;
pub mod safety;
pub mod time;
pub use math::{Quaternion, Vec3};
pub use safety::*;
pub use time::Tick;
