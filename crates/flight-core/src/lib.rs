#![no_std]
#![forbid(unsafe_code)]

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
