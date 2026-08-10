#![no_std]
//! Pure clock/divisor search calculators used by the Zynq-7000 HAL.
//!
//! These calculators are split out of `zynq7000-hal` into their own crate specifically so
//! they can be unit-tested with a plain `cargo test` on the host. `zynq7000-hal` defaults
//! to the `armv7a-none-eabihf` target and transitively depends on crates (e.g.
//! `aarch32-cpu`'s MMU/coprocessor support) that are gated to ARM targets only, so it
//! cannot be built for - and therefore cannot run its own test suite on - the host.
//! This crate has no such dependencies, so it builds and tests the same way on any target.

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod fpga;
pub mod uart;
