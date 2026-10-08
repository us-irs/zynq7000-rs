#![no_std]
//! Pure clock/divisor search calculators of the Zynq-7000 HAL.
//!
//! `zynq7000-hal` cannot be built for the host, so its calculators cannot be unit-tested
//! there. This crate holds a verbatim copy of `zynq7000-hal/src/clock_calc` together with
//! the unit tests, which live in separate files so the copy stays verbatim.

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod fpga;
pub mod i2c;
pub mod uart;

#[cfg(test)]
mod fpga_tests;
#[cfg(test)]
mod i2c_tests;
#[cfg(test)]
mod uart_tests;
