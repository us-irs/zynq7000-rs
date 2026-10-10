//! Clock divisor search calculators.
//!
//! The `z7-clock-calc` host crate contains a verbatim copy of these modules with unit tests,
//! because this crate cannot be built for the host. `just sync-clock-calc` copies them over.

pub mod fpga;
pub mod uart;
