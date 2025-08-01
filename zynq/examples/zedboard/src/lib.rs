#![no_std]
use zynq7000_hal::time::Hertz;

// Define the clock frequency as a constant
pub const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_333);
