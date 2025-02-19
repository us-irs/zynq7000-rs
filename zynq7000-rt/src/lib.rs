//! Rust bare metal run-time support for the AMD Zynq 7000 SoCs
//!
//! This includes basic low-level startup code similar to the bare-metal boot routines
//! [provided by Xilinx](https://github.com/Xilinx/embeddedsw/tree/master/lib/bsp/standalone/src/arm/cortexa9/gcc).
#![no_std]

pub mod mmu;
#[cfg(feature = "rt")]
mod mmu_table;
#[cfg(feature = "rt")]
pub mod rt;
