//! Rust bare metal run-time support for the AMD Zynq 7000 SoCs
//!
//! This includes basic low-level startup code similar to the bare-metal boot routines
//! [provided by Xilinx](https://github.com/Xilinx/embeddedsw/tree/master/lib/bsp/standalone/src/arm/cortexa9/gcc).
#![no_std]
#[cfg(feature = "rt")]
pub use cortex_a_rt::*;

#[cfg(feature = "rt")]
use zynq_mmu::L1TableWrapper;

pub mod mmu;
#[cfg(feature = "rt")]
mod mmu_table;
#[cfg(feature = "rt")]
pub mod rt;

/// Retrieves a mutable reference to the MMU L1 page table.
#[cfg(feature = "rt")]
pub fn mmu_l1_table_mut() -> L1TableWrapper<'static> {
    let mmu_table = mmu_table::MMU_L1_PAGE_TABLE.0.get();
    // Safety: We retrieve a reference to the MMU page table singleton.
    L1TableWrapper::new(unsafe { &mut *mmu_table })
}
