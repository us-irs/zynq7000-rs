//! Rust bare metal run-time support for the AMD Zynq 7000 SoCs
//!
//! This includes basic low-level startup code similar to the bare-metal boot routines
//! [provided by Xilinx](https://github.com/Xilinx/embeddedsw/tree/master/lib/bsp/standalone/src/arm/cortexa9/gcc).
#![no_std]

use zynq_mmu::L1TableRef;

pub mod mmu;
#[cfg(feature = "rt")]
mod mmu_table;
#[cfg(feature = "rt")]
pub mod rt;

/// Retrieves a mutable reference to the MMU L1 page table.
pub fn mmu_l1_table_mut() -> L1TableRef<'static> {
    // Safety: We retrieve a reference to the MMU page table singleton.
    L1TableRef::new(unsafe { &mut *mmu_table::MMU_L1_PAGE_TABLE.get() })
}
