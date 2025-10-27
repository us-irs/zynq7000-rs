//! # Rust bare metal run-time support for the AMD Zynq 7000 SoCs
//!
//! This includes basic low-level startup code similar to the bare-metal boot routines
//! [provided by Xilinx](https://github.com/Xilinx/embeddedsw/tree/master/lib/bsp/standalone/src/arm/cortexa9/gcc).
//! Some major differences:
//!
//! - No L2 cache initialization is performed.
//! - MMU table is specified as Rust code.
//! - Modification to the stack setup code, because a different linker script is used.
#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]
#[cfg(all(feature = "rt", arm_profile = "a"))]
pub use aarch32_rt::*;

#[cfg(feature = "rt")]
use zynq7000_mmu::L1TableWrapper;

pub mod mmu;
#[cfg(all(feature = "rt", arm_profile = "a"))]
mod mmu_table;
#[cfg(all(feature = "rt", arm_profile = "a"))]
pub mod rt;

/// Retrieves a mutable reference to the MMU L1 page table.
#[cfg(all(feature = "rt", arm_profile = "a"))]
pub fn mmu_l1_table_mut() -> L1TableWrapper<'static> {
    let mmu_table = mmu_table::MMU_L1_PAGE_TABLE.0.get();
    // Safety: We retrieve a reference to the MMU page table singleton.
    L1TableWrapper::new(unsafe { &mut *mmu_table })
}
