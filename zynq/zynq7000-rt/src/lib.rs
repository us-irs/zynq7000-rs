//! # Rust bare metal run-time support for the AMD Zynq 7000 SoCs
//!
//! Startup code and minimal runtime for the AMD Zynq7000 SoC to write bare metal Rust code.
//! This run-time crate is strongly based on the
//! [startup code provided by AMD](https://github.com/Xilinx/embeddedsw/blob/master/lib/bsp/standalone/src/arm/cortexa9/gcc/boot.S).
//!
//! It mostly builds on [aarch32-rt](https://github.com/rust-embedded/aarch32/tree/main/aarch32-rt).
//! It activates the `fpu-d32` feature on that crate and overrides the `_default_start` method
//! to add necessary setup code for the Zynq7000. It re-exports the `aarch32-rt` crate, including
//! the attributes macros. The [documentation](https://docs.rs/aarch32-rt/latest/aarch32_rt/) specifies
//! these in detail.
//!
//! Some major differences to the startup code provided by AMD:
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
