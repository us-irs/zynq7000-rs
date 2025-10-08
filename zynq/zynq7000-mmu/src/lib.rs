//! # Zynq7000 Memory Management Unit (MMU) 
//!
//! Dedicated shared crate for Zynq7000 MMU abstractions which can be used by Zynq
//! runtime crates, PACs and HALs.
#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

use core::cell::UnsafeCell;
use cortex_ar::mmu::L1Section;
#[cfg(not(feature = "tools"))]
use cortex_ar::{
    asm::{dsb, isb},
    cache::clean_and_invalidate_l1_data_cache,
    mmu::SectionAttributes,
    register::{BpIAll, TlbIAll},
};

pub const NUM_L1_PAGE_TABLE_ENTRIES: usize = 4096;

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
#[error("address is not aligned to 1MB boundary")]
pub struct AddrNotAlignedToOneMb;

/// Raw L1 table wrapper.
///
/// You can use [L1Table] to create a static global L1 table, which can be shared and updated
/// without requiring a static mutable global.
#[repr(C, align(16384))]
pub struct L1TableRaw(pub [L1Section; NUM_L1_PAGE_TABLE_ENTRIES]);

impl L1TableRaw {
    #[inline(always)]
    pub const fn as_ptr(&self) -> *const u32 {
        self.0.as_ptr() as *const _
    }

    #[inline(always)]
    pub const fn as_mut_ptr(&mut self) -> *mut u32 {
        self.0.as_mut_ptr() as *mut _
    }

    #[cfg(not(feature = "tools"))]
    pub fn update(
        &mut self,
        addr: u32,
        section_attrs: SectionAttributes,
    ) -> Result<(), AddrNotAlignedToOneMb> {
        if addr & 0x000F_FFFF != 0 {
            return Err(AddrNotAlignedToOneMb);
        }
        let index = addr as usize / 0x10_0000;
        self.0[index].set_section_attrs(section_attrs);

        // The Zynq 7000 has a 32 kB 4-way associative cache with a line length of 32 bytes.
        // 4-way associative cache: A == 2
        // 32 bytes line length: N == 5
        // 256 (32kB / (32 * 4)) sets: S == 8
        clean_and_invalidate_l1_data_cache::<2, 5, 8>();
        TlbIAll::write();
        BpIAll::write();
        dsb();
        isb();

        Ok(())
    }
}

/// This is a thin helper structure to allow declaring one static global L1 table
/// while also allowing mutable access to it without requiring static mutables.
///
/// The L1 table is usually expected as some data structure at a certain address which can be
/// declared with initial values and placed inside the .data section.
#[repr(transparent)]
pub struct L1Table(pub UnsafeCell<L1TableRaw>);

unsafe impl Sync for L1Table {}

impl L1Table {
    #[inline]
    pub const fn new(l1_table: [L1Section; NUM_L1_PAGE_TABLE_ENTRIES]) -> L1Table {
        L1Table(UnsafeCell::new(L1TableRaw(l1_table)))
    }
}

/// Wrapper structure to modify the L1 table given a mutable reference to the table.
pub struct L1TableWrapper<'a>(pub &'a mut L1TableRaw);

impl<'a> L1TableWrapper<'a> {
    pub fn new(l1_table: &'a mut L1TableRaw) -> L1TableWrapper<'a> {
        L1TableWrapper(l1_table)
    }
}

impl L1TableWrapper<'_> {
    #[cfg(not(feature = "tools"))]
    pub fn update(
        &mut self,
        addr: u32,
        section_attrs: SectionAttributes,
    ) -> Result<(), AddrNotAlignedToOneMb> {
        self.0.update(addr, section_attrs)
    }
}
