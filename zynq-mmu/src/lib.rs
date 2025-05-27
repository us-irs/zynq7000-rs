//! The MMU structures live inside a dedicated shared crate so it can be used by both the Zynq
//! runtime crate and teh HAL crate.
#![no_std]

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

#[repr(C, align(16384))]
pub struct L1Table(pub [u32; NUM_L1_PAGE_TABLE_ENTRIES]);

impl L1Table {
    #[inline(always)]
    pub const fn as_ptr(&self) -> *const u32 {
        self.0.as_ptr()
    }

    #[inline(always)]
    pub const fn as_mut_ptr(&mut self) -> *mut u32 {
        self.0.as_mut_ptr()
    }

    pub fn update(
        &mut self,
        addr: u32,
        section_attrs: SectionAttributes,
    ) -> Result<(), AddrNotAlignedToOneMb> {
        if addr & 0x000F_FFFF != 0 {
            return Err(AddrNotAlignedToOneMb);
        }
        let index = addr as usize / 0x10_0000;
        self.0[index] = (self.0[index] & 0xFFF0_0000) | section_attrs.raw();

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

pub struct L1TableRef<'a>(pub &'a mut L1Table);

impl<'a> L1TableRef<'a> {
    pub fn new(l1_table: &'a mut L1Table) -> L1TableRef<'a> {
        L1TableRef(l1_table)
    }
}

impl L1TableRef<'_> {
    pub fn update(
        &mut self,
        addr: u32,
        section_attrs: SectionAttributes,
    ) -> Result<(), AddrNotAlignedToOneMb> {
        self.0.update(addr, section_attrs)
    }
}
