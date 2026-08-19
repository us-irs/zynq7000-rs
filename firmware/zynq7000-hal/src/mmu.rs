//! The overview of translation table memory attributes is described below.
//!
//!|                       | Memory Range            | Definition in Translation Table   |
//!|-----------------------|-------------------------|-----------------------------------|
//!| DDR                   | 0x00000000 - 0x3FFFFFFF | Normal write-back Cacheable       |
//!| PL                    | 0x40000000 - 0xBFFFFFFF | Strongly Ordered                  |
//!| Reserved              | 0xC0000000 - 0xDFFFFFFF | Unassigned                        |
//!| Memory mapped devices | 0xE0000000 - 0xE02FFFFF | Device Memory                     |
//!| Reserved              | 0xE0300000 - 0xE0FFFFFF | Unassigned                        |
//!| NAND, NOR             | 0xE1000000 - 0xE3FFFFFF | Device memory                     |
//!| SRAM                  | 0xE4000000 - 0xE5FFFFFF | Normal write-back Cacheable       |
//!| Reserved              | 0xE6000000 - 0xF7FFFFFF | Unassigned                        |
//!| AMBA APB Peripherals  | 0xF8000000 - 0xF8FFFFFF | Device Memory                     |
//!| Reserved              | 0xF9000000 - 0xFBFFFFFF | Unassigned                        |
//!| Linear QSPI - XIP     | 0xFC000000 - 0xFDFFFFFF | Normal write-through cacheable    |
//!| Reserved              | 0xFE000000 - 0xFFEFFFFF | Unassigned                        |
//!| OCM                   | 0xFFF00000 - 0xFFFFFFFF | Normal inner write-back cacheable |
//!
//! For region 0x00000000 - 0x3FFFFFFF, a system where DDR is less than 1 GB,
//! region after DDR and before PL is marked as undefined/reserved in translation
//! table. In 0xF8000000 - 0xF8FFFFFF, 0xF8000C00 - 0xF8000FFF, 0xF8010000 -
//! 0xF88FFFFF and 0xF8F03000 to 0xF8FFFFFF are reserved  but due to granual size
//! of 1 MB, it is not possible to define separate regions for them. For region
//! 0xFFF00000 - 0xFFFFFFFF, 0xFFF00000 to 0xFFFB0000 is reserved but due to 1MB
//! granual size, it is not possible to define separate region for it.
//!
//! ## Cache maintenance
//!
//! Placing a buffer in OCM does **not** exempt it from cache maintenance if it's used as a DMA
//! target/source. The `OCM` MMU attribute (see [`section_attrs::OCM`]) is inner
//! write-back-cacheable, outer non-cacheable: the L1 (inner) cache still caches it exactly like
//! DDR does, so DMA engines (which write to/read from physical memory directly, bypassing the
//! cache) can still see stale data or have their writes clobbered by a later cache write-back,
//! same as for any other cacheable memory. The difference from DDR is only that the L2 (outer)
//! half of cache maintenance is unnecessary for OCM, since the L2 never caches it in the first
//! place, use the `_inner`-only variants in [`crate::cache`] (e.g.
//! [`crate::cache::invalidate_data_cache_range_inner`],
//! [`crate::cache::clean_data_cache_range_inner`]) for OCM buffers instead of the full
//! inner+outer functions used for DDR buffers, to skip that wasted L2 work. See the `zedboard`
//! `axi-dma` example for a side-by-side comparison of both.

use aarch32_cpu::mmu::L1Section;
use aarch32_cpu::{
    asm::{dsb, isb},
    cache::clean_and_invalidate_l1_data_cache,
    mmu::SectionAttributes,
    register::{BpIAll, TlbIAll},
};
use core::cell::UnsafeCell;

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
    pub fn update(
        &mut self,
        addr: u32,
        section_attrs: SectionAttributes,
    ) -> Result<(), AddrNotAlignedToOneMb> {
        self.0.update(addr, section_attrs)
    }
}

/// Initialize the global MMU table.
///
/// # Safety
///
/// Must only be called ONCE per core during program initialization
pub unsafe fn init() {
    unsafe {
        set_mmu();
        enable_mmu_and_cache();
    }
}

/// Set the MMU base register to the global MMU table.
///
/// # Safety
///
/// Must only be called ONCE per core during program initialization
pub unsafe fn set_mmu() {
    let ttbr0 = aarch32_cpu::register::Ttbr0::new_with_raw_value(0)
        .with_address(super::mmu_table::MMU_L1_PAGE_TABLE.0.get() as usize)
        .with_irgn(false)
        .with_nos(false)
        .with_rgn(aarch32_cpu::register::ttbr0::Region::WriteBackWriteAllocCacheable)
        .with_s(true)
        .with_c(true);
    unsafe { aarch32_cpu::register::Ttbr0::write(ttbr0) }
}

/// Enable the MMU and the cache
///
/// # Safety
///
/// Must only be called ONCE per core during program initialization
pub unsafe fn enable_mmu_and_cache() {
    // Enable Manager access to Domain 0
    aarch32_cpu::register::Dacr::modify(|d| {
        d.set_d(0, aarch32_cpu::register::dacr::DomainAccess::Manager);
    });
    // This function contains the barrier we need to flush the pipeline
    aarch32_cpu::register::Sctlr::modify(|s| {
        // Enable I-Cache
        s.set_i(true);
        // Enable D-Cache
        s.set_c(true);
        // Enable MMU
        s.set_m(true);
    });
    aarch32_cpu::asm::dsb();
    aarch32_cpu::asm::isb();
}

/// Retrieves a mutable reference to the MMU L1 page table.
pub fn mmu_l1_table_mut() -> L1TableWrapper<'static> {
    let mmu_table = super::mmu_table::MMU_L1_PAGE_TABLE.0.get();
    // Safety: We retrieve a reference to the MMU page table singleton.
    L1TableWrapper::new(unsafe { &mut *mmu_table })
}

pub const MAX_DDR_SIZE: usize = 0x4000_0000;
pub const ONE_MB: usize = 0x10_0000;

pub mod offsets {
    pub const OFFSET_DDR: usize = 0;
    pub const OFFSET_DDR_ALL_ACCESSIBLE: usize = 0x10_0000;

    pub const OFFSET_FPGA_SLAVE_0: usize = 0x4000_0000;
    pub const OFFSET_FPGA_SLAVE_1_START: usize = 0x8000_0000;
    pub const OFFSET_FPGA_SLAVE_1_END: usize = 0xC000_0000;

    pub const OFFSET_IO_PERIPHERALS_START: usize = 0xE000_0000;
    pub const OFFSET_IO_PERIPHERALS_END: usize = 0xE030_0000;

    pub const OFFSET_NAND_MEMORY: usize = 0xE100_0000;
    pub const OFFSET_NOR_MEMORY: usize = 0xE200_0000;
    pub const OFFSET_SRAM_MEMORY: usize = 0xE400_0000;
    pub const OFFSET_SMC_MEMORIES_END: usize = 0xE600_0000;

    /// 0xf8000c00 to 0xf8000fff, 0xf8010000 to 0xf88fffff and
    /// 0xf8f03000 to 0xf8ffffff are reserved  but due to granual size of
    /// 1MB, it is not possible to define separate regions for them.
    pub const OFFSET_AMBA_APB_START: usize = 0xF800_0000;
    pub const OFFSET_AMBA_APB_END: usize = 0xF900_0000;

    pub const OFFSET_QSPI_XIP_START: usize = 0xFC00_0000;
    pub const OFFSET_QSPI_XIP_END: usize = 0xFE00_0000;

    /// 0xfff00000 to 0xfffb0000 is reserved but due to granual size of
    /// 1MB, it is not possible to define separate region for it
    pub const OFFSET_OCM_MAPPED_HIGH_START: usize = 0xFFF0_0000;
    pub const OFFSET_OCM_MAPPED_HIGH_END: u64 = 0x1_0000_0000;
}
pub mod segments {
    pub use super::offsets::*;
    use super::{MAX_DDR_SIZE, ONE_MB};

    /// First 1 MB of DDR has special treatment, access is dependant on SCU/OCM state.
    /// Refer to Zynq TRM UG585 p.106 for more details.
    pub const DDR_FULL_ACCESSIBLE: usize = (MAX_DDR_SIZE - ONE_MB) / ONE_MB;
    pub const FPGA_SLAVE: usize = (OFFSET_FPGA_SLAVE_1_START - OFFSET_FPGA_SLAVE_0) / ONE_MB;
    pub const UNASSIGNED_0: usize =
        (OFFSET_IO_PERIPHERALS_START - OFFSET_FPGA_SLAVE_1_END) / ONE_MB;
    pub const IO_PERIPHS: usize =
        (OFFSET_IO_PERIPHERALS_END - OFFSET_IO_PERIPHERALS_START) / ONE_MB;
    pub const UNASSIGNED_1: usize = (OFFSET_NAND_MEMORY - OFFSET_IO_PERIPHERALS_END) / ONE_MB;
    pub const NAND: usize = (OFFSET_NOR_MEMORY - OFFSET_NAND_MEMORY) / ONE_MB;
    pub const NOR: usize = (OFFSET_SRAM_MEMORY - OFFSET_NOR_MEMORY) / ONE_MB;
    pub const SRAM: usize = (OFFSET_SMC_MEMORIES_END - OFFSET_SRAM_MEMORY) / ONE_MB;
    pub const SEGMENTS_UNASSIGNED_2: usize =
        (OFFSET_AMBA_APB_START - OFFSET_SMC_MEMORIES_END) / ONE_MB;
    pub const AMBA_APB: usize = (OFFSET_AMBA_APB_END - OFFSET_AMBA_APB_START) / ONE_MB;
    pub const UNASSIGNED_3: usize = (OFFSET_QSPI_XIP_START - OFFSET_AMBA_APB_END) / ONE_MB;
    pub const QSPI_XIP: usize = (OFFSET_QSPI_XIP_END - OFFSET_QSPI_XIP_START) / ONE_MB;
    pub const UNASSIGNED_4: usize = (OFFSET_OCM_MAPPED_HIGH_START - OFFSET_QSPI_XIP_END) / ONE_MB;
    pub const OCM_MAPPED_HIGH: usize = ((OFFSET_OCM_MAPPED_HIGH_END
        - OFFSET_OCM_MAPPED_HIGH_START as u64)
        / ONE_MB as u64) as usize;
}

pub mod section_attrs {
    use aarch32_cpu::mmu::{
        AccessPermissions, CachePolicy, MemoryRegionAttributes, SectionAttributes,
    };
    use arbitrary_int::u4;

    pub const DEFAULT_DOMAIN: u4 = u4::new(0b0000);

    pub const DDR: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: true,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::CacheableMemory {
            inner: CachePolicy::WriteBackWriteAlloc,
            outer: CachePolicy::WriteBackWriteAlloc,
        }
        .as_raw(),
    };
    pub const FPGA_SLAVES: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: false,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::StronglyOrdered.as_raw(),
    };
    pub const SHAREABLE_DEVICE: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: false,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::ShareableDevice.as_raw(),
    };
    pub const SRAM: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: false,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::OuterAndInnerWriteBackNoWriteAlloc.as_raw(),
    };
    /// For the QSPI XIP, we profit from caching reads to both inner and outer cache.
    /// Writes are not relevant, because the QSPI controller does not support writes in linear
    /// addressing mode. The TRM mentions that the AXI bus will immediately acknowledge the command
    /// but will not perform any actual operations. I think using write through without allocation
    /// prevents cache pollution for writes, but those should never happen anyway..
    pub const QSPI_XIP: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: false,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::OuterAndInnerWriteThroughNoWriteAlloc.as_raw(),
    };
    pub const OCM: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        // Matches the real ps7_init.tcl/Xilinx boot.S reference: the low OCM alias is marked
        // shareable there too, so this stays SCU snoop-coherent between the two cores' L1 caches.
        shareable: true,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::CacheableMemory {
            inner: CachePolicy::WriteBackWriteAlloc,
            outer: CachePolicy::NonCacheable,
        }
        .as_raw(),
    };
    #[cfg(feature = "first-segment-ddr-attr")]
    pub const FIRST_SEGMENT: SectionAttributes = DDR;
    #[cfg(not(feature = "first-segment-ddr-attr"))]
    pub const FIRST_SEGMENT: SectionAttributes = OCM;
    pub const UNASSIGNED_RESERVED: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: false,
        access: AccessPermissions::PermissionFault,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::StronglyOrdered.as_raw(),
    };
}
