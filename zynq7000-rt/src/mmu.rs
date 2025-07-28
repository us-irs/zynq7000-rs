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
    use arbitrary_int::u4;
    use cortex_ar::mmu::{
        AccessPermissions, CacheableMemoryAttribute, MemoryRegionAttributes, SectionAttributes,
    };

    pub const DEFAULT_DOMAIN: u4 = u4::new(0b0000);
    // DDR is in different domain, but all domains are set as manager domains during run-time
    // initialization.
    pub const DDR_DOMAIN: u4 = u4::new(0b1111);

    pub const DDR: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: true,
        access: AccessPermissions::FullAccess,
        // Manager domain
        domain: DDR_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::CacheableMemory {
            inner: CacheableMemoryAttribute::WriteBackWriteAlloc,
            outer: CacheableMemoryAttribute::WriteBackWriteAlloc,
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
    pub const QSPI_XIP: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: false,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::OuterAndInnerWriteThroughNoWriteAlloc.as_raw(),
    };
    pub const OCM_MAPPED_HIGH: SectionAttributes = SectionAttributes {
        non_global: false,
        p_bit: false,
        shareable: false,
        access: AccessPermissions::FullAccess,
        domain: DEFAULT_DOMAIN,
        execute_never: false,
        memory_attrs: MemoryRegionAttributes::CacheableMemory {
            inner: CacheableMemoryAttribute::WriteThroughNoWriteAlloc,
            outer: CacheableMemoryAttribute::NonCacheable,
        }
        .as_raw(),
    };
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

/// Load the MMU translation table base address into the MMU.
///
/// # Safety
///
/// This function is unsafe because it directly writes to the MMU related registers. It has to be
/// called once in the boot code before enabling the MMU, and it should be called while the MMU is
/// disabled.
#[unsafe(no_mangle)]
#[cfg(feature = "rt")]
unsafe extern "C" fn load_mmu_table() {
    let table_base = crate::mmu_table::MMU_L1_PAGE_TABLE.0.get() as u32;

    unsafe {
        core::arch::asm!(
            "orr {0}, {0}, #0x5B",     // Outer-cacheable, WB
            "mcr p15, 0, {0}, c2, c0, 0", // Load table pointer
            inout(reg) table_base => _,
            options(nostack, preserves_flags)
        );
    }
}
