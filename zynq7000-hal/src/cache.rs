use cortex_ar::{
    asm::dsb,
    cache::{
        clean_and_invalidate_data_cache_line_to_poc, clean_data_cache_line_to_poc,
        invalidate_data_cache_line_to_poc,
    },
};
use zynq7000::l2_cache::{L2Cache, MmioL2Cache};

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("alignment error, addresses and lengths must be aligned to 32 byte cache line length")]
pub struct AlignmentError;

pub fn clean_and_invalidate_l2c_line(l2c: &mut MmioL2Cache<'static>, addr: u32) {
    l2c.write_clean_by_pa(addr);
    l2c.write_invalidate_by_pa(addr);
}

/// Invalidate an address range.
///
/// This function invalidates both the L1 and L2 cache. The L2C must be enabled and set up
/// correctly for this function to work correctly.
///
/// The provided address and the range to invalidate must both be aligned to the 32 byte cache line
/// length.
pub fn invalidate_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if addr % 32 != 0 || len % 32 != 0 {
        return Err(AlignmentError);
    }
    let mut l2c = unsafe { L2Cache::new_mmio_fixed() };
    let mut current_addr = addr;
    // Invalidate outer caches lines first, see chapter 3.3.10 of the L2C technical reference manual.
    while current_addr < addr + len as u32 {
        l2c.write_clean_by_pa(current_addr);
        current_addr += 32;
    }
    while l2c.read_cache_sync().busy() {}

    // Invalidate inner cache lines.
    current_addr = addr;
    while current_addr < addr + len as u32 {
        invalidate_data_cache_line_to_poc(addr);
        current_addr += 32;
    }
    // Synchronize the cache maintenance.
    dsb();
    Ok(())
}

/// Clean and then invalidate an address range.
///
/// This is commonly also called cache flushing. This function cleans and invalidates both L1
/// and L2 cache. The L2C must be enabled and set up correctly for this function to work correctly.
pub fn clean_and_invalidate_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if addr % 32 != 0 || len % 32 != 0 {
        return Err(AlignmentError);
    }
    // For details on the following section, see chapter 3.3.10 of the L2C technical reference
    // manual.
    // Clean inner cache lines first.
    let mut current_addr = addr;
    while current_addr < addr + len as u32 {
        clean_data_cache_line_to_poc(current_addr);
        current_addr += 32;
    }
    dsb();

    // Clean and invalidates outer cache.
    let mut l2c = unsafe { L2Cache::new_mmio_fixed() };
    current_addr = addr;
    while current_addr < addr + len as u32 {
        // ARM errate 588369 specifies that clean and invalidate need to be separate, but the
        // current revision of the L2C on the Zynq7000 seems to be revision 8 (r3p2), and the
        // errata was fixed in r2p0. Both Xilinx and zynq-rs use the clean and invalidate operation,
        // so it should be fine. Considering the debug control in Xilinx code which disable
        // linefills and write-backs, zynq-rs does not appear to do that and it should not be
        // necessary.. I think this was related to the errata.
        l2c.write_clean_invalidate_by_pa(current_addr);
        current_addr += 32;
    }
    while l2c.read_cache_sync().busy() {}

    // Now clean and invalidate inner cache.
    current_addr = addr;
    while current_addr < addr + len as u32 {
        clean_and_invalidate_data_cache_line_to_poc(current_addr);
        current_addr += 32;
    }
    dsb();

    Ok(())
}
