//! # Cache management module
//!
//! A lot of cache maintenance operations for this SoC have to be performed on both the L1 and the
//! L2 cache in the correct order. This module provides commonly required operations.
use core::sync::atomic::compiler_fence;

use aarch32_cpu::{
    asm::dsb,
    cache::{
        clean_and_invalidate_data_cache_line_to_poc, clean_data_cache_line_to_poc,
        invalidate_data_cache_line_to_poc,
    },
};
use zynq7000::l2_cache::{MmioRegisters, Registers};

pub const CACHE_LINE_SIZE: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("alignment error, addresses and lengths must be aligned to 32 byte cache line length")]
pub struct AlignmentError;

pub fn clean_and_invalidate_l2c_line(l2c: &mut MmioRegisters<'static>, addr: u32) {
    l2c.write_clean_by_pa(addr);
    l2c.write_invalidate_by_pa(addr);
}

/// Cleans and invalidates the full L1 and L2 cache.
pub fn clean_and_invalidate_data_cache() {
    dsb();

    aarch32_cpu::cache::clean_l1_data_cache::<2, 5, 8>();
    dsb();

    // Clean all ways in L2 cache.
    let mut l2c = unsafe { Registers::new_mmio_fixed() };
    l2c.write_clean_invalidate_by_way(0xffff);
    while l2c.read_cache_sync().busy() {}
    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    aarch32_cpu::cache::clean_and_invalidate_l1_data_cache::<2, 5, 8>();
    dsb();
}

/// Invalidate an address range.
///
/// This function invalidates both the L1 and L2 cache. The L2C must be enabled and set up
/// correctly for this function to work correctly.
///
/// The provided address and the range to invalidate must both be aligned to the 32 byte cache line
/// length.
pub fn invalidate_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if !addr.is_multiple_of(CACHE_LINE_SIZE as u32) || !len.is_multiple_of(CACHE_LINE_SIZE) {
        return Err(AlignmentError);
    }
    let mut current_addr = addr;
    let end_addr = addr.saturating_add(len as u32);
    let mut l2c = unsafe { Registers::new_mmio_fixed() };

    dsb();
    // Invalidate outer caches lines first, see chapter 3.3.10 of the L2C technical reference
    // manual.
    while current_addr < end_addr {
        l2c.write_invalidate_by_pa(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    while l2c.read_cache_sync().busy() {}

    // Invalidate inner cache lines.
    current_addr = addr;
    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    while current_addr < end_addr {
        invalidate_data_cache_line_to_poc(addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    // Synchronize the cache maintenance.
    dsb();
    Ok(())
}

/// Clean and then invalidate an address range.
///
/// This is commonly also called cache flushing. This function cleans and invalidates both L1
/// and L2 cache. The L2C must be enabled and set up correctly for this function to work correctly.
///
/// Both the address and length to clean and invalidate must be a multiple of the 32 byte cache
/// line.
pub fn clean_and_invalidate_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if !addr.is_multiple_of(CACHE_LINE_SIZE as u32) || !len.is_multiple_of(CACHE_LINE_SIZE) {
        return Err(AlignmentError);
    }
    let end_addr = addr.saturating_add(len as u32);
    let mut current_addr = addr;
    dsb();

    // For details on the following section, see chapter 3.3.10 of the L2C technical reference
    // manual.
    // Clean inner cache lines first.
    while current_addr < end_addr {
        clean_data_cache_line_to_poc(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    dsb();

    // Clean and invalidates outer cache.
    let mut l2c = unsafe { Registers::new_mmio_fixed() };
    current_addr = addr;
    while current_addr < end_addr {
        // ARM errate 588369 specifies that clean and invalidate need to be separate, but the
        // current revision of the L2C on the Zynq7000 seems to be revision 8 (r3p2), and the
        // errata was fixed in r2p0. Both Xilinx and zynq-rs use the clean and invalidate operation,
        // so it should be fine. Considering the debug control in Xilinx code which disable
        // linefills and write-backs, zynq-rs does not appear to do that and it should not be
        // necessary.. I think this was related to the errata.
        l2c.write_clean_invalidate_by_pa(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    while l2c.read_cache_sync().busy() {}

    // Now clean and invalidate inner cache.
    current_addr = addr;
    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    while current_addr < end_addr {
        clean_and_invalidate_data_cache_line_to_poc(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    dsb();
    Ok(())
}

/// Cleans an address range.
///
/// This function cleans and invalidates both L1
/// and L2 cache. The L2C must be enabled and set up correctly for this function to work correctly.
///
/// Both the address and length to clean and invalidate must be a multiple of the 32 byte cache
/// line.
pub fn clean_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if !addr.is_multiple_of(32) || !len.is_multiple_of(32) {
        return Err(AlignmentError);
    }

    let end_addr = addr.saturating_add(len as u32);
    let mut current_addr = addr;
    dsb();

    // For details on the following section, see chapter 3.3.10 of the L2C technical reference
    // manual.
    // Clean inner cache lines first.
    while current_addr < end_addr {
        clean_data_cache_line_to_poc(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    dsb();

    // Clean and invalidates outer cache.
    let mut l2c = unsafe { Registers::new_mmio_fixed() };
    current_addr = addr;
    while current_addr < end_addr {
        l2c.write_clean_by_pa(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    while l2c.read_cache_sync().busy() {}
    compiler_fence(core::sync::atomic::Ordering::SeqCst);
    Ok(())
}
