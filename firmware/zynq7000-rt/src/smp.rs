//! # Symmetric multi-processing (SMP) support.
//!
//! CPU1 is parked by the Zynq-7000 BootROM in a `WFE` loop, polling a fixed physical
//! address (`0xFFFFFFF0`) for its entry point. Per the TRM: "When CPU 1 receives a system
//! event, it immediately reads the contents of address 0xFFFFFFF0 and jumps to that address."
use aarch32_cpu::{
    asm::{dsb, sev},
    cache::clean_data_cache_line_to_poc,
};
use zynq7000::slcr::reset::CpuResetControl;

/// Fixed physical address the boot ROM polls.
pub const CPU1_ENTRY_MAILBOX_ADDR: u32 = 0xFFFF_FFF0;

/// Fixed physical address pointer the boot ROM polls for CPU1's entry point.
pub const CPU1_ENTRY_MAILBOX: *mut u32 = CPU1_ENTRY_MAILBOX_ADDR as *mut u32;

// Same unlock/lock keys `rt.rs`'s `check_efuse` path already uses (in raw assembly, since it
// runs before Rust/stacks are set up) to *assert* CPU1's reset on single-core-efuse silicon;
// here we do the inverse using the typed SLCR driver, since this runs from ordinary Rust code.
const SLCR_UNLOCK_KEY: u32 = 0xDF0D;
const SLCR_LOCK_KEY: u32 = 0x767B;

unsafe extern "C" {
    fn _start();
}

/// Releases CPU1 from the boot ROM's park loop, starting it at [`crate::rt`]'s `_start` (the
/// same entry point CPU0 used), which dispatches to `kmain_secondary` for any non-zero MPIDR
/// core.
///
/// Must only be called after CPU0 has finished all shared, one-time initialization (SLCR
/// access, L2 cache init, etc.) that CPU1 must not race, since CPU1 begins executing
/// immediately once released.
///
/// Returns the [`CpuResetControl`] value from before it was touched. This function clears
/// the reset bits of the CPU reset control. If the second core does not start properly,
/// you might need to explicitely run/release the core using your debugger.
pub fn start_core1() -> CpuResetControl {
    let prev_reset_control = unsafe {
        let mut slcr = zynq7000::slcr::Registers::new_mmio_fixed();
        slcr.write_unlock(SLCR_UNLOCK_KEY);
        let prev = slcr.reset_ctrl().read_a9_cpu();
        slcr.reset_ctrl()
            .modify_a9_cpu(|v| v.with_cpu1_reset(false).with_cpu1_clockstop(false));
        slcr.write_lock(SLCR_LOCK_KEY);
        prev
    };

    let entry = _start as *const () as u32;
    // Safety: `CPU1_ENTRY_MAILBOX` is a fixed physical mailbox address the boot ROM
    // guarantees is valid to write while CPU1 is parked.
    unsafe {
        CPU1_ENTRY_MAILBOX.write_volatile(entry);
    }
    // The mailbox address falls inside a cacheable MMU region, but CPU1 polls it with its
    // own MMU/cache still off, so the write has to be flushed out to physical memory before
    // it can observe it.
    clean_data_cache_line_to_poc(CPU1_ENTRY_MAILBOX as u32);
    dsb();
    sev();

    prev_reset_control
}

/// Default secondary-core entry point, used if the application does not define its own
/// `kmain_secondary`. Parks the core forever, so a build that never calls [`start_core1`]
/// behaves exactly as it did before SMP support was added.
#[unsafe(no_mangle)]
pub extern "C" fn _default_kmain_secondary() {
    loop {
        aarch32_cpu::asm::wfe();
    }
}
