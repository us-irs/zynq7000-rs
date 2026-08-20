//! # Symmetric multi-processing (SMP) support.
//!
//! The Zynq7000 has two Cortex-A9 cores. The AMD BootROM code parks CPU1 inside an [`aarch32_cpu::asm::wfe`]
//! loop until user software writes a start address at the [`CPU1_ENTRY_MAILBOX_ADDR`]
//! address and then issues an [`aarch32_cpu::asm::sev`] instruction. Depending on how you boot
//! your system, you might also have to explicitly start CPU1 with your debugger.
//!
//! The run-time library provides [`start_core1`] to release CPU1 from regular Rust code
//! running on CPU0. When CPU1 is released, it runs a reduced version of the regular run-time
//! code. It sets up all the core-specific steps like the stack, but skips steps that only have
//! to be performed once, like `.bss` and `.data` segment initialization. After that, it jumps to
//! a user-defined `kmain_secondary` function instead of the regular `kmain`. The function
//! should have the following signature:
//!
//! ```rust
//! #[unsafe(no_mangle)]
//! pub extern "C" fn kmain_secondary() {
//!     // (user code here ..)
//! }
//! ```
//!
//! Your `memory.x` must set `PROVIDE(_num_cores = 2);`. `z7link.x` sizes every per-mode stack
//! region as `_xxx_stack_size * _num_cores`, and each core picks its own slice of that region
//! based on its MPIDR index. Without this, the stack region stays sized for one core, and CPU1's
//! stack pointer ends up at the bottom of CPU0's stack area instead of its own, so both cores
//! silently corrupt each other's stacks. See the `zedboard-smp` example's `memory.x` for a real
//! template.
//!
//! The MMU/cache enable and GIC setup for the new core are left
//! to the application. The `zynq7000_hal` library provides components to do this
//! and the `zedboard-smp` example shows a complete two-core setup.
//!
//! Using `critical_section` across both cores requires the [`aarch32_cpu`]
//! `critical-section-multi-core` feature. The default `critical-section-single-core`
//! implementation only disables IRQs on the current core, which does not provide real mutual
//! exclusion once a second core is running.
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
