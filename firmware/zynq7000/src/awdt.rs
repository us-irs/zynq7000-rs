//! # CPU private watchdog timer module.
//!
//! Each Cortex-A9 CPU has its own private watchdog module.
//! The watchdog can run in timer mode, functionally identical to the private timer, raising an
//! interrupt on underflow. It can also run in watchdog mode, where an underflow instead asserts a
//! per-CPU reset request. Once [crate::awdt::Control::watchdog_mode] is set, it can only be
//! cleared again through the magic-value sequence on the watchdog disable register.
//!
//! SLCR provides the matching reset routing: see [crate::slcr::reset::WatchTimerResetControl].

/// Base address of the CPU private watchdog register block.
pub const CPU_AWDT_BASE_ADDR: usize = super::mpcore::MPCORE_BASE_ADDR + 0x0000_0620;

pub use types::*;

/// Register helper types.
pub mod types {
    /// CPU private watchdog control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Control {
        /// Clock prescaler applied to the counter.
        #[bits(8..=15, rw)]
        prescaler: u8,
        /// Selects watchdog mode. Writing 0 has no effect, writing 1 enables watchdog mode.
        /// Once set, this bit can only be cleared again through the magic-value sequence on
        /// the watchdog disable register.
        #[bit(3, rw)]
        watchdog_mode: bool,
        /// Enables the timeout interrupt. Ignored in watchdog mode.
        #[bit(2, rw)]
        interrupt_enable: bool,
        /// Reloads the counter from the load register on underflow instead of stopping.
        #[bit(1, rw)]
        auto_reload: bool,
        /// Enables the counter.
        #[bit(0, rw)]
        enable: bool,
    }

    /// CPU private watchdog interrupt status. Only relevant in timer mode.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptStatus {
        /// Set when the counter reaches zero in timer mode. Sticky, write 1 to clear.
        #[bit(0, rw)]
        event_flag: bool,
    }

    /// CPU private watchdog reset status. Only relevant in watchdog mode.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ResetStatus {
        /// Set when the counter reaches zero in watchdog mode and a reset request was sent.
        /// Sticky, write 1 to clear. Unlike a normal CPU reset, this flag survives it, so it
        /// can be used after boot to distinguish a watchdog-triggered reset from a normal
        /// one.
        #[bit(0, rw)]
        reset_flag: bool,
    }
}

/// CPU private watchdog register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Load value, copied into the counter register on start, on auto-reload, or on a write
    /// to this register.
    load: u32,
    /// Current counter value. A decrementing counter, counts down while [Control::enable] is
    /// set.
    counter: u32,
    /// Watchdog control.
    control: Control,
    /// Watchdog interrupt status.
    interrupt_status: InterruptStatus,
    /// Watchdog reset status.
    reset_status: ResetStatus,
    /// Write 0x1234_5678 followed by 0x8765_4321, with no other access in between, to clear
    /// [Control::watchdog_mode] and return to timer mode. Any other value, or an intervening
    /// access, aborts the sequence and leaves the watchdog in its current mode.
    #[mmio(Write)]
    disable: u32,
}

impl Registers {
    /// Create a new CPU Private Watchdog MMIO instance at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    ///
    /// It should also be noted that the calls to this MMIO structure are private for each CPU
    /// core, which might lead to unexpected results when using this in a SMP system.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Registers::new_mmio_at(CPU_AWDT_BASE_ADDR) }
    }
}
