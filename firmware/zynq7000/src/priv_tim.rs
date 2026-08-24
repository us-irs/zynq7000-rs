//! # CPU private timer module.

/// Base address of the CPU private timer register block.
pub const CPU_PRIV_TIM_BASE_ADDR: usize = super::mpcore::MPCORE_BASE_ADDR + 0x0000_0600;

/// CPU private timer control.
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
    /// Enables the timer interrupt.
    #[bit(2, rw)]
    interrupt_enable: bool,
    /// Reloads the counter from the load register on underflow instead of stopping.
    #[bit(1, rw)]
    auto_reload: bool,
    /// Enables the timer.
    #[bit(0, rw)]
    enable: bool,
}

/// CPU private timer interrupt status.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct InterruptStatus {
    /// Cleared by writing a one.
    #[bit(0, rw)]
    event_flag: bool,
}

/// CPU private timer register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Load value, copied into the counter on start or auto-reload
    reload: u32,
    /// Current counter value
    counter: u32,
    /// Timer control
    control: Control,
    /// Timer interrupt status
    interrupt_status: InterruptStatus,
}

impl Registers {
    /// Create a new CPU Private Timer MMIO instance at the fixed base address.
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
        unsafe { Registers::new_mmio_at(CPU_PRIV_TIM_BASE_ADDR) }
    }
}
