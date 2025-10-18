//! # CPU private timer module.

pub const CPU_PRIV_TIM_BASE_ADDR: usize = super::mpcore::MPCORE_BASE_ADDR + 0x0000_0600;

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct Control {
    #[bits(8..=15, rw)]
    prescaler: u8,
    #[bit(2, rw)]
    interrupt_enable: bool,
    #[bit(1, rw)]
    auto_reload: bool,
    #[bit(0, rw)]
    enable: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct InterruptStatus {
    /// Cleared by writing a one.
    #[bit(0, rw)]
    event_flag: bool,
}

/// CPU private timer register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    reload: u32,
    counter: u32,
    control: Control,
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
