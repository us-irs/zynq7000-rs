//! # Global timer counter module.

pub const GTC_BASE_ADDR: usize = super::mpcore::MPCORE_BASE_ADDR + 0x0000_0200;

#[bitbybit::bitfield(u32)]
pub struct GtcCtrl {
    #[bits(8..=15, rw)]
    prescaler: u8,
    #[bit(3, rw)]
    auto_increment: bool,
    #[bit(2, rw)]
    irq_enable: bool,
    #[bit(1, rw)]
    comparator_enable: bool,
    #[bit(0, rw)]
    enable: bool,
}

#[bitbybit::bitfield(u32)]
pub struct InterruptStatus {
    #[bit(0, rw)]
    event_flag: bool,
}

/// Global timer counter.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct GlobalTimerCounter {
    /// Count register 0, lower 32 bits
    count_lower: u32,
    /// Count register 1, upper 32 bits
    count_upper: u32,
    /// Control register
    ctrl: GtcCtrl,
    /// Interrupt status register
    #[mmio(PureRead, Write)]
    isr: InterruptStatus,
    /// Comparator 0, lower 32 bits
    comparator_lower: u32,
    /// Comparator 1, upper 32 bits
    comparator_upper: u32,
    /// Auto-increment register
    auto_increment: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<GlobalTimerCounter>(), 0x1C);

impl GlobalTimerCounter {
    /// Create a new GTC MMIO instance at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioGlobalTimerCounter<'static> {
        unsafe { GlobalTimerCounter::new_mmio_at(GTC_BASE_ADDR) }
    }
}
