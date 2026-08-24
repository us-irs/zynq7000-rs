//! # Global timer counter module.

/// Base address of the global timer counter register block.
pub const GTC_BASE_ADDR: usize = super::mpcore::MPCORE_BASE_ADDR + 0x0000_0200;

pub use types::*;

/// Register helper types.
pub mod types {
    /// Global timer counter control.
    #[bitbybit::bitfield(u32, debug, forbid_overlaps, defmt_bitfields(feature = "defmt"))]
    pub struct Control {
        /// Clock prescaler applied to the counter.
        #[bits(8..=15, rw)]
        prescaler: u8,
        /// Auto-increment the comparator by the auto-increment register value on each match.
        #[bit(3, rw)]
        auto_increment: bool,
        /// Enables the comparator interrupt.
        #[bit(2, rw)]
        irq_enable: bool,
        /// Enables the comparator.
        #[bit(1, rw)]
        comparator_enable: bool,
        /// Enables the counter.
        #[bit(0, rw)]
        enable: bool,
    }

    /// Global timer counter interrupt status.
    #[bitbybit::bitfield(u32, debug, forbid_overlaps, defmt_bitfields(feature = "defmt"))]
    pub struct InterruptStatus {
        /// Comparator match event flag, cleared by writing a one.
        #[bit(0, rw)]
        event_flag: bool,
    }
}

/// Global timer counter register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Count register 0, lower 32 bits
    count_lower: u32,
    /// Count register 1, upper 32 bits
    count_upper: u32,
    /// Control register
    control: Control,
    /// Interrupt status register
    #[mmio(PureRead, Write)]
    interrupt_status: InterruptStatus,
    /// Comparator 0, lower 32 bits
    comparator_lower: u32,
    /// Comparator 1, upper 32 bits
    comparator_upper: u32,
    /// Auto-increment register
    auto_increment: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x1C);

impl Registers {
    /// Create a new GTC MMIO instance at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Registers::new_mmio_at(GTC_BASE_ADDR) }
    }
}
