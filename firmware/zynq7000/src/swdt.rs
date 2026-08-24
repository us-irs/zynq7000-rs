//! System watchdog timer (SWDT) register module.
#![deny(missing_docs)]
pub use arbitrary_int::u3;
use arbitrary_int::u12;

/// Base address of the system watchdog timer register block.
pub const SWDT_BASE_ADDR: usize = 0xF800_5000;

/// Key which must be written to [Control]'s key field for any other field write in that
/// register to take effect.
pub const CONTROL_KEY: u12 = u12::new(0xABC);
/// Key which must be written to [CounterConfig]'s key field for any other field write in
/// that register to take effect.
pub const COUNTER_CONFIG_KEY: u12 = u12::new(0x248);
/// Key which must be written to [Restart] to restart the watchdog.
pub const RESTART_KEY: u16 = 0x1999;

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u3, u12};

    /// Length of the pulse generated on the IRQ output, in PCLK cycles.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum IrqLengthCycles {
        /// 4 PCLK cycles.
        Four = 0b00,
        /// 8 PCLK cycles.
        Eight = 0b01,
        /// 16 PCLK cycles.
        Sixteen = 0b10,
        /// 32 PCLK cycles.
        ThirtyTwo = 0b11,
    }

    /// Mode register controlling how the watchdog behaves once it expires.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Control {
        /// Must be written to [super::CONTROL_KEY].
        #[bits(12..=23, rw)]
        key: u12,
        /// Length of the pulse generated on the IRQ output.
        #[bits(7..=8, rw)]
        irq_len: IrqLengthCycles,
        /// Reserved field which must always be written as 4.
        #[bits(4..=6, rw)]
        should_be_four: u3,
        /// Enables the IRQ output on watchdog expiry.
        #[bit(2, rw)]
        irq_enable: bool,
        /// Enables the reset output on watchdog expiry.
        #[bit(1, rw)]
        reset_enable: bool,
        /// Enables the watchdog counter.
        #[bit(0, rw)]
        enable: bool,
    }

    /// Prescaler applied to PCLK to derive the watchdog counter clock.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum PclkPrescaler {
        /// PCLK divided by 8.
        Div8 = 0b00,
        /// PCLK divided by 64.
        Div64 = 0b01,
        /// PCLK divided by 512.
        Div512 = 0b10,
        /// PCLK divided by 4096.
        Div4096 = 0b11,
    }

    impl PclkPrescaler {
        /// All four variants, smallest divisor first.
        pub const ALL: [Self; 4] = [Self::Div8, Self::Div64, Self::Div512, Self::Div4096];

        /// The divisor this prescaler applies to PCLK.
        pub const fn divisor(&self) -> u32 {
            match self {
                Self::Div8 => 8,
                Self::Div64 => 64,
                Self::Div512 => 512,
                Self::Div4096 => 4096,
            }
        }
    }

    /// Counter control register configuring the watchdog counter restart value and clock.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CounterConfig {
        /// Must be written to [super::COUNTER_CONFIG_KEY].
        #[bits(14..=25, rw)]
        key: u12,
        /// Counter restart value upper bits. The watchdog will be restarted with the value
        /// 0xNNNFFF
        #[bits(2..=13, rw)]
        restart_upper_bits: u12,
        /// Prescaler applied to PCLK to derive the watchdog counter clock.
        #[bits(0..=1, rw)]
        clock_prescaler: PclkPrescaler,
    }

    /// Restart register. Writing [super::RESTART_KEY] to it restarts (kicks) the watchdog.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Restart {
        /// Must be written to [super::RESTART_KEY] to restart the watchdog.
        #[bits(0..=15, w)]
        key: u16,
    }

    impl Restart {
        /// Pre-built value which restarts (kicks) the watchdog when written.
        pub const RESTART: Self = Self::builder().with_key(super::RESTART_KEY).build();
    }

    /// Status register.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct Status {
        /// Set once the watchdog counter has reached zero.
        #[bit(0, r)]
        watchdog_zero: bool,
    }
}

/// SWDT register block.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Control register.
    control: Control,
    /// Counter config register.
    counter_config: CounterConfig,
    /// Restart register.
    #[mmio(Write)]
    restart: Restart,
    /// Status register.
    #[mmio(PureRead)]
    status: Status,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x10);

impl Registers {
    /// Create a new SWDT MMIO instance at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Registers::new_mmio_at(SWDT_BASE_ADDR) }
    }
}
