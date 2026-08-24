//! SPI register module.

/// Base address of the SPI0 register block.
pub const SPI_0_BASE_ADDR: usize = 0xE000_6000;
/// Base address of the SPI1 register block.
pub const SPI_1_BASE_ADDR: usize = 0xE000_7000;

pub use types::*;

/// Register helper types.
pub mod types {
    pub use crate::{SpiClockPhase, SpiClockPolarity};
    use arbitrary_int::{prelude::*, u4};

    /// The SPI reference clock will be divided by a divisor value.
    #[bitbybit::bitenum(u3)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum BaudDivSel {
        /// Divide reference clock by 4.
        By4 = 0b001,
        /// Divide reference clock by 8.
        By8 = 0b010,
        /// Divide reference clock by 16.
        By16 = 0b011,
        /// Divide reference clock by 32.
        By32 = 0b100,
        /// Divide reference clock by 64.
        By64 = 0b101,
        /// Divide reference clock by 128.
        By128 = 0b110,
        /// Divide reference clock by 256.
        By256 = 0b111,
    }

    impl BaudDivSel {
        /// Numerical divisor value for this selection.
        pub const fn div_value(&self) -> usize {
            match self {
                BaudDivSel::By4 => 4,
                BaudDivSel::By8 => 8,
                BaudDivSel::By16 => 16,
                BaudDivSel::By32 => 32,
                BaudDivSel::By64 => 64,
                BaudDivSel::By128 => 128,
                BaudDivSel::By256 => 256,
            }
        }
    }

    /// SPI controller mode.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Mode {
        /// Slave mode.
        Slave = 0,
        /// Master mode.
        Master = 1,
    }

    /// SPI configuration register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Config {
        /// Enables generation of the mode fault error.
        #[bit(17, rw)]
        modefail_gen_en: bool,
        /// Starts a transfer manually when manual start mode is enabled.
        #[bit(16, w)]
        manual_start: bool,
        /// Enables manual start mode.
        #[bit(15, rw)]
        manual_start_enable: bool,
        /// Enables manual chip select mode.
        #[bit(14, rw)]
        manual_cs: bool,
        /// Raw chip select value, active low, one bit per slave select line.
        #[bits(10..=13, rw)]
        cs_raw: u4,
        /// Peripheral select decode, 1: Allow external 3-to-8 decode.
        /// I am not sure how exactly this work, but I suspect the last three bits of the chip
        /// select bits will be output directly to the 3 chip select output lines.
        #[bit(9, rw)]
        peri_sel: bool,
        /// Uses SPI reference clock, value 1 is not supported.
        #[bit(8, r)]
        ref_clk: bool,
        /// Baud rate divisor selection.
        #[bits(3..=5, rw)]
        baud_rate_div: Option<BaudDivSel>,
        /// Clock phase. 1: The SPI clock is inactive outside the word.
        #[bit(2, rw)]
        cpha: SpiClockPhase,
        /// Clock phase. 1: The SPI clock is quiescent high.
        #[bit(1, rw)]
        cpol: SpiClockPolarity,
        /// Master mode enable. 1 is master mode.
        #[bit(0, rw)]
        mode: Mode,
    }

    /// Interrupt status register, cleared by writing 1 to a set bit.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptStatus {
        /// TX FIFO underflow interrupt.
        #[bit(6, rw)]
        tx_underflow: bool,
        /// RX FIFO full interrupt.
        #[bit(5, rw)]
        rx_full: bool,
        /// RX FIFO not empty interrupt.
        #[bit(4, rw)]
        rx_not_empty: bool,
        /// Switches to 1 when the FIFO becomes full and then remains asserted until the FIFO falls
        /// below the configured threshold level.
        #[bit(3, rw)]
        tx_full: bool,
        /// TX FIFO level below configured threshold.
        #[bit(2, rw)]
        tx_below_threshold: bool,
        /// Mode fault interrupt.
        #[bit(1, rw)]
        mode_fault: bool,
        /// Receiver overflow interrupt.
        #[bit(0, rw)]
        rx_overrun: bool,
    }

    /// Write-to-clear interrupt control, used to clear interrupt status bits.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    #[derive(Debug)]
    pub struct InterruptControl {
        /// TX FIFO underflow interrupt.
        #[bit(6, w)]
        tx_underflow: bool,
        /// RX FIFO full interrupt.
        #[bit(5, w)]
        rx_full: bool,
        /// RX FIFO not empty interrupt.
        #[bit(4, w)]
        rx_not_empty: bool,
        /// Switches to 1 when the FIFO becomes full and then remains asserted until the FIFO falls
        /// below the configured threshold level.
        #[bit(3, w)]
        tx_full: bool,
        /// Interrupt when TX FIFO level below configured threshold.
        #[bit(2, w)]
        tx_below_threshold: bool,
        /// Mode fault interrupt.
        #[bit(1, w)]
        mode_fault: bool,
        /// Receiver overflow interrupt.
        #[bit(0, w)]
        rx_ovr: bool,
    }

    impl InterruptControl {
        /// All interrupt bits set.
        pub const ALL: Self = Self::builder()
            .with_tx_underflow(true)
            .with_rx_full(true)
            .with_rx_not_empty(true)
            .with_tx_full(true)
            .with_tx_below_threshold(true)
            .with_mode_fault(true)
            .with_rx_ovr(true)
            .build();
    }

    /// Interrupt mask register, reads back which interrupts are currently enabled.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct InterruptEnabled {
        /// TX FIFO underflow interrupt.
        #[bit(6, r)]
        tx_underflow: bool,
        /// RX FIFO full interrupt.
        #[bit(5, r)]
        rx_full: bool,
        /// RX FIFO not empty interrupt.
        #[bit(4, r)]
        rx_not_empty: bool,
        /// Switches to 1 when the FIFO becomes full and then remains asserted until the FIFO falls
        /// below the configured threshold level.
        #[bit(3, r)]
        tx_full: bool,
        /// Interrupt when TX FIFO level below configured threshold.
        #[bit(2, r)]
        tx_below_threshold: bool,
        /// Mode fault interrupt.
        #[bit(1, r)]
        mode_fault: bool,
        /// Receiver overflow interrupt.
        #[bit(0, r)]
        rx_ovr: bool,
    }

    /// Single byte written into the TX FIFO.
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct FifoWrite(arbitrary_int::UInt<u32, 8>);

    impl FifoWrite {
        /// Creates a new FIFO write value.
        #[inline]
        pub fn new(data: u8) -> Self {
            Self(data.into())
        }

        /// Returns the byte value.
        #[inline]
        pub fn value(&self) -> u8 {
            self.0.as_u8()
        }

        /// Sets the byte value.
        #[inline]
        pub fn write(&mut self, value: u8) {
            self.0 = value.into();
        }
    }

    /// Single byte read from the RX FIFO.
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct FifoRead(arbitrary_int::UInt<u32, 8>);

    impl FifoRead {
        /// Creates a new FIFO read value.
        #[inline]
        pub fn new(data: u8) -> Self {
            Self(data.into())
        }

        /// Returns the byte value.
        #[inline]
        pub fn value(&self) -> u8 {
            self.0.as_u8()
        }
    }

    /// The numbers specified in the register fields are always specified in number of
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        forbid_overlaps,
        defmt_bitfields(feature = "defmt")
    )]
    pub struct DelayControl {
        /// Number of cycles the chip select is de-asserted between words when CPHA = 0
        #[bits(24..=31, rw)]
        inter_word_cs_deassert: u8,
        /// Delay between one chip select being de-activated, and activation of another.
        #[bits(16..=23, rw)]
        between_cs_assertion: u8,
        /// Delay between words.
        #[bits(8..=15, rw)]
        inter_word: u8,
        /// Added delay between assertion of slave select and first bit transfer.
        #[bits(0..=7, rw)]
        cs_to_first_bit: u8,
    }
}

/// SPI register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Configuration register.
    config: Config,
    /// Interrupt status register.
    #[mmio(PureRead, Write)]
    interrupt_status: InterruptStatus,
    /// Interrupt Enable Register.
    #[mmio(Write)]
    interrupt_enable: InterruptControl,
    /// Interrupt Disable Register.
    #[mmio(Write)]
    interupt_disable: InterruptControl,
    /// Interrupt Mask Register.
    #[mmio(PureRead)]
    enabled_interrupts: InterruptEnabled,
    /// Enable register.
    enable: u32,
    /// Delay register.
    delay_control: DelayControl,
    /// TX data register.
    #[mmio(Write)]
    tx_data: FifoWrite,
    /// RX data register.
    #[mmio(Read)]
    rx_data: FifoRead,
    /// Slave idle count register.
    sicr: u32,
    /// TX FIFO threshold register.
    tx_trig: u32,
    /// RX FIFO threshold register.
    rx_trig: u32,
    _reserved: [u32; 0x33],
    // Reset value: 0x90106
    /// Module ID register.
    #[mmio(PureRead)]
    mod_id: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x100);

impl Registers {
    /// Create a new SPI MMIO instance for SPI0 at address [SPI_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(SPI_0_BASE_ADDR) }
    }

    /// Create a new SPI MMIO instance for SPI1 at address [SPI_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(SPI_1_BASE_ADDR) }
    }
}
