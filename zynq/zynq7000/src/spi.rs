//! SPI register module.
use arbitrary_int::{prelude::*, u4};

pub use crate::{SpiClockPhase, SpiClockPolarity};

pub const SPI_0_BASE_ADDR: usize = 0xE000_6000;
pub const SPI_1_BASE_ADDR: usize = 0xE000_7000;

/// The SPI reference block will be divided by a divisor value.
#[bitbybit::bitenum(u3)]
#[derive(Debug, PartialEq, Eq)]
pub enum BaudDivSel {
    By4 = 0b001,
    By8 = 0b010,
    By16 = 0b011,
    By32 = 0b100,
    By64 = 0b101,
    By128 = 0b110,
    By256 = 0b111,
}

impl BaudDivSel {
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

// TODO: Use bitbybit debug support as soon as it was added.
#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct Config {
    #[bit(17, rw)]
    modefail_gen_en: bool,
    #[bit(16, w)]
    manual_start: bool,
    #[bit(15, rw)]
    manual_start_enable: bool,
    #[bit(14, rw)]
    manual_cs: bool,
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
    master_ern: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct InterruptStatus {
    #[bit(6, rw)]
    tx_underflow: bool,
    #[bit(5, rw)]
    rx_full: bool,
    #[bit(4, rw)]
    rx_not_empty: bool,
    #[bit(3, rw)]
    tx_full: bool,
    #[bit(2, rw)]
    tx_not_full: bool,
    #[bit(1, rw)]
    mode_fault: bool,
    /// Receiver overflow interrupt.
    #[bit(0, rw)]
    rx_ovr: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct InterruptControl {
    #[bit(6, w)]
    tx_underflow: bool,
    #[bit(5, w)]
    rx_full: bool,
    #[bit(4, w)]
    rx_not_empty: bool,
    #[bit(3, w)]
    tx_full: bool,
    #[bit(2, w)]
    tx_trig: bool,
    #[bit(1, w)]
    mode_fault: bool,
    /// Receiver overflow interrupt.
    #[bit(0, w)]
    rx_ovr: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptMask {
    #[bit(6, r)]
    tx_underflow: bool,
    #[bit(5, r)]
    rx_full: bool,
    #[bit(4, r)]
    rx_not_empty: bool,
    #[bit(3, r)]
    tx_full: bool,
    #[bit(2, r)]
    tx_trig: bool,
    #[bit(1, r)]
    mode_fault: bool,
    /// Receiver overflow interrupt.
    #[bit(0, r)]
    rx_ovr: bool,
}

#[derive(Debug)]
pub struct FifoWrite(arbitrary_int::UInt<u32, 8>);

impl FifoWrite {
    #[inline]
    pub fn new(data: u8) -> Self {
        Self(data.into())
    }

    #[inline]
    pub fn value(&self) -> u8 {
        self.0.as_u8()
    }

    #[inline]
    pub fn write(&mut self, value: u8) {
        self.0 = value.into();
    }
}

#[derive(Debug)]
pub struct FifoRead(arbitrary_int::UInt<u32, 8>);

impl FifoRead {
    #[inline]
    pub fn new(data: u8) -> Self {
        Self(data.into())
    }

    #[inline]
    pub fn value(&self) -> u8 {
        self.0.as_u8()
    }
}

/// The numbers specified in the register fields are always specified in number of
#[bitbybit::bitfield(u32, default = 0x0, debug)]
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

/// SPI register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Spi {
    cr: Config,
    #[mmio(PureRead, Write)]
    isr: InterruptStatus,
    /// Interrupt Enable Register.
    #[mmio(Write)]
    ier: InterruptControl,
    /// Interrupt Disable Register.
    #[mmio(Write)]
    idr: InterruptControl,
    /// Interrupt Mask Register.
    #[mmio(PureRead)]
    imr: InterruptMask,
    enable: u32,
    delay_control: DelayControl,
    #[mmio(Write)]
    txd: FifoWrite,
    #[mmio(Read)]
    rxd: FifoRead,
    sicr: u32,
    tx_trig: u32,
    rx_trig: u32,
    _reserved: [u32; 0x33],
    // Reset value: 0x90106
    #[mmio(PureRead)]
    mod_id: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Spi>(), 0x100);

impl Spi {
    /// Create a new SPI MMIO instance for SPI0 at address [SPI_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioSpi<'static> {
        unsafe { Self::new_mmio_at(SPI_0_BASE_ADDR) }
    }

    /// Create a new SPI MMIO instance for SPI1 at address [SPI_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioSpi<'static> {
        unsafe { Self::new_mmio_at(SPI_1_BASE_ADDR) }
    }
}
