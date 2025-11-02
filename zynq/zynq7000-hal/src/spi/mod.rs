//! SPI module
//!
//! ## Examples
//!
//! - [L3GD20H SPI sensor](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq/examples/zedboard/src/bin/l3gd20h-spi-mio.rs)
use core::convert::Infallible;

use crate::clocks::Clocks;
use crate::gpio::IoPeriphPin;
use crate::gpio::mio::{
    Mio10, Mio11, Mio12, Mio13, Mio14, Mio15, Mio28, Mio29, Mio30, Mio31, Mio32, Mio33, Mio34,
    Mio35, Mio36, Mio37, Mio38, Mio39, MioPin, MuxConfig, Pin,
};
#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27, Mio40,
    Mio41, Mio42, Mio43, Mio44, Mio45, Mio46, Mio47, Mio48, Mio49, Mio50, Mio51,
};
use crate::{enable_amba_peripheral_clock, spi_mode_const_to_cpol_cpha};

use crate::{clocks::IoClocks, slcr::Slcr, time::Hertz};
use arbitrary_int::{prelude::*, u3, u4, u6};
use embedded_hal::delay::DelayNs;
pub use embedded_hal::spi::Mode;
use embedded_hal::spi::SpiBus as _;
use zynq7000::slcr::reset::DualRefAndClockReset;
use zynq7000::spi::{
    BaudDivSel, DelayControl, FifoWrite, InterruptControl, InterruptMask, InterruptStatus,
    MmioRegisters, SPI_0_BASE_ADDR, SPI_1_BASE_ADDR,
};

pub const FIFO_DEPTH: usize = 128;
pub const MODULE_ID: u32 = 0x90106;

pub mod asynch;
pub use asynch::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiId {
    Spi0 = 0,
    Spi1 = 1,
}

pub trait PsSpi {
    fn reg_block(&self) -> MmioRegisters<'static>;
    fn id(&self) -> Option<SpiId>;
}

impl PsSpi for MmioRegisters<'static> {
    #[inline]
    fn reg_block(&self) -> MmioRegisters<'static> {
        unsafe { self.clone() }
    }

    #[inline]
    fn id(&self) -> Option<SpiId> {
        let base_addr = unsafe { self.ptr() } as usize;
        if base_addr == SPI_0_BASE_ADDR {
            return Some(SpiId::Spi0);
        } else if base_addr == SPI_1_BASE_ADDR {
            return Some(SpiId::Spi1);
        }
        None
    }
}

pub trait SckPin: MioPin {
    const SPI: SpiId;
    const GROUP: usize;
}

pub trait MosiPin: MioPin {
    const SPI: SpiId;
    const GROUP: usize;
}

pub trait MisoPin: MioPin {
    const SPI: SpiId;
    const GROUP: usize;
}

pub trait SsPin: MioPin {
    const IDX: usize;
    const SPI: SpiId;
    const GROUP: usize;
}

pub const SPI_MUX_CONF: MuxConfig = MuxConfig::new_with_l3(u3::new(0b101));

// SPI0, choice 1
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SckPin for Pin<Mio16> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MosiPin for Pin<Mio21> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MisoPin for Pin<Mio17> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio18> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 0;
    const IDX: usize = 0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio19> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 0;
    const IDX: usize = 1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio20> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 0;
    const IDX: usize = 2;
}

// SPI0, choice 2
impl SckPin for Pin<Mio28> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 1;
}
impl MosiPin for Pin<Mio33> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 1;
}
impl MisoPin for Pin<Mio29> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 1;
}
impl SsPin for Pin<Mio30> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 1;
    const IDX: usize = 0;
}
impl SsPin for Pin<Mio31> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 1;
    const IDX: usize = 1;
}
impl SsPin for Pin<Mio32> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 1;
    const IDX: usize = 2;
}

// SPI0, choice 3
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SckPin for Pin<Mio40> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 2;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MosiPin for Pin<Mio45> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 2;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MisoPin for Pin<Mio41> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 2;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio42> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 2;
    const IDX: usize = 0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio43> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 2;
    const IDX: usize = 1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio44> {
    const SPI: SpiId = SpiId::Spi0;
    const GROUP: usize = 2;
    const IDX: usize = 2;
}

// SPI1, choice 1
impl SckPin for Pin<Mio12> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 0;
}
impl MosiPin for Pin<Mio10> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 0;
}
impl MisoPin for Pin<Mio11> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 0;
}
impl SsPin for Pin<Mio13> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 0;
    const IDX: usize = 0;
}
impl SsPin for Pin<Mio14> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 0;
    const IDX: usize = 1;
}
impl SsPin for Pin<Mio15> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 0;
    const IDX: usize = 2;
}

// SPI1, choice 2
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SckPin for Pin<Mio24> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MosiPin for Pin<Mio22> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MisoPin for Pin<Mio23> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio25> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 1;
    const IDX: usize = 0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio26> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 1;
    const IDX: usize = 1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio27> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 1;
    const IDX: usize = 2;
}

// SPI1, choice 2
impl SckPin for Pin<Mio36> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 2;
}
impl MosiPin for Pin<Mio34> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 2;
}
impl MisoPin for Pin<Mio35> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 2;
}
impl SsPin for Pin<Mio37> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 2;
    const IDX: usize = 0;
}
impl SsPin for Pin<Mio38> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 2;
    const IDX: usize = 1;
}
impl SsPin for Pin<Mio39> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 2;
    const IDX: usize = 2;
}

// SPI1, choice 3
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SckPin for Pin<Mio48> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 3;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MosiPin for Pin<Mio46> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 3;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl MisoPin for Pin<Mio47> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 3;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio49> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 3;
    const IDX: usize = 0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio50> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 3;
    const IDX: usize = 1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SsPin for Pin<Mio51> {
    const SPI: SpiId = SpiId::Spi1;
    const GROUP: usize = 3;
    const IDX: usize = 2;
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum ChipSelect {
    Slave0,
    Slave1,
    Slave2,
    Decoded { cs0: bool, cs1: bool, cs2: bool },
    None,
}

impl ChipSelect {
    pub const fn raw_reg(&self) -> u4 {
        u4::new(match self {
            ChipSelect::Slave0 => 0b0000,
            ChipSelect::Slave1 => 0b0001,
            ChipSelect::Slave2 => 0b0010,
            ChipSelect::None => 0b1111,
            ChipSelect::Decoded { cs0, cs1, cs2 } => {
                (1 << 3) | (*cs0 as u8) | ((*cs1 as u8) << 1) | ((*cs2 as u8) << 2)
            }
        })
    }

    #[inline]
    pub const fn cs_no_ext_decoding(&self, raw: u4) -> Option<ChipSelect> {
        let val = raw.value();
        if val == 0b1111 {
            return Some(ChipSelect::None);
        }
        if val & 0b1 == 0 {
            return Some(ChipSelect::Slave0);
        } else if val & 0b11 == 0b01 {
            return Some(ChipSelect::Slave1);
        } else if val & 0b111 == 0b010 {
            return Some(ChipSelect::Slave2);
        }
        None
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
/// Slave select configuration.
pub enum SlaveSelectConfig {
    /// User must take care of controlling slave select lines as well as issuing a start command.
    ManualWithManualStart = 0b11,
    ManualAutoStart = 0b10,
    /// Hardware slave select, but start needs to be issued manually.
    AutoWithManualStart = 0b01,
    /// Hardware slave select, auto serialiation if there is data in the TX FIFO.
    #[default]
    AutoWithAutoStart = 0b00,
}

#[derive(Debug, Copy, Clone)]
pub struct Config {
    baud_div: BaudDivSel,
    init_mode: Mode,
    ss_config: SlaveSelectConfig,
    with_ext_decoding: bool,
}

impl Config {
    pub fn new(baud_div: BaudDivSel, init_mode: Mode, ss_config: SlaveSelectConfig) -> Self {
        Self {
            baud_div,
            init_mode,
            ss_config,
            with_ext_decoding: false,
        }
    }

    pub fn enable_external_decoding(&mut self) {
        self.with_ext_decoding = true;
    }
}

/// Thin re-usable low-level helper to interface with the SPI.
pub struct SpiLowLevel {
    id: SpiId,
    regs: zynq7000::spi::MmioRegisters<'static>,
}

impl SpiLowLevel {
    /// Steal the SPI low level helper.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub unsafe fn steal(id: SpiId) -> Self {
        let regs = unsafe {
            match id {
                SpiId::Spi0 => zynq7000::spi::Registers::new_mmio_fixed_0(),
                SpiId::Spi1 => zynq7000::spi::Registers::new_mmio_fixed_1(),
            }
        };
        Self { id, regs }
    }

    /// Clone the SPI low level helper.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub unsafe fn clone(&self) -> Self {
        Self {
            id: self.id,
            regs: unsafe { self.regs.clone() },
        }
    }

    pub fn id(&self) -> SpiId {
        self.id
    }

    #[inline]
    pub fn disable(&mut self) {
        self.regs.write_enable(0);
    }

    #[inline]
    pub fn enable(&mut self) {
        self.regs.write_enable(1);
    }

    /// Select the peripheral chip select line.
    ///
    /// This needs to be done before starting a transfer to select the correct peripheral chip
    /// select line.
    ///
    /// The decoder bits logic is is based
    /// [on online documentation](https://www.realdigital.org/doc/3eb4f7a05e5003f2e0e6858a27a679bb?utm_source=chatgpt.com)
    /// because the TRM does not specify how decoding really works. This also only works if
    /// the external decoding was enabled via the [Config::enable_external_decoding] option.
    #[inline]
    pub fn select_hw_cs(&mut self, chip_select: ChipSelect) {
        self.regs.modify_cr(|mut val| {
            val.set_cs_raw(chip_select.raw_reg());
            val
        });
    }

    /// Re-configures the mode register.
    #[inline]
    pub fn configure_mode(&mut self, mode: Mode) {
        let (cpol, cpha) = spi_mode_const_to_cpol_cpha(mode);
        self.regs.modify_cr(|mut val| {
            val.set_cpha(cpha);
            val.set_cpol(cpol);
            val
        });
    }

    /// Re-configures the delay control register.
    #[inline]
    pub fn configure_delays(&mut self, config: DelayControl) {
        self.regs.write_delay_control(config)
    }

    /// Re-configures the SPI peripheral with the initial configuration.
    pub fn reconfigure(&mut self, config: Config) {
        self.regs.write_enable(0);
        let (man_ss, man_start) = match config.ss_config {
            SlaveSelectConfig::ManualWithManualStart => (true, true),
            SlaveSelectConfig::ManualAutoStart => (true, false),
            SlaveSelectConfig::AutoWithManualStart => (false, true),
            SlaveSelectConfig::AutoWithAutoStart => (false, false),
        };
        let (cpol, cpha) = spi_mode_const_to_cpol_cpha(config.init_mode);

        self.regs.write_cr(
            zynq7000::spi::Config::builder()
                .with_modefail_gen_en(false)
                .with_manual_start(false)
                .with_manual_start_enable(man_start)
                .with_manual_cs(man_ss)
                .with_cs_raw(ChipSelect::None.raw_reg())
                .with_peri_sel(config.with_ext_decoding)
                .with_baud_rate_div(config.baud_div)
                .with_cpha(cpha)
                .with_cpol(cpol)
                .with_master_ern(true)
                .build(),
        );
        // Configures for polling mode by default: TX trigger by one will lead to the
        // TX FIFO not full signal being set when the TX FIFO is emtpy.
        self.regs.write_tx_trig(1);
        // Same as TX.
        self.regs.write_rx_trig(1);

        self.regs.write_enable(1);
    }

    /// [Resets][reset] and [re-configures][Self::reconfigure] the SPI peripheral.
    ///
    /// This can also be used to reset the FIFOs and the state machine of the SPI.
    pub fn reset_and_reconfigure(&mut self, config: Config) {
        reset(self.id());
        self.reconfigure(config);
    }

    /// No peripheral slave selection.
    #[inline]
    pub fn no_hw_cs(&mut self) {
        self.select_hw_cs(ChipSelect::None);
    }

    #[inline(always)]
    pub fn write_fifo_unchecked(&mut self, data: u8) {
        self.regs.write_txd(FifoWrite::new(data));
    }

    #[inline(always)]
    pub fn read_fifo_unchecked(&mut self) -> u8 {
        self.regs.read_rxd().value()
    }

    #[inline]
    pub fn issue_manual_start(&mut self) {
        self.regs.modify_cr(|mut val| {
            val.set_manual_start(true);
            val
        });
    }

    #[inline]
    pub fn read_isr(&self) -> InterruptStatus {
        self.regs.read_isr()
    }

    #[inline]
    pub fn read_imr(&self) -> InterruptMask {
        self.regs.read_imr()
    }

    #[inline]
    pub fn read_rx_not_empty_threshold(&self) -> u32 {
        self.regs.read_rx_trig()
    }

    #[inline]
    pub fn set_rx_fifo_trigger(&mut self, trigger: u32) -> Result<(), InvalidTriggerError> {
        if trigger > FIFO_DEPTH as u32 {
            return Err(InvalidTriggerError(trigger as usize));
        }
        self.regs.write_rx_trig(trigger.value());
        Ok(())
    }

    #[inline]
    pub fn set_tx_fifo_trigger(&mut self, trigger: u32) -> Result<(), InvalidTriggerError> {
        if trigger > FIFO_DEPTH as u32 {
            return Err(InvalidTriggerError(trigger as usize));
        }
        self.regs.write_tx_trig(trigger.value());
        Ok(())
    }

    /// This disables all interrupts relevant for non-blocking interrupt driven SPI operation
    /// in SPI master mode.
    #[inline]
    pub fn disable_interrupts(&mut self) {
        self.regs.write_idr(
            InterruptControl::builder()
                .with_tx_underflow(true)
                .with_rx_full(true)
                .with_rx_not_empty(true)
                .with_tx_full(false)
                .with_tx_trig(true)
                .with_mode_fault(false)
                .with_rx_ovr(true)
                .build(),
        );
    }

    /// This enables all interrupts relevant for non-blocking interrupt driven SPI operation
    /// in SPI master mode.
    #[inline]
    pub fn enable_interrupts(&mut self) {
        self.regs.write_ier(
            InterruptControl::builder()
                .with_tx_underflow(true)
                .with_rx_full(true)
                .with_rx_not_empty(true)
                .with_tx_full(false)
                .with_tx_trig(true)
                .with_mode_fault(false)
                .with_rx_ovr(true)
                .build(),
        );
    }

    /// This clears all interrupts relevant for non-blocking interrupt driven SPI operation
    /// in SPI master mode.
    #[inline]
    pub fn clear_interrupts(&mut self) {
        self.regs.write_isr(
            InterruptStatus::builder()
                .with_tx_underflow(true)
                .with_rx_full(true)
                .with_rx_not_empty(true)
                .with_tx_full(false)
                .with_tx_not_full(true)
                .with_mode_fault(false)
                .with_rx_ovr(true)
                .build(),
        );
    }
}

/// Blocking Driver for the PS SPI peripheral in master mode.
pub struct Spi {
    inner: SpiLowLevel,
    sclk: Hertz,
    config: Config,
    outstanding_rx: bool,
}

#[derive(Debug, thiserror::Error)]
#[error("invalid SPI ID")]
pub struct InvalidPsSpiError;

#[derive(Debug, thiserror::Error)]
#[error("invalid trigger value {0}")]
pub struct InvalidTriggerError(pub usize);

// TODO: Add and handle MUX config check.
#[derive(Debug, thiserror::Error)]
pub enum SpiConstructionError {
    #[error("invalid SPI ID {0}")]
    InvalidPsSpi(#[from] InvalidPsSpiError),
    /// The specified pins are not compatible to the specified SPI peripheral.
    #[error("pin invalid for SPI ID")]
    PinInvalidForSpiId,
    /// The specified pins are not from the same pin group.
    #[error("pin group missmatch")]
    GroupMissmatch,
    #[error("invalid pin configuration for SPI")]
    InvalidPinConf,
}

impl Spi {
    pub fn new_no_hw_ss<Sck: SckPin, Mosi: MosiPin, Miso: MisoPin>(
        spi: impl PsSpi,
        clocks: &IoClocks,
        config: Config,
        spi_pins: (Sck, Mosi, Miso),
    ) -> Result<Self, SpiConstructionError> {
        let spi_id = spi.id();
        if spi_id.is_none() {
            return Err(InvalidPsSpiError.into());
        }
        let spi_id = spi_id.unwrap();
        if Sck::GROUP != Mosi::GROUP || Sck::GROUP != Miso::GROUP {
            return Err(SpiConstructionError::GroupMissmatch);
        }
        if Sck::SPI != spi_id || Mosi::SPI != spi_id || Miso::SPI != spi_id {
            return Err(SpiConstructionError::PinInvalidForSpiId);
        }
        IoPeriphPin::new(spi_pins.0, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.1, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.2, SPI_MUX_CONF, Some(false));
        Ok(Self::new_generic_unchecked(
            spi_id,
            spi.reg_block(),
            clocks,
            config,
        ))
    }

    pub fn new_one_hw_cs<Sck: SckPin, Mosi: MosiPin, Miso: MisoPin, Ss: SsPin>(
        spi: impl PsSpi,
        clocks: &IoClocks,
        config: Config,
        spi_pins: (Sck, Mosi, Miso),
        ss_pin: Ss,
    ) -> Result<Self, SpiConstructionError> {
        let spi_id = spi.id();
        if spi_id.is_none() {
            return Err(InvalidPsSpiError.into());
        }
        let spi_id = spi_id.unwrap();
        if Sck::GROUP != Mosi::GROUP || Sck::GROUP != Miso::GROUP || Sck::GROUP != Ss::GROUP {
            return Err(SpiConstructionError::GroupMissmatch);
        }
        if Sck::SPI != spi_id || Mosi::SPI != spi_id || Miso::SPI != spi_id || Ss::SPI != spi_id {
            return Err(SpiConstructionError::PinInvalidForSpiId);
        }
        IoPeriphPin::new(spi_pins.0, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.1, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.2, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(ss_pin, SPI_MUX_CONF, Some(false));
        Ok(Self::new_generic_unchecked(
            spi_id,
            spi.reg_block(),
            clocks,
            config,
        ))
    }

    pub fn new_with_two_hw_cs<Sck: SckPin, Mosi: MosiPin, Miso: MisoPin, Ss0: SsPin, Ss1: SsPin>(
        spi: impl PsSpi,
        clocks: &IoClocks,
        config: Config,
        spi_pins: (Sck, Mosi, Miso),
        ss_pins: (Ss0, Ss1),
    ) -> Result<Self, SpiConstructionError> {
        let spi_id = spi.id();
        if spi_id.is_none() {
            return Err(InvalidPsSpiError.into());
        }
        let spi_id = spi_id.unwrap();
        if Sck::GROUP != Mosi::GROUP
            || Sck::GROUP != Miso::GROUP
            || Sck::GROUP != Ss0::GROUP
            || Sck::GROUP != Ss1::GROUP
        {
            return Err(SpiConstructionError::GroupMissmatch);
        }
        if Sck::SPI != spi_id
            || Mosi::SPI != spi_id
            || Miso::SPI != spi_id
            || Ss0::SPI != spi_id
            || Ss1::SPI != spi_id
        {
            return Err(SpiConstructionError::PinInvalidForSpiId);
        }
        IoPeriphPin::new(spi_pins.0, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.1, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.2, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(ss_pins.0, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(ss_pins.1, SPI_MUX_CONF, Some(false));
        Ok(Self::new_generic_unchecked(
            spi_id,
            spi.reg_block(),
            clocks,
            config,
        ))
    }

    pub fn new_with_three_hw_cs<
        Sck: SckPin,
        Mosi: MosiPin,
        Miso: MisoPin,
        Ss0: SsPin,
        Ss1: SsPin,
        Ss2: SsPin,
    >(
        spi: impl PsSpi,
        clocks: &IoClocks,
        config: Config,
        spi_pins: (Sck, Mosi, Miso),
        ss_pins: (Ss0, Ss1, Ss2),
    ) -> Result<Self, SpiConstructionError> {
        let spi_id = spi.id();
        if spi_id.is_none() {
            return Err(InvalidPsSpiError.into());
        }
        let spi_id = spi_id.unwrap();
        if Sck::GROUP != Mosi::GROUP
            || Sck::GROUP != Miso::GROUP
            || Sck::GROUP != Ss0::GROUP
            || Sck::GROUP != Ss1::GROUP
            || Sck::GROUP != Ss2::GROUP
        {
            return Err(SpiConstructionError::GroupMissmatch);
        }
        if Sck::SPI != spi_id
            || Mosi::SPI != spi_id
            || Miso::SPI != spi_id
            || Ss0::SPI != spi_id
            || Ss1::SPI != spi_id
            || Ss2::SPI != spi_id
        {
            return Err(SpiConstructionError::PinInvalidForSpiId);
        }
        IoPeriphPin::new(spi_pins.0, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.1, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(spi_pins.2, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(ss_pins.0, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(ss_pins.1, SPI_MUX_CONF, Some(false));
        IoPeriphPin::new(ss_pins.2, SPI_MUX_CONF, Some(false));
        Ok(Self::new_generic_unchecked(
            spi_id,
            spi.reg_block(),
            clocks,
            config,
        ))
    }

    pub fn new_generic_unchecked(
        id: SpiId,
        regs: MmioRegisters<'static>,
        clocks: &IoClocks,
        config: Config,
    ) -> Self {
        let periph_sel = match id {
            SpiId::Spi0 => crate::PeriphSelect::Spi0,
            SpiId::Spi1 => crate::PeriphSelect::Spi1,
        };
        enable_amba_peripheral_clock(periph_sel);
        let sclk = clocks.spi_clk() / config.baud_div.div_value() as u32;
        let mut spi = Self {
            inner: SpiLowLevel { regs, id },
            sclk,
            config,
            outstanding_rx: false,
        };
        spi.reset_and_reconfigure();
        spi
    }

    /// Re-configures the SPI peripheral with the initial configuration.
    pub fn reconfigure(&mut self) {
        self.inner.reconfigure(self.config);
    }

    /// [Resets][reset] and [re-configures][Self::reconfigure] the SPI peripheral.
    ///
    /// This can also be used to reset the FIFOs and the state machine of the SPI.
    pub fn reset_and_reconfigure(&mut self) {
        reset(self.inner.id());
        self.reconfigure();
    }

    #[inline]
    pub fn issue_manual_start_for_manual_cfg(&mut self) {
        if self.config.ss_config == SlaveSelectConfig::AutoWithManualStart
            || self.config.ss_config == SlaveSelectConfig::ManualWithManualStart
        {
            self.inner.issue_manual_start();
        }
    }

    /// Retrieve SCLK clock frequency currently configured for this SPI.
    #[inline]
    pub const fn sclk(&self) -> Hertz {
        self.sclk
    }

    /// Retrieve inner low-level helper.
    #[inline]
    pub const fn inner(&mut self) -> &mut SpiLowLevel {
        &mut self.inner
    }

    #[inline]
    pub fn regs(&mut self) -> &mut MmioRegisters<'static> {
        &mut self.inner.regs
    }

    fn initial_fifo_fill(&mut self, words: &[u8]) -> usize {
        let write_len = core::cmp::min(FIFO_DEPTH, words.len());
        (0..write_len).for_each(|idx| {
            self.inner.write_fifo_unchecked(words[idx]);
        });
        write_len
    }

    fn prepare_generic_blocking_transfer(&mut self, words: &[u8]) -> usize {
        // We want to ensure the FIFO is empty for a new transfer. This is the simpler
        // implementation for now.
        self.flush().unwrap();
        // Write this to 1 in any case to allow polling, defensive programming.
        self.inner.regs.write_rx_trig(1);

        // Fill the FIFO with initial data.
        let written = self.initial_fifo_fill(words);

        // We assume that the slave select configuration was already performed, but we take
        // care of issuing a start if necessary.
        self.issue_manual_start_for_manual_cfg();
        written
    }
}

impl embedded_hal::spi::ErrorType for Spi {
    type Error = Infallible;
}

impl embedded_hal::spi::SpiBus for Spi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        // We want to ensure the FIFO is empty for a new transfer. This is the simpler
        // implementation for now.
        self.flush()?;
        // Write this to 1 in any case to allow polling, defensive programming.
        self.regs().write_rx_trig(1);

        let mut write_idx = core::cmp::min(FIFO_DEPTH, words.len());
        // Send dummy bytes.
        (0..write_idx).for_each(|_| {
            self.inner.write_fifo_unchecked(0);
        });

        // We assume that the slave select configuration was already performed, but we take
        // care of issuing a start if necessary.
        self.issue_manual_start_for_manual_cfg();

        let mut read_idx = 0;
        while read_idx < words.len() {
            let status = self.regs().read_isr();
            if status.rx_not_empty() {
                words[read_idx] = self.inner.read_fifo_unchecked();
                read_idx += 1;
            }
            // Continue pumping the FIFO if necesary and possible.
            if write_idx < words.len() && !status.tx_full() {
                self.inner.write_fifo_unchecked(0);
                write_idx += 1;
            }
        }

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        let mut written = self.prepare_generic_blocking_transfer(words);
        let mut read_idx = 0;

        while written < words.len() {
            let status = self.regs().read_isr();
            // We empty the FIFO to prevent it filling up completely, as long as we have to write
            // bytes
            if status.rx_not_empty() {
                self.inner.read_fifo_unchecked();
                read_idx += 1;
            }
            if !status.tx_full() {
                self.inner.write_fifo_unchecked(words[written]);
                written += 1;
            }
        }
        // We exit once all bytes have been written, so some bytes to read might be outstanding.
        // We use the FIFO trigger mechanism to determine when we can read all the remaining bytes.
        self.regs().write_rx_trig((words.len() - read_idx) as u32);
        self.outstanding_rx = true;
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        if read.is_empty() {
            return Ok(());
        }
        let mut write_idx = self.prepare_generic_blocking_transfer(write);
        let mut read_idx = 0;
        let max_idx = core::cmp::max(write.len(), read.len());

        let mut writes_finished = write_idx == max_idx;
        let mut reads_finished = false;
        while !writes_finished || !reads_finished {
            let status = self.regs().read_isr();
            if status.rx_not_empty() && !reads_finished {
                if read_idx < read.len() {
                    read[read_idx] = self.inner.read_fifo_unchecked();
                } else {
                    // Discard the data.
                    self.inner.read_fifo_unchecked();
                }
                read_idx += 1;
            }

            if !status.tx_full() && !writes_finished {
                if write_idx < write.len() {
                    self.inner.write_fifo_unchecked(write[write_idx]);
                } else {
                    // Send dummy bytes.
                    self.inner.write_fifo_unchecked(0);
                }
                write_idx += 1;
            }
            writes_finished = write_idx == max_idx;
            reads_finished = read_idx == max_idx;
        }

        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        let mut write_idx = self.prepare_generic_blocking_transfer(words);
        let mut read_idx = 0;

        let mut writes_finished = write_idx == words.len();
        let mut reads_finished = false;
        while !writes_finished || !reads_finished {
            let status = self.inner.read_isr();
            if status.rx_not_empty() && !reads_finished {
                words[read_idx] = self.inner.read_fifo_unchecked();
                read_idx += 1;
            }

            if !status.tx_full() && !writes_finished {
                self.inner.write_fifo_unchecked(words[write_idx]);
                write_idx += 1;
            }
            writes_finished = write_idx == words.len();
            reads_finished = read_idx == words.len();
        }

        Ok(())
    }

    /// Blocking flush implementation.
    fn flush(&mut self) -> Result<(), Self::Error> {
        if !self.outstanding_rx {
            return Ok(());
        }
        let rx_trig = self.inner.read_rx_not_empty_threshold();
        while !self.inner.read_isr().rx_not_empty() {}
        (0..rx_trig).for_each(|_| {
            self.inner.read_fifo_unchecked();
        });
        self.inner.set_rx_fifo_trigger(1).unwrap();
        self.outstanding_rx = false;
        Ok(())
    }
}

pub struct SpiWithHwCs<Delay: DelayNs> {
    pub spi: Spi,
    pub cs: ChipSelect,
    pub delay: Delay,
}

impl<Delay: DelayNs> SpiWithHwCs<Delay> {
    pub fn new(spi: Spi, cs: ChipSelect, delay: Delay) -> Self {
        Self { spi, cs, delay }
    }

    pub fn release(self) -> Spi {
        self.spi
    }
}

impl<Delay: DelayNs> embedded_hal::spi::ErrorType for SpiWithHwCs<Delay> {
    type Error = Infallible;
}

impl<Delay: DelayNs> embedded_hal::spi::SpiDevice for SpiWithHwCs<Delay> {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.spi.inner.select_hw_cs(self.cs);
        for op in operations {
            match op {
                embedded_hal::spi::Operation::Read(items) => {
                    self.spi.read(items)?;
                }
                embedded_hal::spi::Operation::Write(items) => {
                    self.spi.write(items)?;
                }
                embedded_hal::spi::Operation::Transfer(read, write) => {
                    self.spi.transfer(read, write)?;
                }
                embedded_hal::spi::Operation::TransferInPlace(items) => {
                    self.spi.transfer_in_place(items)?;
                }
                embedded_hal::spi::Operation::DelayNs(delay) => {
                    self.delay.delay_ns(*delay);
                }
            }
        }
        self.spi.flush()?;
        self.spi.inner.no_hw_cs();
        Ok(())
    }
}

/// Reset the SPI peripheral using the SLCR reset register for SPI.
///
/// Please note that this function will interfere with an already configured
/// SPI instance.
#[inline]
pub fn reset(id: SpiId) {
    let assert_reset = match id {
        SpiId::Spi0 => DualRefAndClockReset::builder()
            .with_periph1_ref_rst(false)
            .with_periph0_ref_rst(true)
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        SpiId::Spi1 => DualRefAndClockReset::builder()
            .with_periph1_ref_rst(true)
            .with_periph0_ref_rst(false)
            .with_periph1_cpu1x_rst(true)
            .with_periph0_cpu1x_rst(false)
            .build(),
    };
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl().write_spi(assert_reset);
            // Keep it in reset for some cycles.. The TMR just mentions some small delay,
            // no idea what is meant with that.
            for _ in 0..3 {
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_spi(DualRefAndClockReset::DEFAULT);
        });
    }
}

/// Calculates the largest allowed SPI reference clock divisor.
///
/// The Zynq7000 SPI peripheral has the following requirement for the SPI reference clock:
/// It must be larger than the CPU 1X clock. Therefore, the divisor used to calculate the reference
/// clock has a maximum value, which can be calculated with this function.
///
/// [configure_spi_ref_clk] can be used to configure the SPI reference clock with the calculated
/// value.
pub fn calculate_largest_allowed_spi_ref_clk_divisor(clks: &Clocks) -> Option<u6> {
    let slcr = unsafe { Slcr::steal() };
    let spi_clk_ctrl = slcr.regs().clk_ctrl_shared().read_spi_clk_ctrl();
    let div = match spi_clk_ctrl.srcsel() {
        zynq7000::slcr::clocks::SrcSelIo::IoPll | zynq7000::slcr::clocks::SrcSelIo::IoPllAlt => {
            clks.io_clocks().ref_clk() / clks.arm_clocks().cpu_1x_clk()
        }
        zynq7000::slcr::clocks::SrcSelIo::ArmPll => {
            clks.arm_clocks().ref_clk() / clks.arm_clocks().cpu_1x_clk()
        }
        zynq7000::slcr::clocks::SrcSelIo::DdrPll => {
            clks.ddr_clocks().ref_clk() / clks.arm_clocks().cpu_1x_clk()
        }
    };
    if div > u6::MAX.value() as u32 {
        return None;
    }

    Some(u6::new(div as u8))
}

pub fn configure_spi_ref_clk(clks: &mut Clocks, divisor: u6) {
    let mut slcr = unsafe { Slcr::steal() };
    let spi_clk_ctrl = slcr.regs().clk_ctrl_shared().read_spi_clk_ctrl();
    slcr.modify(|regs| {
        regs.clk_ctrl().modify_spi_clk_ctrl(|mut val| {
            val.set_divisor(divisor);
            val
        });
    });
    let new_clk = match spi_clk_ctrl.srcsel() {
        zynq7000::slcr::clocks::SrcSelIo::IoPll | zynq7000::slcr::clocks::SrcSelIo::IoPllAlt => {
            clks.io_clocks().ref_clk() / divisor.value() as u32
        }
        zynq7000::slcr::clocks::SrcSelIo::ArmPll => {
            clks.arm_clocks().ref_clk() / divisor.value() as u32
        }
        zynq7000::slcr::clocks::SrcSelIo::DdrPll => {
            clks.ddr_clocks().ref_clk() / divisor.value() as u32
        }
    };
    clks.io_clocks_mut().update_spi_clk(new_clk);
}
