//! # QSPI module
use core::ops::{Deref, DerefMut};

use arbitrary_int::{prelude::*, u2, u3, u6};
pub use zynq7000::qspi::LinearQspiConfig;
use zynq7000::{
    qspi::{
        BaudRateDivisor, Config, InstructionCode, InterruptStatus, LoopbackMasterClockDelay,
        SpiEnable,
    },
    slcr::{clocks::SingleCommonPeriphIoClockControl, mio::Speed, reset::ResetControlQspiSmc},
};

pub use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3, Mode};
pub use zynq7000::slcr::clocks::SrcSelIo;
pub use zynq7000::slcr::mio::IoType;

use crate::{
    PeriphSelect,
    clocks::Clocks,
    enable_amba_peripheral_clock,
    gpio::{
        IoPeriphPin,
        mio::{
            Mio0, Mio1, Mio2, Mio3, Mio4, Mio5, Mio6, Mio8, Mio9, Mio10, Mio11, Mio12, Mio13,
            MioPin, MuxConfig, Pin,
        },
    },
    slcr::Slcr,
    spi_mode_const_to_cpol_cpha,
    time::Hertz,
};

pub(crate) mod lqspi_configs;

/// MIO multiplexer configuration for the QSPI pins.
pub const QSPI_MUX_CONFIG: MuxConfig = MuxConfig::new_with_l0();
/// Depth of the TX and RX FIFO in 32 bit words.
pub const FIFO_DEPTH: usize = 63;
/// From the TRM:
///
/// > The maximum number of bytes per command sequence in this mode is limited by the depth of the
/// > TxFIFO of 252 bytes.
pub const MAX_BYTES_PER_TRANSFER_IO_MODE: usize = FIFO_DEPTH * 4;

/// In linear-addressed mode, the QSPI is memory-mapped, with the address starting here.
pub const QSPI_START_ADDRESS: usize = 0xFC00_0000;

/// Error for the [ClockConfig] calculation.
#[derive(Debug, thiserror::Error)]
pub enum ClockCalculationError {
    /// The QSPI reference clock must be greater than the CPU 1x clock.
    #[error("violated clock ratio restriction")]
    RefClockSmallerThanCpu1xClock,
    /// The reference clock divisor is out of range.
    #[error("reference divisor out of range")]
    RefDivOutOfRange,
}

/// Baud rate configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaudRateConfig {
    /// Loopback clock mode, which requires a baud rate divisor of 2.
    WithLoopback,
    /// No loopback clock, with the given baud rate divisor.
    WithoutLoopback(BaudRateDivisor),
}

impl BaudRateConfig {
    /// Baud rate divisor.
    #[inline]
    pub const fn baud_rate_divisor(&self) -> BaudRateDivisor {
        match self {
            BaudRateConfig::WithLoopback => BaudRateDivisor::_2,
            BaudRateConfig::WithoutLoopback(divisor) => *divisor,
        }
    }
}

/// QSPI clock configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    /// Reference clock source.
    pub src_sel: SrcSelIo,
    /// Reference clock divisor.
    pub ref_clk_div: u6,
    /// Baud rate configuration.
    pub baud_rate_config: BaudRateConfig,
}

/// QSPI flash vendor, which determines the linear mode configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QspiVendor {
    /// Winbond or Spansion.
    WinbondAndSpansion,
    /// Micron.
    Micron,
}

/// Read instruction used in linear addressing mode.
pub type OperatingMode = InstructionCode;

/// MIO pin which can be used as QSPI 0 chip select.
pub trait Qspi0ChipSelectPin: MioPin {}
/// MIO pin which can be used as QSPI 0 IO0.
pub trait Qspi0Io0Pin: MioPin {}
/// MIO pin which can be used as QSPI 0 IO1.
pub trait Qspi0Io1Pin: MioPin {}
/// MIO pin which can be used as QSPI 0 IO2.
pub trait Qspi0Io2Pin: MioPin {}
/// MIO pin which can be used as QSPI 0 IO3.
pub trait Qspi0Io3Pin: MioPin {}
/// MIO pin which can be used as QSPI 0 clock.
pub trait Qspi0ClockPin: MioPin {}

impl Qspi0ChipSelectPin for Pin<Mio1> {}
impl Qspi0Io0Pin for Pin<Mio2> {}
impl Qspi0Io1Pin for Pin<Mio3> {}
impl Qspi0Io2Pin for Pin<Mio4> {}
impl Qspi0Io3Pin for Pin<Mio5> {}
impl Qspi0ClockPin for Pin<Mio6> {}

/// MIO pin which can be used as QSPI 1 chip select.
pub trait Qspi1ChipSelectPin: MioPin {}
/// MIO pin which can be used as QSPI 1 IO0.
pub trait Qspi1Io0Pin: MioPin {}
/// MIO pin which can be used as QSPI 1 IO1.
pub trait Qspi1Io1Pin: MioPin {}
/// MIO pin which can be used as QSPI 1 IO2.
pub trait Qspi1Io2Pin: MioPin {}
/// MIO pin which can be used as QSPI 1 IO3.
pub trait Qspi1Io3Pin: MioPin {}
/// MIO pin which can be used as QSPI 1 clock.
pub trait Qspi1ClockPin: MioPin {}

impl Qspi1ChipSelectPin for Pin<Mio0> {}
impl Qspi1Io0Pin for Pin<Mio10> {}
impl Qspi1Io1Pin for Pin<Mio11> {}
impl Qspi1Io2Pin for Pin<Mio12> {}
impl Qspi1Io3Pin for Pin<Mio13> {}
impl Qspi1ClockPin for Pin<Mio9> {}

/// MIO pin which can be used as QSPI feedback clock.
pub trait FeedbackClockPin: MioPin {}

impl FeedbackClockPin for Pin<Mio8> {}

/// QSPI flash setup, used to derive the linear mode configuration.
pub struct QspiDeviceCombination {
    /// Flash vendor.
    pub vendor: QspiVendor,
    /// Read instruction.
    pub operating_mode: OperatingMode,
    /// Two flash devices are used.
    pub two_devices: bool,
}

impl From<QspiDeviceCombination> for LinearQspiConfig {
    fn from(value: QspiDeviceCombination) -> Self {
        linear_mode_config_for_common_devices(value)
    }
}

/// Linear mode configuration for common QSPI flash devices.
pub const fn linear_mode_config_for_common_devices(
    dev_combination: QspiDeviceCombination,
) -> LinearQspiConfig {
    match dev_combination.operating_mode {
        InstructionCode::Read => {
            if dev_combination.two_devices {
                lqspi_configs::RD_TWO
            } else {
                lqspi_configs::RD_ONE
            }
        }
        InstructionCode::FastRead => {
            if dev_combination.two_devices {
                lqspi_configs::FAST_RD_TWO
            } else {
                lqspi_configs::FAST_RD_ONE
            }
        }
        InstructionCode::FastReadDualOutput => {
            if dev_combination.two_devices {
                lqspi_configs::DUAL_OUT_FAST_RD_TWO
            } else {
                lqspi_configs::DUAL_OUT_FAST_RD_ONE
            }
        }
        InstructionCode::FastReadQuadOutput => {
            if dev_combination.two_devices {
                lqspi_configs::QUAD_OUT_FAST_RD_TWO
            } else {
                lqspi_configs::QUAD_OUT_FAST_RD_ONE
            }
        }
        InstructionCode::FastReadDualIo => {
            match (dev_combination.vendor, dev_combination.two_devices) {
                (QspiVendor::WinbondAndSpansion, false) => {
                    lqspi_configs::winbond_spansion::DUAL_IO_FAST_RD_ONE
                }
                (QspiVendor::WinbondAndSpansion, true) => {
                    lqspi_configs::winbond_spansion::DUAL_IO_FAST_RD_TWO
                }
                (QspiVendor::Micron, false) => lqspi_configs::micron::DUAL_IO_FAST_RD_ONE,
                (QspiVendor::Micron, true) => lqspi_configs::micron::DUAL_IO_FAST_RD_TWO,
            }
        }
        InstructionCode::FastReadQuadIo => {
            match (dev_combination.vendor, dev_combination.two_devices) {
                (QspiVendor::WinbondAndSpansion, false) => {
                    lqspi_configs::winbond_spansion::QUAD_IO_FAST_RD_ONE
                }
                (QspiVendor::WinbondAndSpansion, true) => {
                    lqspi_configs::winbond_spansion::QUAD_IO_FAST_RD_TWO
                }
                (QspiVendor::Micron, false) => lqspi_configs::micron::QUAD_IO_FAST_RD_ONE,
                (QspiVendor::Micron, true) => lqspi_configs::micron::QUAD_IO_FAST_RD_TWO,
            }
        }
    }
}

impl ClockConfig {
    /// Create a new configuration.
    pub fn new(src_sel: SrcSelIo, ref_clk_div: u6, baud_rate_config: BaudRateConfig) -> Self {
        Self {
            src_sel,
            ref_clk_div,
            baud_rate_config,
        }
    }

    /// This constructor calculates the necessary clock divisor for a target QSPI reference clock,
    /// assuming that a loopback clock is used and thus constraining the baud rate divisor to 2.
    ///
    /// It also checks that the clock ratio restriction is not violated: The QSPI reference clock must
    /// be greater than the CPU 1x clock.
    pub fn calculate_with_loopback(
        src_sel: SrcSelIo,
        clocks: &Clocks,
        target_qspi_interface_clock: Hertz,
    ) -> Result<Self, ClockCalculationError> {
        // For loopback mode, the baud rate divisor MUST be 2.
        let target_ref_clock = target_qspi_interface_clock * 2;
        let ref_clk = match src_sel {
            SrcSelIo::IoPll | SrcSelIo::IoPllAlt => clocks.io_clocks().ref_clk(),
            SrcSelIo::ArmPll => clocks.arm_clocks().ref_clk(),
            SrcSelIo::DdrPll => clocks.ddr_clocks().ref_clk(),
        };
        let ref_clk_div = ref_clk.to_raw().div_ceil(target_ref_clock.to_raw());
        if ref_clk_div > u6::MAX.as_u32() {
            return Err(ClockCalculationError::RefDivOutOfRange);
        }
        Ok(Self {
            src_sel,
            ref_clk_div: u6::new(ref_clk_div as u8),
            baud_rate_config: BaudRateConfig::WithLoopback,
        })
    }

    /// This constructor calculates the necessary clock configuration for both a target QSPI
    /// reference clock as well as a target QSPI interface clock.
    ///
    /// It also checks that the clock ratio restriction is not violated: The QSPI reference clock must
    /// be greater than the CPU 1x clock.
    pub fn calculate(
        src_sel: SrcSelIo,
        clocks: &Clocks,
        target_qspi_ref_clock: Hertz,
        target_qspi_interface_clock: Hertz,
    ) -> Result<Self, ClockCalculationError> {
        let (ref_clk_div, ref_clk) = match src_sel {
            SrcSelIo::IoPll | SrcSelIo::IoPllAlt => (
                clocks
                    .io_clocks()
                    .ref_clk()
                    .to_raw()
                    .div_ceil(target_qspi_ref_clock.to_raw()),
                clocks.io_clocks().ref_clk(),
            ),
            SrcSelIo::ArmPll => (
                clocks
                    .arm_clocks()
                    .ref_clk()
                    .to_raw()
                    .div_ceil(target_qspi_ref_clock.to_raw()),
                clocks.arm_clocks().ref_clk(),
            ),
            SrcSelIo::DdrPll => (
                clocks
                    .ddr_clocks()
                    .ref_clk()
                    .to_raw()
                    .div_ceil(target_qspi_ref_clock.to_raw()),
                clocks.ddr_clocks().ref_clk(),
            ),
        };
        if ref_clk_div > u6::MAX.as_u32() {
            return Err(ClockCalculationError::RefDivOutOfRange);
        }
        let qspi_ref_clk = ref_clk / ref_clk_div;
        if qspi_ref_clk < clocks.arm_clocks().cpu_1x_clk() {
            return Err(ClockCalculationError::RefClockSmallerThanCpu1xClock);
        }
        let qspi_baud_rate_div = qspi_ref_clk
            .to_raw()
            .div_ceil(target_qspi_interface_clock.to_raw());
        let baud_rate_div = match qspi_baud_rate_div {
            0..=2 => BaudRateDivisor::_2,
            3..=4 => BaudRateDivisor::_4,
            5..=8 => BaudRateDivisor::_8,
            9..=16 => BaudRateDivisor::_16,
            17..=32 => BaudRateDivisor::_32,
            65..=128 => BaudRateDivisor::_64,
            129..=256 => BaudRateDivisor::_128,
            _ => BaudRateDivisor::_256,
        };
        Ok(Self {
            src_sel,
            ref_clk_div: u6::new(ref_clk_div as u8),
            baud_rate_config: BaudRateConfig::WithoutLoopback(baud_rate_div),
        })
    }
}

/// Low-level QSPI driver.
pub struct QspiLowLevel(zynq7000::qspi::MmioRegisters<'static>);

impl QspiLowLevel {
    /// Create a low-level driver. This does not configure the peripheral.
    #[inline]
    pub fn new(regs: zynq7000::qspi::MmioRegisters<'static>) -> Self {
        Self(regs)
    }

    /// QSPI registers.
    pub fn regs(&mut self) -> &mut zynq7000::qspi::MmioRegisters<'static> {
        &mut self.0
    }

    /// Reset the peripheral, then configure the clock and the SPI mode.
    pub fn initialize(&mut self, clock_config: ClockConfig, mode: embedded_hal::spi::Mode) {
        enable_amba_peripheral_clock(PeriphSelect::Lqspi);
        reset();
        let (cpol, cpha) = spi_mode_const_to_cpol_cpha(mode);
        // SAFETY: Only the QSPI clock control register is written.
        unsafe {
            Slcr::with(|slcr| {
                slcr.clk_ctrl().write_lqspi_clk_ctrl(
                    SingleCommonPeriphIoClockControl::builder()
                        .with_divisor(clock_config.ref_clk_div)
                        .with_srcsel(clock_config.src_sel)
                        .with_clk_act(true)
                        .build(),
                );
            })
        }
        let baudrate_config = clock_config.baud_rate_config;
        self.0.write_config(
            Config::builder()
                .with_interface_mode(zynq7000::qspi::InterfaceMode::FlashMemoryInterface)
                .with_edianness(zynq7000::qspi::Endianness::Little)
                .with_holdb_dr(true)
                .with_manual_start_command(false)
                .with_manual_start_enable(false)
                .with_manual_cs(false)
                .with_peripheral_chip_select(false)
                .with_fifo_width(u2::new(0b11))
                .with_baud_rate_div(baudrate_config.baud_rate_divisor())
                .with_clock_phase(cpha)
                .with_clock_polarity(cpol)
                .with_mode_select(true)
                .build(),
        );
        if baudrate_config == BaudRateConfig::WithLoopback {
            self.0.write_loopback_master_clock_delay(
                LoopbackMasterClockDelay::builder()
                    .with_use_loopback(true)
                    .with_delay_1(u2::new(0x0))
                    .with_delay_0(u3::new(0x0))
                    .build(),
            );
        }
    }

    /// Switch to linear addressing mode with the given configuration.
    pub fn enable_linear_addressing(&mut self, config: LinearQspiConfig) {
        self.0
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
        self.0.modify_config(|mut val| {
            // Those two bits should be set to 0 according to the TRM.
            val.set_manual_start_enable(false);
            val.set_manual_cs(false);
            val.set_peripheral_chip_select(false);
            val
        });
        self.0.write_linear_qspi_config(config);
    }

    /// Switch to IO mode.
    pub fn enable_io_mode(&mut self, dual_flash: bool) {
        self.0.modify_config(|mut val| {
            val.set_manual_start_enable(true);
            val.set_manual_cs(true);
            val
        });
        self.0.write_rx_fifo_threshold(0x1);
        self.0.write_tx_fifo_threshold(0x1);
        self.0.write_linear_qspi_config(
            LinearQspiConfig::builder()
                .with_enable_linear_mode(false)
                .with_both_memories(dual_flash)
                .with_separate_memory_bus(dual_flash)
                .with_upper_memory_page(false)
                .with_mode_enable(false)
                .with_mode_on(true)
                // Reset values from the TRM are set here, but they do not matter anyway.
                .with_mode_bits(0xA0)
                .with_num_dummy_bytes(u3::new(0x2))
                .with_instruction_code(InstructionCode::FastReadQuadIo)
                .build(),
        );
    }

    /// Disable the peripheral and deassert the chip select.
    pub fn disable(&mut self) {
        self.0
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
        self.0.modify_config(|mut val| {
            val.set_peripheral_chip_select(true);
            val
        });
    }
}

/// QSPI driver before the mode was selected.
pub struct Qspi {
    ll: QspiLowLevel,
}

impl Qspi {
    /// Create a driver for a single flash connected to the QSPI 0 MIO pins.
    pub fn new_single_qspi<
        ChipSelect: Qspi0ChipSelectPin,
        Io0: Qspi0Io0Pin,
        Io1: Qspi0Io1Pin,
        Io2: Qspi0Io2Pin,
        Io3: Qspi0Io3Pin,
        Clock: Qspi0ClockPin,
    >(
        regs: zynq7000::qspi::MmioRegisters<'static>,
        clock_config: ClockConfig,
        mode: embedded_hal::spi::Mode,
        voltage: IoType,
        cs: ChipSelect,
        ios: (Io0, Io1, Io2, Io3),
        clock: Clock,
    ) -> Self {
        IoPeriphPin::new_with_full_config(
            cs,
            zynq7000::slcr::mio::Config::builder()
                .with_disable_hstl_rcvr(false)
                .with_pullup(true)
                .with_io_type(voltage)
                .with_speed(Speed::FastCmosEdge)
                .with_l3_sel(QSPI_MUX_CONFIG.l3_sel())
                .with_l2_sel(QSPI_MUX_CONFIG.l2_sel())
                .with_l1_sel(QSPI_MUX_CONFIG.l1_sel())
                .with_l0_sel(QSPI_MUX_CONFIG.l0_sel())
                .with_tri_enable(false)
                .build(),
        );
        let io_and_clock_config = zynq7000::slcr::mio::Config::builder()
            .with_disable_hstl_rcvr(false)
            .with_pullup(false)
            .with_io_type(voltage)
            .with_speed(Speed::FastCmosEdge)
            .with_l3_sel(QSPI_MUX_CONFIG.l3_sel())
            .with_l2_sel(QSPI_MUX_CONFIG.l2_sel())
            .with_l1_sel(QSPI_MUX_CONFIG.l1_sel())
            .with_l0_sel(QSPI_MUX_CONFIG.l0_sel())
            .with_tri_enable(false)
            .build();
        IoPeriphPin::new_with_full_config(ios.0, io_and_clock_config);
        IoPeriphPin::new_with_full_config(ios.1, io_and_clock_config);
        IoPeriphPin::new_with_full_config(ios.2, io_and_clock_config);
        IoPeriphPin::new_with_full_config(ios.3, io_and_clock_config);
        IoPeriphPin::new_with_full_config(clock, io_and_clock_config);
        let mut ll = QspiLowLevel::new(regs);
        ll.initialize(clock_config, mode);
        Self { ll }
    }

    /// Like [Self::new_single_qspi], with the additional feedback clock pin.
    #[allow(clippy::too_many_arguments)]
    pub fn new_single_qspi_with_feedback<
        ChipSelect: Qspi0ChipSelectPin,
        Io0: Qspi0Io0Pin,
        Io1: Qspi0Io1Pin,
        Io2: Qspi0Io2Pin,
        Io3: Qspi0Io3Pin,
        Clock: Qspi0ClockPin,
        Feedback: FeedbackClockPin,
    >(
        regs: zynq7000::qspi::MmioRegisters<'static>,
        clock_config: ClockConfig,
        mode: embedded_hal::spi::Mode,
        voltage: IoType,
        cs: ChipSelect,
        io: (Io0, Io1, Io2, Io3),
        clock: Clock,
        feedback: Feedback,
    ) -> Self {
        IoPeriphPin::new_with_full_config(
            feedback,
            zynq7000::slcr::mio::Config::builder()
                .with_disable_hstl_rcvr(false)
                .with_pullup(false)
                .with_io_type(voltage)
                .with_speed(Speed::FastCmosEdge)
                .with_l3_sel(QSPI_MUX_CONFIG.l3_sel())
                .with_l2_sel(QSPI_MUX_CONFIG.l2_sel())
                .with_l1_sel(QSPI_MUX_CONFIG.l1_sel())
                .with_l0_sel(QSPI_MUX_CONFIG.l0_sel())
                .with_tri_enable(false)
                .build(),
        );
        Self::new_single_qspi(regs, clock_config, mode, voltage, cs, io, clock)
    }

    /// QSPI registers.
    #[inline]
    pub fn regs(&mut self) -> &mut zynq7000::qspi::MmioRegisters<'static> {
        &mut self.ll.0
    }

    /// Switch to linear addressing mode.
    pub fn into_linear_addressed(mut self, config: LinearQspiConfig) -> QspiLinearAddressing {
        self.ll.enable_linear_addressing(config);
        QspiLinearAddressing { ll: self.ll }
    }

    /// Switch to IO mode.
    pub fn into_io_mode(mut self, dual_flash: bool) -> QspiIoMode {
        self.ll.enable_io_mode(dual_flash);
        QspiIoMode { ll: self.ll }
    }
}

/// QSPI driver in IO mode, where commands are sent manually.
pub struct QspiIoMode {
    ll: QspiLowLevel,
}

impl QspiIoMode {
    /// QSPI registers.
    #[inline]
    pub fn regs(&mut self) -> &mut zynq7000::qspi::MmioRegisters<'static> {
        &mut self.ll.0
    }

    /// Create a guard for one transfer.
    pub fn transfer_guard(&mut self) -> QspiIoTransferGuard<'_> {
        QspiIoTransferGuard::new(self)
    }

    /// Disable the peripheral and convert back into a [Qspi] driver.
    pub fn into_qspi(mut self, ll: QspiLowLevel) -> Qspi {
        self.ll.disable();
        Qspi { ll }
    }

    /// Transmits 1-byte command and 3-byte data OR 4-byte data.
    #[inline]
    pub fn write_word_txd_00(&mut self, word: u32) {
        self.regs().write_tx_data_00(word);
    }

    /// Transmits 1-byte command.
    #[inline]
    pub fn write_word_txd_01(&mut self, word: u32) {
        self.regs().write_tx_data_01(word);
    }

    /// Transmits 1-byte command and 1-byte data.
    #[inline]
    pub fn write_word_txd_10(&mut self, word: u32) {
        self.regs().write_tx_data_10(word);
    }

    /// Transmits 1-byte command and 2-byte data.
    #[inline]
    pub fn write_word_txd_11(&mut self, word: u32) {
        self.regs().write_tx_data_11(word);
    }

    /// Read one word from the RX FIFO.
    #[inline]
    pub fn read_rx_data(&mut self) -> u32 {
        self.regs().read_rx_data()
    }

    /// Assert the chip select and enable the peripheral.
    pub fn transfer_init(&mut self) {
        self.regs().modify_config(|mut val| {
            val.set_peripheral_chip_select(false);
            val
        });
        self.regs()
            .write_spi_enable(SpiEnable::builder().with_enable(true).build());
    }

    /// Start the transfer of the TX FIFO content.
    pub fn transfer_start(&mut self) {
        self.regs().modify_config(|mut val| {
            val.set_manual_start_command(true);
            val
        });
    }

    /// Deassert the chip select and disable the peripheral.
    pub fn transfer_done(&mut self) {
        self.regs().modify_config(|mut val| {
            val.set_peripheral_chip_select(true);
            val
        });
        self.regs()
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
    }

    /// Read the interrupt status.
    pub fn read_status(&mut self) -> InterruptStatus {
        self.regs().read_interrupt_status()
    }

    /// Discard the RX FIFO content.
    pub fn clear_rx_fifo(&mut self) {
        while self.read_status().rx_above_threshold() {
            self.read_rx_data();
        }
    }

    /// Switch to linear addressing mode.
    pub fn into_linear_addressed(mut self, config: LinearQspiConfig) -> QspiLinearAddressing {
        self.ll.enable_linear_addressing(config);
        QspiLinearAddressing { ll: self.ll }
    }
}

/// This guard structure takes care of commonly required operations before starting a transfer
/// and after finishing it.
pub struct QspiIoTransferGuard<'a>(&'a mut QspiIoMode);

impl<'a> QspiIoTransferGuard<'a> {
    /// Clear the RX FIFO and start the transfer setup.
    pub fn new(qspi: &'a mut QspiIoMode) -> Self {
        qspi.clear_rx_fifo();
        qspi.transfer_init();
        Self(qspi)
    }
}

impl QspiIoTransferGuard<'_> {
    /// Start the transfer of the TX FIFO content.
    pub fn start(&mut self) {
        self.0.transfer_start();
    }
}

impl Deref for QspiIoTransferGuard<'_> {
    type Target = QspiIoMode;

    fn deref(&self) -> &Self::Target {
        self.0
    }
}

impl DerefMut for QspiIoTransferGuard<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0
    }
}

impl<'a> Drop for QspiIoTransferGuard<'a> {
    fn drop(&mut self) {
        self.0.transfer_done();
    }
}

/// QSPI driver in linear addressing mode, where the flash is memory mapped.
pub struct QspiLinearAddressing {
    ll: QspiLowLevel,
}

impl QspiLinearAddressing {
    /// Memory-mapped QSPI base address.
    pub const BASE_ADDRESS: usize = QSPI_START_ADDRESS;

    /// Switch to IO mode.
    pub fn into_io_mode(mut self, dual_flash: bool) -> QspiIoMode {
        self.ll.enable_io_mode(dual_flash);
        QspiIoMode { ll: self.ll }
    }

    /// Enable the peripheral for memory mapped reads while the guard lives.
    pub fn read_guard(&mut self) -> QspiLinearReadGuard<'_> {
        QspiLinearReadGuard::new(self)
    }
}

/// Keeps the peripheral enabled for memory mapped reads.
pub struct QspiLinearReadGuard<'a>(&'a mut QspiLinearAddressing);

impl QspiLinearReadGuard<'_> {
    /// Memory-mapped QSPI base address.
    pub const BASE_ADDRESS: usize = QSPI_START_ADDRESS;

    /// Enable the peripheral.
    pub fn new(qspi: &mut QspiLinearAddressing) -> QspiLinearReadGuard<'_> {
        qspi.ll
            .0
            .write_spi_enable(SpiEnable::builder().with_enable(true).build());
        QspiLinearReadGuard(qspi)
    }
}

impl Drop for QspiLinearReadGuard<'_> {
    fn drop(&mut self) {
        self.0
            .ll
            .0
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
    }
}

/// Reset the QSPI peripheral using the SLCR reset register for QSPI.
///
/// Please note that this function will interfere with an already configured
/// QSPI instance.
#[inline]
pub fn reset() {
    // SAFETY: Only the QSPI reset register is written.
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl().write_lqspi(
                ResetControlQspiSmc::builder()
                    .with_ref_reset(true)
                    .with_cpu_1x_reset(true)
                    .build(),
            );
            // Keep it in reset for some cycles.
            for _ in 0..10 {
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_lqspi(ResetControlQspiSmc::DEFAULT);
        });
    }
}
