//! # QSPI module
//!
//! ## Examples
//!
//! - [Zedboard QSPI](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq/examples/zedboard/src/bin/qspi.rs)
use core::ops::{Deref, DerefMut};

use arbitrary_int::{prelude::*, u2, u3, u6};
pub use zynq7000::qspi::LinearQspiConfig;
use zynq7000::{
    qspi::{
        BaudRateDivisor, Config, InstructionCode, InterruptStatus, LoopbackMasterClockDelay,
        SpiEnable,
    },
    slcr::{clocks::SingleCommonPeriphIoClockControl, mio::Speed, reset::QspiResetControl},
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

pub const QSPI_MUX_CONFIG: MuxConfig = MuxConfig::new_with_l0();
pub const FIFO_DEPTH: usize = 63;
/// In linear-addressed mode, the QSPI is memory-mapped, with the address starting here.
pub const QSPI_START_ADDRESS: usize = 0xFC00_0000;

#[derive(Debug, thiserror::Error)]
pub enum ClockCalculationError {
    #[error("violated clock ratio restriction")]
    RefClockSmallerThanCpu1xClock,
    #[error("reference divisor out of range")]
    RefDivOutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaudRateConfig {
    WithLoopback,
    WithoutLoopback(BaudRateDivisor),
}

impl BaudRateConfig {
    #[inline]
    pub const fn baud_rate_divisor(&self) -> BaudRateDivisor {
        match self {
            BaudRateConfig::WithLoopback => BaudRateDivisor::_2,
            BaudRateConfig::WithoutLoopback(divisor) => *divisor,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    pub src_sel: SrcSelIo,
    pub ref_clk_div: u6,
    pub baud_rate_config: BaudRateConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QspiVendor {
    WinbondAndSpansion,
    Micron,
}

pub type OperatingMode = InstructionCode;

pub trait Qspi0ChipSelectPin: MioPin {}
pub trait Qspi0Io0Pin: MioPin {}
pub trait Qspi0Io1Pin: MioPin {}
pub trait Qspi0Io2Pin: MioPin {}
pub trait Qspi0Io3Pin: MioPin {}
pub trait Qspi0ClockPin: MioPin {}

impl Qspi0ChipSelectPin for Pin<Mio1> {}
impl Qspi0Io0Pin for Pin<Mio2> {}
impl Qspi0Io1Pin for Pin<Mio3> {}
impl Qspi0Io2Pin for Pin<Mio4> {}
impl Qspi0Io3Pin for Pin<Mio5> {}
impl Qspi0ClockPin for Pin<Mio6> {}

pub trait Qspi1ChipSelectPin: MioPin {}
pub trait Qspi1Io0Pin: MioPin {}
pub trait Qspi1Io1Pin: MioPin {}
pub trait Qspi1Io2Pin: MioPin {}
pub trait Qspi1Io3Pin: MioPin {}
pub trait Qspi1ClockPin: MioPin {}

impl Qspi1ChipSelectPin for Pin<Mio0> {}
impl Qspi1Io0Pin for Pin<Mio10> {}
impl Qspi1Io1Pin for Pin<Mio11> {}
impl Qspi1Io2Pin for Pin<Mio12> {}
impl Qspi1Io3Pin for Pin<Mio13> {}
impl Qspi1ClockPin for Pin<Mio9> {}

pub trait FeedbackClockPin: MioPin {}

impl FeedbackClockPin for Pin<Mio8> {}

pub struct QspiDeviceCombination {
    pub vendor: QspiVendor,
    pub operating_mode: OperatingMode,
    pub two_devices: bool,
}

impl From<QspiDeviceCombination> for LinearQspiConfig {
    fn from(value: QspiDeviceCombination) -> Self {
        linear_mode_config_for_common_devices(value)
    }
}

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
        let ref_clk_div = ref_clk.raw().div_ceil(target_ref_clock.raw());
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
                    .raw()
                    .div_ceil(target_qspi_ref_clock.raw()),
                clocks.io_clocks().ref_clk(),
            ),
            SrcSelIo::ArmPll => (
                clocks
                    .arm_clocks()
                    .ref_clk()
                    .raw()
                    .div_ceil(target_qspi_ref_clock.raw()),
                clocks.arm_clocks().ref_clk(),
            ),
            SrcSelIo::DdrPll => (
                clocks
                    .ddr_clocks()
                    .ref_clk()
                    .raw()
                    .div_ceil(target_qspi_ref_clock.raw()),
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
        let qspi_baud_rate_div = qspi_ref_clk / target_qspi_interface_clock;
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

pub struct QspiLowLevel(zynq7000::qspi::MmioQspi<'static>);

impl QspiLowLevel {
    #[inline]
    pub fn new(regs: zynq7000::qspi::MmioQspi<'static>) -> Self {
        Self(regs)
    }

    pub fn regs(&mut self) -> &mut zynq7000::qspi::MmioQspi<'static> {
        &mut self.0
    }

    pub fn initialize(&mut self, clock_config: ClockConfig, mode: embedded_hal::spi::Mode) {
        enable_amba_peripheral_clock(PeriphSelect::Lqspi);
        reset();
        let (cpol, cpha) = spi_mode_const_to_cpol_cpha(mode);
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

    pub fn disable(&mut self) {
        self.0
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
        self.0.modify_config(|mut val| {
            val.set_peripheral_chip_select(true);
            val
        });
    }
}

pub struct Qspi {
    ll: QspiLowLevel,
}

impl Qspi {
    pub fn new_single_qspi<
        ChipSelect: Qspi0ChipSelectPin,
        Io0: Qspi0Io0Pin,
        Io1: Qspi0Io1Pin,
        Io2: Qspi0Io2Pin,
        Io3: Qspi0Io3Pin,
        Clock: Qspi0ClockPin,
    >(
        regs: zynq7000::qspi::MmioQspi<'static>,
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
                .with_speed(Speed::SlowCmosEdge)
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
            .with_speed(Speed::SlowCmosEdge)
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
        regs: zynq7000::qspi::MmioQspi<'static>,
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
                .with_speed(Speed::SlowCmosEdge)
                .with_l3_sel(QSPI_MUX_CONFIG.l3_sel())
                .with_l2_sel(QSPI_MUX_CONFIG.l2_sel())
                .with_l1_sel(QSPI_MUX_CONFIG.l1_sel())
                .with_l0_sel(QSPI_MUX_CONFIG.l0_sel())
                .with_tri_enable(false)
                .build(),
        );
        Self::new_single_qspi(regs, clock_config, mode, voltage, cs, io, clock)
    }

    #[inline]
    pub fn regs(&mut self) -> &mut zynq7000::qspi::MmioQspi<'static> {
        &mut self.ll.0
    }

    pub fn into_linear_addressed(mut self, config: LinearQspiConfig) -> QspiLinearAddressing {
        self.ll.enable_linear_addressing(config);
        QspiLinearAddressing { ll: self.ll }
    }

    pub fn into_io_mode(mut self, dual_flash: bool) -> QspiIoMode {
        self.ll.enable_io_mode(dual_flash);
        QspiIoMode { ll: self.ll }
    }
}

pub struct QspiIoMode {
    ll: QspiLowLevel,
}

impl QspiIoMode {
    #[inline]
    pub fn regs(&mut self) -> &mut zynq7000::qspi::MmioQspi<'static> {
        &mut self.ll.0
    }

    pub fn transfer_guard(&mut self) -> QspiIoTransferGuard<'_> {
        QspiIoTransferGuard::new(self)
    }

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

    #[inline]
    pub fn read_rx_data(&mut self) -> u32 {
        self.regs().read_rx_data()
    }

    pub fn transfer_init(&mut self) {
        self.regs().modify_config(|mut val| {
            val.set_peripheral_chip_select(false);
            val
        });
        self.regs()
            .write_spi_enable(SpiEnable::builder().with_enable(true).build());
    }

    pub fn transfer_start(&mut self) {
        self.regs().modify_config(|mut val| {
            val.set_manual_start_command(true);
            val
        });
    }

    pub fn transfer_done(&mut self) {
        self.regs().modify_config(|mut val| {
            val.set_peripheral_chip_select(true);
            val
        });
        self.regs()
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
    }

    pub fn read_status(&mut self) -> InterruptStatus {
        self.regs().read_interrupt_status()
    }

    pub fn clear_rx_fifo(&mut self) {
        while self.read_status().rx_above_threshold() {
            self.read_rx_data();
        }
    }

    pub fn into_linear_addressed(mut self, config: LinearQspiConfig) -> QspiLinearAddressing {
        self.ll.enable_linear_addressing(config);
        QspiLinearAddressing { ll: self.ll }
    }
}

/// This guard structure takes care of commonly required operations before starting a transfer
/// and after finishing it.
pub struct QspiIoTransferGuard<'a>(&'a mut QspiIoMode);

impl<'a> QspiIoTransferGuard<'a> {
    pub fn new(qspi: &'a mut QspiIoMode) -> Self {
        qspi.clear_rx_fifo();
        qspi.transfer_init();
        Self(qspi)
    }
}

impl QspiIoTransferGuard<'_> {
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

pub struct QspiLinearAddressing {
    ll: QspiLowLevel,
}

impl QspiLinearAddressing {
    /// Memory-mapped QSPI base address.
    pub const BASE_ADDRESS: usize = QSPI_START_ADDRESS;

    pub fn into_io_mode(mut self, dual_flash: bool) -> QspiIoMode {
        self.ll.enable_io_mode(dual_flash);
        QspiIoMode { ll: self.ll }
    }

    pub fn read_guard(&mut self) -> QspiLinearReadGuard<'_> {
        QspiLinearReadGuard::new(self)
    }
}

pub struct QspiLinearReadGuard<'a>(&'a mut QspiLinearAddressing);

impl QspiLinearReadGuard<'_> {
    /// Memory-mapped QSPI base address.
    pub const BASE_ADDRESS: usize = QSPI_START_ADDRESS;

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
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl().write_lqspi(
                QspiResetControl::builder()
                    .with_qspi_ref_reset(true)
                    .with_cpu_1x_reset(true)
                    .build(),
            );
            // Keep it in reset for some cycles.
            for _ in 0..3 {
                cortex_ar::asm::nop();
            }
            regs.reset_ctrl().write_lqspi(QspiResetControl::DEFAULT);
        });
    }
}
