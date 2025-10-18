use arbitrary_int::{traits::Integer as _, u3, u4, u6, u12};
use embedded_sdmmc::sdcard::{
    CardType,
    argument::{self, OcrLower},
    csd::{Csd, InvalidCsdStructureFieldError},
    response::{self, R1, R6},
};
use zynq7000::{
    sdio::{
        BusWidth, CommandRegister, InterruptMask, InterruptStatus, PresentState, SDIO_BASE_ADDR_0,
        SDIO_BASE_ADDR_1, SdClockDivisor,
    },
    slcr::{clocks::SrcSelIo, reset::DualRefAndClockResetSdio},
};

use crate::{
    clocks::IoClocks,
    enable_amba_peripheral_clock,
    gpio::{
        IoPeriphPin,
        mio::{MioPin, MuxConfig},
    },
    slcr::Slcr,
    time::Hertz,
};

pub use zynq7000::slcr::mio::IoType;

pub mod commands;
pub mod pins;

pub const MUX_CONF: MuxConfig = MuxConfig::new_with_l3(u3::new(0b100));
pub const BLOCK_LEN: usize = 512;

/// Negotiated as part of ACMD41 during SD card initialization.
pub const VOLTAGE_LEVEL_CAPABILITIES: OcrLower = OcrLower::builder()
    .with__3_5_to_3_6v(false)
    .with__3_4_to_3_5v(false)
    .with__3_3_to_3_4v(false)
    .with__3_2_to_3_3v(true)
    .with__3_1_to_3_2v(false)
    .with__3_0_to_3_1v(false)
    .with__2_9_to_3_0v(false)
    .with__2_8_to_2_9v(false)
    .with__2_7_to_2_8v(false)
    .with_reserved_low_voltage(false)
    .build();

/// Information retrieved from the SD card during the card initialization process.
#[derive(Debug, Clone)]
pub struct SdCardInfo {
    card_type: CardType,
    rca: u16,
    cid: embedded_sdmmc::sdcard::cid::Cid,
    csd: embedded_sdmmc::sdcard::csd::Csd,
}

impl SdCardInfo {
    #[inline]
    pub fn card_type(&self) -> CardType {
        self.card_type
    }

    /// Relative card address (RCA).
    #[inline]
    pub fn rca(&self) -> u16 {
        self.rca
    }

    #[inline]
    pub fn csd(&self) -> &embedded_sdmmc::sdcard::csd::Csd {
        &self.csd
    }

    #[inline]
    pub fn cid(&self) -> &embedded_sdmmc::sdcard::cid::Cid {
        &self.cid
    }
}

#[derive(Debug, thiserror::Error)]
#[error("invalid peripheral instance")]
pub struct InvalidPeripheralError;

#[derive(Debug, Clone, Copy, thiserror::Error)]
pub enum MciError {
    #[error("response CRC")]
    ResponseCrc,
    #[error("response end bit")]
    ResponseEndBit,
    #[error("response timeout")]
    ResponseTimeout,
    #[error("data CRC")]
    DataCrc,
    #[error("data timeout")]
    DataTimeout,
    #[error("data end bit")]
    DataEndBit,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ResponseErrorBits {
    index: bool,
    crc: bool,
    end_bit: bool,
    timeout: bool,
}

impl ResponseErrorBits {
    #[inline]
    pub const fn has_error(&self) -> bool {
        self.index || self.crc || self.end_bit || self.timeout
    }

    #[inline]
    pub const fn index(&self) -> bool {
        self.index
    }

    #[inline]
    pub const fn crc(&self) -> bool {
        self.crc
    }

    #[inline]
    pub const fn end_bit(&self) -> bool {
        self.end_bit
    }

    #[inline]
    pub const fn response_timeout(&self) -> bool {
        self.timeout
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DataErrorBits {
    crc: bool,
    timeout: bool,
    end_bit: bool,
}

impl DataErrorBits {
    #[inline]
    pub const fn crc(&self) -> bool {
        self.crc
    }

    #[inline]
    pub const fn timeout(&self) -> bool {
        self.timeout
    }
}

#[derive(Debug, Copy, Clone)]
pub struct StatusWrapper(pub zynq7000::sdio::InterruptStatus);

impl StatusWrapper {
    #[inline]
    pub fn response_errors(&self) -> ResponseErrorBits {
        ResponseErrorBits {
            index: self.0.command_index_error(),
            crc: self.0.command_crc_error(),
            end_bit: self.0.command_end_bit_error(),
            timeout: self.0.command_timeout_error(),
        }
    }

    #[inline]
    pub fn has_response_errors(&self) -> bool {
        self.0.command_index_error()
            || self.0.command_crc_error()
            || self.0.command_end_bit_error()
            || self.0.command_timeout_error()
    }

    #[inline]
    pub fn has_data_error(&self) -> bool {
        self.0.data_timeout_error() || self.0.data_crc_error() || self.0.data_end_bit_error()
    }

    #[inline]
    pub fn data_errors(&self) -> DataErrorBits {
        DataErrorBits {
            crc: self.0.data_crc_error(),
            timeout: self.0.data_timeout_error(),
            end_bit: self.0.data_end_bit_error(),
        }
    }
}

#[derive(Debug, Copy, Clone, thiserror::Error)]
pub enum InitializationError {
    #[error("response error")]
    ResponseError(ResponseErrorBits),
    #[error("unexpected response")]
    UnexpectedResponse,
    #[error("unexpected SD state")]
    UnexpectedState,
    #[error("invalid CSD: {0}")]
    Csd(#[from] InvalidCsdStructureFieldError),
    #[error("invalid CID")]
    Cid(#[from] embedded_sdmmc::sdcard::cid::ChecksumInvalidError),
}

#[derive(Debug, Copy, Clone)]
pub struct InitializationErrorWithStep {
    step: InitStep,
    error: InitializationError,
}

impl InitializationErrorWithStep {
    #[inline]
    pub fn error(&self) -> InitializationError {
        self.error
    }

    #[inline]
    pub fn step(&self) -> InitStep {
        self.step
    }
}

#[derive(Debug, Copy, Clone)]
pub enum InitStep {
    /// CMD0.
    IdleCommand,
    SendingIfCondCmd8,
    SendingIfCondAcmd41,
    RequestCid,
    RequestCsd,
    /// CMD3.
    RequestRca,
    PutIntoTransferState,
    /// ACMD6.
    SetBusWidth,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SdioId {
    _0,
    _1,
}

impl SdioId {
    /// Steal the ethernet register block for the given ethernet ID.
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees of the HAL.
    pub const unsafe fn steal_regs(&self) -> zynq7000::sdio::MmioRegisters<'static> {
        unsafe {
            match self {
                SdioId::_0 => zynq7000::sdio::Registers::new_mmio_fixed_0(),
                SdioId::_1 => zynq7000::sdio::Registers::new_mmio_fixed_1(),
            }
        }
    }
}

pub trait SdioRegisters {
    fn reg_block(&self) -> zynq7000::sdio::MmioRegisters<'static>;
    fn id(&self) -> Option<SdioId>;
}

impl SdioRegisters for zynq7000::sdio::MmioRegisters<'static> {
    #[inline]
    fn reg_block(&self) -> zynq7000::sdio::MmioRegisters<'static> {
        unsafe { self.clone() }
    }

    #[inline]
    fn id(&self) -> Option<SdioId> {
        let base_addr = unsafe { self.ptr() } as usize;
        if base_addr == SDIO_BASE_ADDR_0 {
            return Some(SdioId::_0);
        } else if base_addr == SDIO_BASE_ADDR_1 {
            return Some(SdioId::_1);
        }
        None
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SdioDivisors {
    /// Divisor which will be used during the initialization phase when ACMD41 is issued.
    ///
    /// The SD card specification mentions that the clock needs to be between 100 and 400 kHz for
    /// that phase.
    pub divisor_init_phase: SdClockDivisor,
    /// Divisor for the regular data transfer phase. Common target speeds are 25 MHz or 50 MHz.
    pub divisor_normal: SdClockDivisor,
}

impl SdioDivisors {
    // Calculate the SDIO clock divisors for the given SDIO reference clock and target speed.
    pub fn calculate(ref_clk: Hertz, target_speed: Hertz) -> Self {
        const INIT_CLOCK_HZ: u32 = 400_000;
        let divisor_select_from_value = |value: u32| match value {
            0..=1 => SdClockDivisor::Div1,
            2 => SdClockDivisor::Div2,
            3..=4 => SdClockDivisor::Div4,
            5..=8 => SdClockDivisor::Div8,
            9..=16 => SdClockDivisor::Div16,
            17..=32 => SdClockDivisor::Div32,
            33..=64 => SdClockDivisor::Div64,
            65..=128 => SdClockDivisor::Div128,
            129.. => SdClockDivisor::Div256,
        };
        Self {
            divisor_init_phase: divisor_select_from_value(ref_clk.raw().div_ceil(INIT_CLOCK_HZ)),
            divisor_normal: divisor_select_from_value(ref_clk.raw().div_ceil(target_speed.raw())),
        }
    }

    /// Calculate divisors for a regular clock configuration which configures the IO clock as
    /// source.
    pub fn calculate_for_io_clock(io_clocks: &IoClocks, target_speed: Hertz) -> Self {
        Self::calculate(io_clocks.sdio_clk(), target_speed)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SdClockConfig {
    /// Selects the source clock for the SDIO peripheral reference clock.
    pub src_sel: SrcSelIo,
    /// Selects the divisor which divies the source clock to create the SDIO peripheral
    /// reference clock.
    pub ref_clock_divisor: u6,
    /// The SDIO peripheral reference clock is divided again to create the SDIO clock.
    pub sdio_clock_divisors: SdioDivisors,
    /// High speed clock configuration, SD speed higher than 25 MHz or MMC speed higher than 20 MHz.
    pub high_speed: bool,
}

impl SdClockConfig {
    pub fn new(
        src_sel: SrcSelIo,
        ref_clock_divisor: u6,
        sdio_clock_divisors: SdioDivisors,
        high_speed: bool,
    ) -> Self {
        Self {
            src_sel,
            ref_clock_divisor,
            sdio_clock_divisors,
            high_speed,
        }
    }

    pub fn calculate_for_io_clock(
        io_clocks: &IoClocks,
        target_ref_clock: Hertz,
        target_sd_speed: Hertz,
    ) -> Option<Self> {
        let ref_clk = io_clocks.ref_clk();
        let io_ref_clock_divisor = ref_clk.raw().div_ceil(target_ref_clock.raw());
        if io_ref_clock_divisor > u6::MAX.as_u32() {
            return None;
        }
        let target_speed = ref_clk / io_ref_clock_divisor;

        let sdio_clock_divisors = SdioDivisors::calculate(target_speed, target_sd_speed);
        Some(Self {
            src_sel: SrcSelIo::IoPll,
            ref_clock_divisor: u6::new(io_ref_clock_divisor as u8),
            sdio_clock_divisors,
            high_speed: target_sd_speed > Hertz::MHz(25),
        })
    }
}
/// SD low-level interface.
///
/// Basic building block for higher-level abstractions.
pub struct SdLowLevel {
    id: SdioId,
    /// Register block. Direct public access is allowed to allow low-level operations.
    pub regs: zynq7000::sdio::MmioRegisters<'static>,
}

impl SdLowLevel {
    /// Create a new SDIO low-level interface from the given register block.
    ///
    /// Returns [None] if the given registers block base address does not correspond to a valid
    /// Ethernet peripheral.
    pub fn new(regs: zynq7000::sdio::MmioRegisters<'static>) -> Option<Self> {
        let id = regs.id()?;
        Some(Self { id, regs })
    }

    /// # Safety
    ///
    /// Allows getting multiple handles to the same SDIO peripheral.
    pub unsafe fn clone(&self) -> Self {
        Self {
            id: self.id,
            regs: unsafe { self.regs.clone() },
        }
    }

    #[inline]
    pub fn disable(&mut self, with_reset: bool, internal_clock_disable: bool) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sd_clock_enable(false);
            val
        });
        if with_reset {
            self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
                val.set_software_reset_for_all(true);
                val.set_internal_clock_enable(!internal_clock_disable);
                val
            });
            while self
                .regs
                .read_clock_timeout_sw_reset_control()
                .software_reset_for_all()
            {}
        }
    }

    #[inline]
    pub fn capabilities(&self) -> zynq7000::sdio::Capabilities {
        self.regs.read_capabilities()
    }

    #[inline]
    pub fn read_status(&self) -> StatusWrapper {
        StatusWrapper(self.regs.read_interrupt_status())
    }

    #[inline]
    pub fn read_data_word(&self) -> u32 {
        self.regs.read_buffer_data_port()
    }

    #[inline]
    pub fn write_data_word(&mut self, word: u32) {
        self.regs.write_buffer_data_port(word)
    }

    /// Common SDIO clock configuration routine which should be called once before using the SDIO.
    ///
    /// This does NOT disable the clock, which should be done before changing the clock
    /// configuration. It also does NOT enable the clock.
    ///
    /// It will configure the SDIO peripheral clock as well as initializing the SD clock frequency
    /// divisor based on the initial phase divider specified in the [SdioDivisors] field of the
    /// configuration.
    pub fn configure_clock(&mut self, clock_config: &SdClockConfig) {
        unsafe {
            Slcr::with(|slcr| {
                slcr.clk_ctrl().modify_sdio_clk_ctrl(|mut val| {
                    val.set_srcsel(clock_config.src_sel);
                    val.set_divisor(clock_config.ref_clock_divisor);
                    if self.id == SdioId::_1 {
                        val.set_clk_1_act(true);
                    } else {
                        val.set_clk_0_act(true);
                    }
                    val
                });
            });
        }
        self.configure_sd_clock_div_init_phase(&clock_config.sdio_clock_divisors);
    }

    pub fn initialize(&mut self, clock_config: &SdClockConfig) {
        if self.id == SdioId::_0 {
            enable_amba_peripheral_clock(crate::PeriphSelect::Sdio0);
        } else {
            enable_amba_peripheral_clock(crate::PeriphSelect::Sdio1);
        }
        self.reset(5);

        // Full software reset.
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_software_reset_for_all(true);
            val
        });
        while self
            .regs
            .read_clock_timeout_sw_reset_control()
            .software_reset_for_all()
        {}

        // Configure the clock which also configures the initial 400 KHz clock.
        self.configure_clock(clock_config);

        // Power off.
        self.regs
            .modify_host_power_blockgap_wakeup_control(|mut val| {
                val.set_sd_bus_voltage_select(zynq7000::sdio::SdBusVoltageSelect::Off);
                val
            });

        // Explicitely clear the clock bit.
        self.disable_sd_clock();

        // Set the only voltage configuration that is supported.
        self.regs
            .modify_host_power_blockgap_wakeup_control(|mut val| {
                val.set_sd_bus_voltage_select(zynq7000::sdio::SdBusVoltageSelect::_3_3V);
                val.set_sd_bus_power(true);
                val
            });

        // As specified in the TRM, wait until the internal clock is stable before enabling the
        // SD clock.
        self.enable_internal_clock();
        while !self
            .regs
            .read_clock_timeout_sw_reset_control()
            .internal_clock_stable()
        {}
        self.enable_sd_clock();

        // This is the value configured by the AMD driver.
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_data_timeout_counter_value(u4::new(0xE));
            val
        });
        // Set standard block size 512 bytes.
        self.regs.modify_block_params(|mut val| {
            val.set_transfer_block_size(u12::new(512));
            val
        });

        // Enable status bits without card interrupt bit.
        self.regs.write_interrupt_status_enable(
            InterruptMask::new_with_raw_value(0xFFFF_FFFF).with_card_interrupt(false),
        );
        // Disable all interrupts signals.
        self.regs.write_interrupt_signal_enable(InterruptMask::ZERO);

        // Clear status bits.
        self.clear_all_status_bits();
    }

    #[inline]
    pub fn read_u32_response(&self) -> u32 {
        // Index valid, unwrap okay.
        self.regs.read_responses(0).unwrap()
    }

    #[inline]
    pub fn read_u128_response(&self, buf: &mut [u32; 4]) -> [u8; 16] {
        for (index, word) in buf.iter_mut().rev().enumerate() {
            *word = self.regs.read_responses(index).unwrap().to_be()
        }
        let bytes: &mut [u8; 16] = bytemuck::cast_mut(buf);
        // For some reason the hardware withholds the last byte and we have to do some copying
        // to ensure the correct layout.
        bytes.copy_within(1.., 0);
        // The checksum is withheld from us. I have no idea why. Maybe the hardware verifies it
        // for us? Did not find docs on this.
        bytes[15] = 0;
        *bytes
    }

    /// Configure the SD clock divisor for the initialization phase (400 kHz target clock).
    pub fn configure_sd_clock_div_init_phase(&mut self, divs: &SdioDivisors) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sdclk_frequency_select(divs.divisor_init_phase);
            val
        });
    }

    /// Configure the SD clock divisor for the normal phase (regular SDIO speed clock).
    pub fn configure_sd_clock_div_normal_phase(&mut self, divs: &SdioDivisors) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sdclk_frequency_select(divs.divisor_normal);
            val
        });
    }

    #[inline]
    pub fn enable_internal_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_internal_clock_enable(true);
            val
        });
    }

    #[inline]
    pub fn enable_sd_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sd_clock_enable(true);
            val
        });
    }

    #[inline]
    pub fn disable_sd_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sd_clock_enable(false);
            val
        });
    }

    /// Send a command with an argument and wait for command completion. Returns the status
    /// register which can be used to check for errors.
    ///
    /// The response can be read from the response registers, depending on the sent command
    /// and the error flags.
    pub fn send_command(&mut self, cmd: CommandRegister, arg: u32) -> StatusWrapper {
        self.write_command(cmd, arg);
        loop {
            let status = self.regs.read_interrupt_status();
            if status.command_complete() || status.error_interrupt() {
                self.regs.write_interrupt_status(status);
                return StatusWrapper(status);
            }
        }
    }

    #[inline]
    pub fn read_present_state(&self) -> PresentState {
        self.regs.read_present_state()
    }

    #[inline]
    pub fn write_command(&mut self, command: CommandRegister, arg: u32) {
        self.clear_all_status_bits();
        self.regs.write_argument(arg);
        self.regs.write_command(command)
    }

    #[inline]
    pub fn clear_all_status_bits(&mut self) {
        self.regs
            .write_interrupt_status(InterruptStatus::new_with_raw_value(
                InterruptStatus::ALL_BITS,
            ));
    }

    #[inline]
    pub fn clear_status_bits(&mut self, status: InterruptStatus) {
        self.regs.write_interrupt_status(status);
    }

    /// Reset the SDIO peripheral using the SLCR reset register for SDIO.
    pub fn reset(&mut self, cycles: u32) {
        reset(self.id, cycles);
    }
}

/// SD card which has not been initialized yet.
///
/// It configures the physical SD pins correctly and exposes an [Self::initialize]r method
/// which converts itself into a [SdCard].
pub struct SdCardUninit {
    ll: SdLowLevel,
    clock_config: SdClockConfig,
}

impl SdCardUninit {
    pub fn new_for_sdio_0<
        Sdio0Clock: pins::Sdio0ClockPin,
        Sdio0Command: pins::Sdio0CommandPin,
        Sdio0Data0: pins::Sdio0Data0Pin,
        Sdio0Data1: pins::Sdio0Data1Pin,
        Sdio0Data2: pins::Sdio0Data2Pin,
        Sdio0Data3: pins::Sdio0Data3Pin,
    >(
        regs: zynq7000::sdio::MmioRegisters<'static>,
        clock_config: SdClockConfig,
        voltage_setting: IoType,
        clock_pin: Sdio0Clock,
        command_pin: Sdio0Command,
        data_pins: (Sdio0Data0, Sdio0Data1, Sdio0Data2, Sdio0Data3),
    ) -> Result<Self, InvalidPeripheralError> {
        let id = regs.id().ok_or(InvalidPeripheralError)?;
        if id != SdioId::_0 {
            return Err(InvalidPeripheralError);
        }
        Ok(Self::new(
            regs,
            clock_config,
            voltage_setting,
            clock_pin,
            command_pin,
            data_pins,
        ))
    }

    pub fn new_for_sdio_1<
        Sdio1Clock: pins::Sdio1ClockPin,
        Sdio1Command: pins::Sdio1CommandPin,
        Sdio1Data0: pins::Sdio1Data0Pin,
        Sdio1Data1: pins::Sdio1Data1Pin,
        Sdio1Data2: pins::Sdio1Data2Pin,
        Sdio1Data3: pins::Sdio1Data3Pin,
    >(
        regs: zynq7000::sdio::MmioRegisters<'static>,
        clock_config: SdClockConfig,
        voltage_setting: IoType,
        clock_pin: Sdio1Clock,
        command_pin: Sdio1Command,
        data_pins: (Sdio1Data0, Sdio1Data1, Sdio1Data2, Sdio1Data3),
    ) -> Result<Self, InvalidPeripheralError> {
        let id = regs.id().ok_or(InvalidPeripheralError)?;
        if id != SdioId::_1 {
            return Err(InvalidPeripheralError);
        }
        Ok(Self::new(
            regs,
            clock_config,
            voltage_setting,
            clock_pin,
            command_pin,
            data_pins,
        ))
    }

    fn new(
        regs: zynq7000::sdio::MmioRegisters<'static>,
        clock_config: SdClockConfig,
        voltage_setting: IoType,
        clock_pin: impl MioPin,
        command_pin: impl MioPin,
        data_pins: (impl MioPin, impl MioPin, impl MioPin, impl MioPin),
    ) -> Self {
        let ll = SdLowLevel::new(regs).unwrap();
        // Configuration taken from ps7_init.html.
        unsafe {
            crate::slcr::Slcr::with(|slcr| {
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    clock_pin,
                    slcr,
                    zynq7000::slcr::mio::Config::builder()
                        .with_disable_hstl_rcvr(false)
                        .with_pullup(false)
                        .with_io_type(voltage_setting)
                        .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
                        .with_l3_sel(MUX_CONF.l3_sel())
                        .with_l2_sel(MUX_CONF.l2_sel())
                        .with_l1_sel(MUX_CONF.l1_sel())
                        .with_l0_sel(MUX_CONF.l0_sel())
                        .with_tri_enable(false)
                        .build(),
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    command_pin,
                    slcr,
                    zynq7000::slcr::mio::Config::builder()
                        .with_disable_hstl_rcvr(false)
                        .with_pullup(false)
                        .with_io_type(voltage_setting)
                        .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
                        .with_l3_sel(MUX_CONF.l3_sel())
                        .with_l2_sel(MUX_CONF.l2_sel())
                        .with_l1_sel(MUX_CONF.l1_sel())
                        .with_l0_sel(MUX_CONF.l0_sel())
                        .with_tri_enable(false)
                        .build(),
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    data_pins.0,
                    slcr,
                    zynq7000::slcr::mio::Config::builder()
                        .with_disable_hstl_rcvr(false)
                        .with_pullup(false)
                        .with_io_type(voltage_setting)
                        .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
                        .with_l3_sel(MUX_CONF.l3_sel())
                        .with_l2_sel(MUX_CONF.l2_sel())
                        .with_l1_sel(MUX_CONF.l1_sel())
                        .with_l0_sel(MUX_CONF.l0_sel())
                        .with_tri_enable(false)
                        .build(),
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    data_pins.1,
                    slcr,
                    zynq7000::slcr::mio::Config::builder()
                        .with_disable_hstl_rcvr(false)
                        .with_pullup(false)
                        .with_io_type(voltage_setting)
                        .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
                        .with_l3_sel(MUX_CONF.l3_sel())
                        .with_l2_sel(MUX_CONF.l2_sel())
                        .with_l1_sel(MUX_CONF.l1_sel())
                        .with_l0_sel(MUX_CONF.l0_sel())
                        .with_tri_enable(false)
                        .build(),
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    data_pins.2,
                    slcr,
                    zynq7000::slcr::mio::Config::builder()
                        .with_disable_hstl_rcvr(false)
                        .with_pullup(false)
                        .with_io_type(voltage_setting)
                        .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
                        .with_l3_sel(MUX_CONF.l3_sel())
                        .with_l2_sel(MUX_CONF.l2_sel())
                        .with_l1_sel(MUX_CONF.l1_sel())
                        .with_l0_sel(MUX_CONF.l0_sel())
                        .with_tri_enable(false)
                        .build(),
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    data_pins.3,
                    slcr,
                    zynq7000::slcr::mio::Config::builder()
                        .with_disable_hstl_rcvr(false)
                        .with_pullup(false)
                        .with_io_type(voltage_setting)
                        .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
                        .with_l3_sel(MUX_CONF.l3_sel())
                        .with_l2_sel(MUX_CONF.l2_sel())
                        .with_l1_sel(MUX_CONF.l1_sel())
                        .with_l0_sel(MUX_CONF.l0_sel())
                        .with_tri_enable(false)
                        .build(),
                );
            });
        }
        Self { ll, clock_config }
    }

    /// Direct access to the low-level SDIO driver.
    pub fn ll_mut(&mut self) -> &mut SdLowLevel {
        &mut self.ll
    }

    pub fn ll(&self) -> &SdLowLevel {
        &self.ll
    }

    /// Performs the initialization sequence and converts itself into a [SdCard].
    pub fn initialize(mut self) -> Result<SdCard, InitializationErrorWithStep> {
        let sd_info = initialize_card(&mut self.ll, &self.clock_config)?;
        Ok(SdCard {
            ll: core::cell::RefCell::new(self.ll),
            clock_config: self.clock_config,
            sd_info,
        })
    }

    #[inline]
    pub fn send_command(&mut self, command: CommandRegister, argument: u32) -> StatusWrapper {
        self.ll.send_command(command, argument)
    }

    #[inline]
    pub fn read_u32_response(&self) -> u32 {
        self.ll.read_u32_response()
    }

    #[inline]
    pub fn regs(&mut self) -> &mut zynq7000::sdio::MmioRegisters<'static> {
        &mut self.ll.regs
    }
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum DeviceError {
    #[error("response error")]
    ResponseError(ResponseErrorBits),
    #[error("data error")]
    DataError(DataErrorBits),
    #[error("unexpected card state: {0:?}")]
    UnexpectedState(response::State),
    #[error("unexpected response")]
    UnexpectedResponse,
    #[error("invalid block size, larger than 512, or not multiple of 512")]
    InvalidBlockSize,
    #[error("too many blocks for a single multi-block transfer")]
    TooManyBlocks,
}

pub struct SdCard {
    ll: core::cell::RefCell<SdLowLevel>,
    clock_config: SdClockConfig,
    sd_info: SdCardInfo,
}

impl SdCard {
    pub fn reinitialize(&mut self) -> Result<(), InitializationErrorWithStep> {
        let mut ll = self.ll.borrow_mut();
        ll.disable(true, false);
        ll.initialize(&self.clock_config);
        drop(ll);

        let sd_info = initialize_card(self.ll.get_mut(), &self.clock_config)?;
        self.sd_info = sd_info;
        Ok(())
    }

    /// # Safety
    ///
    /// Allows to create multiple handles to the same SD card.
    pub unsafe fn clone(&self) -> SdCard {
        unsafe {
            Self {
                ll: core::cell::RefCell::new(self.ll.borrow_mut().clone()),
                clock_config: self.clock_config,
                sd_info: self.sd_info.clone(),
            }
        }
    }

    #[inline]
    pub fn ll(&self) -> core::cell::RefMut<'_, SdLowLevel> {
        self.ll.borrow_mut()
    }

    pub fn send_command(&self, cmd: CommandRegister, arg: u32) -> StatusWrapper {
        self.ll.borrow_mut().send_command(cmd, arg)
    }

    #[inline]
    pub fn card_info(&self) -> &SdCardInfo {
        &self.sd_info
    }

    #[inline]
    pub fn rca(&self) -> u16 {
        self.sd_info.rca
    }

    #[inline]
    pub fn read_u32_response(&self) -> u32 {
        self.ll.borrow_mut().read_u32_response()
    }

    pub fn read_status(&self) -> Result<response::CardStatus, DeviceError> {
        let status = self.send_command(
            commands::CMD13_SEND_STATUS,
            argument::Cmd13::builder()
                .with_rca(self.sd_info.rca)
                .with_send_task_status(false)
                .build()
                .raw_value(),
        );
        if status.has_response_errors() {
            return Err(DeviceError::ResponseError(status.response_errors()));
        }
        Ok(R1::new_with_raw_value(self.read_u32_response()))
    }

    /// Card IDentification (CID).
    #[inline]
    pub fn cid(&self) -> &embedded_sdmmc::sdcard::cid::Cid {
        &self.sd_info.cid
    }

    /// Card specific data (CSD).
    #[inline]
    pub fn csd(&self) -> &embedded_sdmmc::sdcard::csd::Csd {
        &self.sd_info.csd
    }

    /// Card type.
    #[inline]
    pub fn card_type(&self) -> CardType {
        self.sd_info.card_type
    }

    pub fn verify_sd_card_transfer_state(
        &self,
        target_state: response::State,
    ) -> Result<(), DeviceError> {
        let card_status = self.read_status()?;
        match card_status.state() {
            Ok(state) => {
                if state != target_state {
                    return Err(DeviceError::UnexpectedState(state));
                }
            }
            Err(_) => return Err(DeviceError::UnexpectedResponse),
        }
        Ok(())
    }

    /// Read a single block from the SD card at the given address.
    pub fn read_single_block(&self, buffer: &mut [u8], addr: u32) -> Result<(), DeviceError> {
        pub enum State {
            Reading,
            WaitForCompletion,
        }

        let mut state = State::Reading;
        if buffer.len() > BLOCK_LEN {
            return Err(DeviceError::InvalidBlockSize);
        }
        self.verify_sd_card_transfer_state(response::State::Tran)?;
        let status = self.send_command(commands::CMD17_READ_SINGLE_BLOCK, addr);
        if status.has_response_errors() {
            return Err(DeviceError::ResponseError(status.response_errors()));
        }
        let mut ll = self.ll();
        let mut bytes_read = 0;
        loop {
            let status = ll.read_status();
            if status.has_data_error() {
                ll.clear_status_bits(status.0);
                return Err(DeviceError::DataError(status.data_errors()));
            }
            match state {
                State::Reading => {
                    if status.0.buffer_read_ready() {
                        ll.clear_status_bits(status.0);

                        while bytes_read < BLOCK_LEN {
                            let word = ll.read_data_word();
                            buffer[bytes_read..bytes_read + 4].copy_from_slice(&word.to_ne_bytes());
                            bytes_read += 4;
                        }
                        state = State::WaitForCompletion;
                    }
                }
                State::WaitForCompletion => {
                    if status.0.transfer_complete() {
                        ll.clear_status_bits(status.0);
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    /// Read multiple blocks from the SD card at the given start address.
    pub fn read_multiple_blocks(
        &self,
        buffer: &mut [u8],
        mut addr: u32,
    ) -> Result<(), DeviceError> {
        // TODO: Proper multi-read implementation.
        if !buffer.len().is_multiple_of(BLOCK_LEN) {
            return Err(DeviceError::InvalidBlockSize);
        }
        // TODO: Support by doing multiple multi-block transfers?
        if (buffer.len() / BLOCK_LEN) > u16::MAX as usize {
            return Err(DeviceError::TooManyBlocks);
        }
        for block in buffer.chunks_mut(BLOCK_LEN) {
            self.read_single_block(block, addr)?;
            addr = addr.saturating_add(BLOCK_LEN as u32);
        }
        Ok(())
    }

    /// Write a single block to the SD card at the given address.
    pub fn write_single_block(&self, buffer: &[u8], addr: u32) -> Result<(), DeviceError> {
        pub enum State {
            Writing,
            WaitForCompletion,
        }

        let mut state = State::Writing;
        if buffer.len() > BLOCK_LEN {
            return Err(DeviceError::InvalidBlockSize);
        }
        self.verify_sd_card_transfer_state(response::State::Tran)?;
        // Write command first.
        let status = self.send_command(commands::CMD24_WRITE_BLOCK, addr);
        if status.has_response_errors() {
            return Err(DeviceError::ResponseError(status.response_errors()));
        }
        let mut ll = self.ll();
        // Wait for the FIFO to be ready.
        loop {
            let status = ll.read_status();
            if status.has_data_error() {
                ll.clear_status_bits(status.0);
                return Err(DeviceError::DataError(status.data_errors()));
            }
            match state {
                State::Writing => {
                    if status.0.buffer_write_ready() {
                        ll.clear_status_bits(status.0);
                        let mut bytes_written = 0;
                        while bytes_written < BLOCK_LEN {
                            ll.write_data_word(u32::from_ne_bytes(
                                buffer[bytes_written..bytes_written + 4].try_into().unwrap(),
                            ));
                            bytes_written += 4;
                        }
                        state = State::WaitForCompletion;
                    }
                }
                State::WaitForCompletion => {
                    if status.0.transfer_complete() {
                        break;
                    }
                }
            }
        }

        // Wait until the SD card is done with programming.
        drop(ll);
        loop {
            let status = self.read_status()?;
            let state = status.state();
            if state.is_err() {
                return Err(DeviceError::UnexpectedResponse);
            }
            if let Ok(state) = state
                && state == response::State::Tran
            {
                return Ok(());
            }
        }
    }

    /// Write multiple blocks from the SD card at the given start address.
    pub fn write_multiple_blocks(&self, buffer: &[u8], mut addr: u32) -> Result<(), DeviceError> {
        // TODO: Proper multi-write implementation.
        if !buffer.len().is_multiple_of(BLOCK_LEN) {
            return Err(DeviceError::InvalidBlockSize);
        }
        // TODO: Support by doing multiple multi-block transfers?
        if (buffer.len() / BLOCK_LEN) > u16::MAX as usize {
            return Err(DeviceError::TooManyBlocks);
        }
        for block in buffer.chunks(BLOCK_LEN) {
            self.write_single_block(block, addr)?;
            addr = addr.saturating_add(BLOCK_LEN as u32);
        }
        Ok(())
    }
}

impl embedded_sdmmc::BlockDevice for SdCard {
    type Error = DeviceError;

    fn read(
        &self,
        blocks: &mut [embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        let addr = match self.sd_info.card_type {
            CardType::SD1 | CardType::SD2 => start_block_idx.0 * BLOCK_LEN as u32,
            CardType::SdhcSdxc => start_block_idx.0,
        };
        for block in blocks.iter_mut() {
            self.read_single_block(block.as_mut_slice(), addr)?;
        }
        Ok(())
    }

    fn write(
        &self,
        blocks: &[embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        let (mut addr, increment) = match self.sd_info.card_type {
            CardType::SD1 | CardType::SD2 => (start_block_idx.0 * BLOCK_LEN as u32, BLOCK_LEN),
            CardType::SdhcSdxc => (start_block_idx.0, 1),
        };
        for block in blocks.iter() {
            self.write_single_block(block.as_slice(), addr)?;
            addr = addr.saturating_add(increment as u32);
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
        Ok(embedded_sdmmc::BlockCount(
            self.csd().card_capacity_blocks(),
        ))
    }
}

/// Card initialization sequence.
///
/// This performs the steps specified in chapter 4 of the SD card specification.
/// It also retrieves the [SdCardInfo] structure from the SD card during this process.
pub fn initialize_card(
    ll: &mut SdLowLevel,
    clock_config: &SdClockConfig,
) -> Result<SdCardInfo, InitializationErrorWithStep> {
    const CMD8_RETRIES: usize = 2;

    ll.initialize(clock_config);

    let status = ll.send_command(commands::CMD0_GO_IDLE_MODE, 0);
    if status.has_response_errors() {
        return Err(InitializationErrorWithStep {
            step: InitStep::IdleCommand,
            error: InitializationError::ResponseError(status.response_errors()),
        });
    }

    // Voltage level negotiation.
    let mut index = 0;
    let mut status_cmd8 = ll.send_command(
        commands::CMD8_SEND_IF_COND,
        argument::Cmd8::ZERO
            .with_voltage_supplied(
                embedded_sdmmc::sdcard::argument::VoltageSuppliedSelect::_2_7To3_6V,
            )
            .with_check_pattern(0xAA)
            .raw_value(),
    );
    while status_cmd8.0.command_timeout_error() && index < CMD8_RETRIES {
        status_cmd8 = ll.send_command(
            commands::CMD8_SEND_IF_COND,
            argument::Cmd8::ZERO
                .with_voltage_supplied(
                    embedded_sdmmc::sdcard::argument::VoltageSuppliedSelect::_2_7To3_6V,
                )
                .with_check_pattern(0xAA)
                .raw_value(),
        );
        index += 1;
    }

    let responded_to_cmd8 = !status_cmd8.0.command_timeout_error();
    let acmd41_arg = if responded_to_cmd8 {
        let r7 = response::R7::new_with_raw_value(ll.read_u32_response());
        if !r7.voltage_accepted().is_ok_and(|val| {
            val == embedded_sdmmc::sdcard::argument::VoltageSuppliedSelect::_2_7To3_6V
        }) || r7.echo_check_pattern() != 0xAA
        {
            return Err(InitializationErrorWithStep {
                step: InitStep::SendingIfCondCmd8,
                error: InitializationError::ResponseError(status.response_errors()),
            });
        }
        argument::Acmd41::builder()
            .with_host_capacity_support(
                embedded_sdmmc::sdcard::argument::HostCapacitySupport::SdhcOrSdxc,
            )
            .with_fast_boot(false)
            .with_xpc(embedded_sdmmc::sdcard::argument::PowerControl::MaximumPerformance)
            .with_s18r(false)
            .with_ocr(VOLTAGE_LEVEL_CAPABILITIES)
            .build()
    } else {
        argument::Acmd41::builder()
            .with_host_capacity_support(
                embedded_sdmmc::sdcard::argument::HostCapacitySupport::SdscOnly,
            )
            .with_fast_boot(false)
            .with_xpc(embedded_sdmmc::sdcard::argument::PowerControl::MaximumPerformance)
            .with_s18r(false)
            .with_ocr(VOLTAGE_LEVEL_CAPABILITIES)
            .build()
    };
    send_acmd(ll, commands::ACMD41_SEND_IF_COND, acmd41_arg.raw_value(), 0).map_err(|e| {
        InitializationErrorWithStep {
            step: InitStep::SendingIfCondAcmd41,
            error: e,
        }
    })?;

    let mut r3 = embedded_sdmmc::sdcard::response::R3::new_with_raw_value(ll.read_u32_response());
    if !r3.initialization_complete() {
        r3 = wait_for_amd41_card_ready(ll, 0).map_err(|e| InitializationErrorWithStep {
            step: InitStep::SendingIfCondAcmd41,
            error: e,
        })?;
    };
    let card_type = if responded_to_cmd8 {
        if r3.card_capacity_status() {
            CardType::SdhcSdxc
        } else {
            CardType::SD2
        }
    } else {
        CardType::SD1
    };

    // Retrieve and cache the CID. This puts it into identification mode.
    let status = ll.send_command(commands::CMD2_ALL_SEND_CID, 0);
    if status.has_response_errors() {
        return Err(InitializationErrorWithStep {
            step: InitStep::RequestCid,
            error: InitializationError::ResponseError(status.response_errors()),
        });
    }
    let mut cid_buf: [u32; 4] = [0; 4];
    let cid_raw = ll.read_u128_response(&mut cid_buf);
    let cid = embedded_sdmmc::sdcard::cid::Cid::new_with_raw_value(u128::from_be_bytes(cid_raw));

    // Send CMD3 to retrieve RCA required for card addressing, as well as put the card
    // into standby mode.
    let status = ll.send_command(commands::CMD3_SEND_RELATIVE_ADDR, 0);
    if status.has_response_errors() {
        return Err(InitializationErrorWithStep {
            step: InitStep::RequestRca,
            error: InitializationError::ResponseError(status.response_errors()),
        });
    }
    let r6 = R6::new_with_raw_value(ll.read_u32_response());
    let rca = r6.rca();

    // Retrieve and cache CSD, which also contains card specific data.
    let status = ll.send_command(
        commands::CMD9_SEND_CSD,
        argument::Cmd9::builder()
            .with_rca(r6.rca())
            .build()
            .raw_value(),
    );
    if status.has_response_errors() {
        return Err(InitializationErrorWithStep {
            step: InitStep::RequestCsd,
            error: InitializationError::ResponseError(status.response_errors()),
        });
    }
    let mut csd_buf: [u32; 4] = [0; 4];
    let csd_raw = ll.read_u128_response(&mut csd_buf);
    let csd = Csd::new_unchecked(&csd_raw).map_err(|e| InitializationErrorWithStep {
        step: InitStep::RequestCsd,
        error: InitializationError::Csd(e),
    })?;

    // CMD7 to put the SD card into transfer state.
    let status = ll.send_command(
        commands::CMD7_SELECT_SD_CARD,
        argument::Cmd7::builder()
            .with_rca(r6.rca())
            .build()
            .raw_value(),
    );
    if status.has_response_errors() {
        return Err(InitializationErrorWithStep {
            step: InitStep::PutIntoTransferState,
            error: InitializationError::ResponseError(status.response_errors()),
        });
    }
    // Check that the card is in transfer mode.
    let status = ll.send_command(
        commands::CMD13_SEND_STATUS,
        argument::Cmd13::builder()
            .with_rca(r6.rca())
            .with_send_task_status(false)
            .build()
            .raw_value(),
    );
    if status.has_response_errors() {
        return Err(InitializationErrorWithStep {
            step: InitStep::PutIntoTransferState,
            error: InitializationError::ResponseError(status.response_errors()),
        });
    }
    let r1 = R1::new_with_raw_value(ll.read_u32_response());
    if r1.state().is_ok_and(|state| state != response::State::Tran) {
        return Err(InitializationErrorWithStep {
            step: InitStep::PutIntoTransferState,
            error: InitializationError::UnexpectedResponse,
        });
    }

    // Put the SD card into 4-bit mode.
    send_acmd(
        ll,
        commands::ACMD6_SET_BUS_WIDTH,
        argument::Acmd6::builder()
            .with_bus_width(argument::BusWidth::_4bits)
            .build()
            .raw_value(),
        rca,
    )
    .map_err(|e| InitializationErrorWithStep {
        step: InitStep::SetBusWidth,
        error: e,
    })?;
    ll.regs
        .modify_host_power_blockgap_wakeup_control(|mut val| {
            val.set_bus_width(BusWidth::_4bits);
            val.set_high_speed_enable(clock_config.high_speed);
            val
        });
    ll.configure_sd_clock_div_normal_phase(&clock_config.sdio_clock_divisors);

    Ok(SdCardInfo {
        card_type,
        rca,
        cid,
        csd,
    })
}

/// Send an ACMD.
pub fn send_acmd(
    ll: &mut SdLowLevel,
    acmd: CommandRegister,
    arg: u32,
    rca: u16,
) -> Result<StatusWrapper, InitializationError> {
    let status = ll.send_command(commands::CMD55_APP_CMD, (rca as u32) << 16);
    if status.has_response_errors() {
        return Err(InitializationError::ResponseError(status.response_errors()));
    }
    let r1 = embedded_sdmmc::sdcard::response::R1::new_with_raw_value(ll.read_u32_response());
    if !r1.app_cmd() {
        return Err(InitializationError::UnexpectedResponse);
    }
    Ok(ll.send_command(acmd, arg))
}

pub fn wait_for_amd41_card_ready(
    ll: &mut SdLowLevel,
    rca: u16,
) -> Result<embedded_sdmmc::sdcard::response::R3, InitializationError> {
    loop {
        let status = send_acmd(ll, commands::ACMD41_SEND_IF_COND, 0, rca)?;
        // Explicitely check only for timeouts here. CRC errors have occured here..
        if status.0.command_timeout_error() {
            return Err(InitializationError::ResponseError(status.response_errors()));
        }
        let r3 = embedded_sdmmc::sdcard::response::R3::new_with_raw_value(ll.read_u32_response());
        if r3.initialization_complete() {
            return Ok(r3);
        }
    }
}

/// Reset the SDIO peripheral using the SLCR reset register for SDIO.
///
/// Please note that this function will interfere with an already configured
/// SDIO instance.
#[inline]
pub fn reset(id: SdioId, cycles: u32) {
    let assert_reset = match id {
        SdioId::_0 => DualRefAndClockResetSdio::builder()
            .with_periph1_ref_rst(false)
            .with_periph0_ref_rst(true)
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        SdioId::_1 => DualRefAndClockResetSdio::builder()
            .with_periph1_ref_rst(true)
            .with_periph0_ref_rst(false)
            .with_periph1_cpu1x_rst(true)
            .with_periph0_cpu1x_rst(false)
            .build(),
    };
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl().write_sdio(assert_reset);
            // Keep it in reset for a few cycle.. not sure if this is necessary.
            for _ in 0..cycles {
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_sdio(DualRefAndClockResetSdio::ZERO);
        });
    }
}
