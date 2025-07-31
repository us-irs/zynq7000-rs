//! GPIO support module for the Zynq7000 SoC.
//!
//! This module contains a MIO and EMIO pin resource managements singleton as well as abstractions
//! to use these pins as GPIOs.
//!
//! # Examples
//!
//! - [Blinky](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/examples/simple/src/main.rs)
//! - [Logger example](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/examples/simple/src/bin/logger.rs)
//!   which uses MIO pins for the UART.
pub mod emio;
pub mod ll;
pub mod mio;

use core::convert::Infallible;
use ll::PinOffset;
use mio::{MioPinMarker, MuxCfg};

use crate::gpio::ll::LowLevelGpio;
use crate::{enable_amba_periph_clk, slcr::Slcr};
pub use embedded_hal::digital::PinState;
use zynq7000::{gpio::MmioGpio, slcr::reset::GpioClkRst};

#[derive(Debug, thiserror::Error)]
#[error("MIO pins 7 and 8 can only be output pins")]
pub struct PinIsOutputOnly;

/// GPIO pin singleton to allow resource management of both MIO and EMIO pins.
pub struct GpioPins {
    pub mio: mio::Pins,
    pub emio: emio::Pins,
}

impl GpioPins {
    pub fn new(gpio: MmioGpio) -> Self {
        enable_amba_periph_clk(crate::PeriphSelect::Gpio);
        Self {
            mio: mio::Pins::new(unsafe { gpio.clone() }),
            emio: emio::Pins::new(gpio),
        }
    }
}

/// Reset the GPIO peripheral using the SLCR reset register for GPIO.
#[inline]
pub fn reset() {
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl()
                .write_gpio(GpioClkRst::builder().with_gpio_cpu1x_rst(true).build());
            // Keep it in reset for one cycle.. not sure if this is necessary.
            cortex_ar::asm::nop();
            regs.reset_ctrl()
                .write_gpio(GpioClkRst::builder().with_gpio_cpu1x_rst(false).build());
        });
    }
}

/// Enumeration of all pin modes. Some of the modes are only valid for MIO pins.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PinMode {
    OutputPushPull,
    /// See [super::gpio] documentation for more information on running an output pin in
    /// open-drain configuration.
    OutputOpenDrain,
    InputFloating,
    InputPullUp,
    /// MIO-only peripheral pin configuration
    MioIoPeriph(MuxCfg),
}

#[derive(Debug, thiserror::Error)]
#[error("invalid pin mode for MIO pin: {0:?}")]
pub struct InvalidPinMode(pub PinMode);

impl embedded_hal::digital::Error for InvalidPinMode {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

pub trait IoPinProvider {
    fn mode(&self) -> PinMode;

    fn offset(&self) -> PinOffset;

    #[inline]
    fn is_input(&self) -> bool {
        matches!(self.mode(), PinMode::InputFloating | PinMode::InputPullUp)
    }
    #[inline]
    fn is_output(&self) -> bool {
        matches!(
            self.mode(),
            PinMode::OutputPushPull | PinMode::OutputOpenDrain
        )
    }

    #[inline]
    fn is_io_periph(&self) -> bool {
        matches!(self.mode(), PinMode::MioIoPeriph(_))
    }
}

/// Flex pin abstraction which can be dynamically re-configured.
///
/// The following functions can be configured at run-time:
///
///  - Input Floating
///  - Input with Pull-Up
///  - Output Push-Pull
///  - Output Open-Drain.
///
/// Flex pins are always floating input pins after construction except for MIO7 and MIO8,
/// which are Push-Pull Output pins with initial low-level.
///
/// ## Notes on [PinMode::OutputOpenDrain] configuration
///
/// For MIO, the open-drain functionality is simulated by only enabling the output driver
/// when driving the pin low, and leaving the pin floating when the pin is driven high.
/// The internal pull-up will also be enabled to have a high state if the pin is not driven.
///
/// For EMIO, the pull-up and the IO buffer needs to be provided in the FPGA design for the
/// used EMIO pins because the EMIO pins are just wires going out to the FPGA design.
/// The software will still perform the necessary logic when driving the pin low or high.
///
/// ## Notes on [PinMode::InputPullUp] configuration
///
/// For EMIO, the pull-up wiring needs to be provided by the FPGA design.
pub struct Flex {
    ll: LowLevelGpio,
    mode: PinMode,
}

impl Flex {
    pub fn new_for_mio<I: mio::PinId>(_pin: mio::Pin<I>) -> Self {
        let mut ll = LowLevelGpio::new(PinOffset::Mio(I::OFFSET));
        if I::OFFSET == 7 || I::OFFSET == 8 {
            ll.configure_as_output_push_pull(PinState::Low);
        } else {
            ll.configure_as_input_floating().unwrap();
        }
        Self {
            ll,
            mode: PinMode::InputFloating,
        }
    }

    pub fn new_for_emio(pin: emio::EmioPin) -> Self {
        let mut ll = LowLevelGpio::new(PinOffset::new_for_emio(pin.offset()).unwrap());
        ll.configure_as_input_floating().unwrap();
        Self {
            ll,
            mode: PinMode::InputFloating,
        }
    }

    pub fn configure_as_input_floating(&mut self) -> Result<(), PinIsOutputOnly> {
        self.mode = PinMode::InputFloating;
        self.ll.configure_as_input_floating()
    }

    pub fn configure_as_input_with_pull_up(&mut self) -> Result<(), PinIsOutputOnly> {
        self.mode = PinMode::InputPullUp;
        self.ll.configure_as_input_with_pull_up()
    }

    pub fn configure_as_output_push_pull(&mut self, level: PinState) {
        self.mode = PinMode::OutputPushPull;
        self.ll.configure_as_output_push_pull(level);
    }

    pub fn configure_as_output_open_drain(&mut self, level: PinState, with_internal_pullup: bool) {
        self.mode = PinMode::OutputOpenDrain;
        self.ll
            .configure_as_output_open_drain(level, with_internal_pullup);
    }

    /// If the pin is configured as an input pin, this function does nothing.
    pub fn set_high(&mut self) {
        if self.is_input() {
            return;
        }
        if self.mode == PinMode::OutputOpenDrain {
            self.ll.disable_output_driver();
        } else {
            self.ll.set_high();
        }
    }

    /// If the pin is configured as an input pin, this function does nothing.
    pub fn set_low(&mut self) {
        if self.is_input() {
            return;
        }
        self.ll.set_low();
        if self.mode == PinMode::OutputOpenDrain {
            self.ll.enable_output_driver();
        }
    }

    /// Reads the input state of the pin, regardless of configured mode.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.ll.is_high()
    }

    /// Reads the input state of the pin, regardless of configured mode.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.ll.is_high()
    }

    /// If the pin is not configured as a stateful output pin like Output Push-Pull, the result
    /// of this function is undefined.
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.ll.is_set_low()
    }

    /// If the pin is not configured as a stateful output pin like Output Push-Pull, the result
    /// of this function is undefined.
    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }
}

impl IoPinProvider for Flex {
    fn mode(&self) -> PinMode {
        self.mode
    }

    fn offset(&self) -> PinOffset {
        self.ll.offset()
    }
}

impl embedded_hal::digital::ErrorType for Flex {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Flex {
    /// Reads the input state of the pin, regardless of configured mode.
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_high())
    }

    /// Reads the input state of the pin, regardless of configured mode.
    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_low())
    }
}

impl embedded_hal::digital::OutputPin for Flex {
    /// If the pin is configured as an input pin, this function does nothing.
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }

    /// If the pin is configured as an input pin, this function does nothing.
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Flex {
    /// If the pin is not configured as a stateful output pin like Output Push-Pull, the result
    /// of this function is undefined.
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_high())
    }

    /// If the pin is not configured as a stateful output pin like Output Push-Pull, the result
    /// of this function is undefined.
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_low())
    }
}

/// Push-Pull output pin.
pub struct Output(LowLevelGpio);

impl Output {
    pub fn new_for_mio<I: mio::PinId>(_pin: mio::Pin<I>, init_level: PinState) -> Self {
        let mut low_level = LowLevelGpio::new(PinOffset::Mio(I::OFFSET));
        low_level.configure_as_output_push_pull(init_level);
        Self(low_level)
    }

    pub fn new_for_emio(pin: emio::EmioPin, init_level: PinState) -> Self {
        let mut low_level = LowLevelGpio::new(PinOffset::new_for_emio(pin.offset()).unwrap());
        low_level.configure_as_output_push_pull(init_level);
        Self(low_level)
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.0.set_low();
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.0.set_high();
    }
}

impl embedded_hal::digital::ErrorType for Output {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Output {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Output {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_set_low())
    }
}

/// Input pin.
pub struct Input(LowLevelGpio);

impl Input {
    pub fn new_for_mio<I: mio::PinId>(_pin: mio::Pin<I>) -> Result<Self, PinIsOutputOnly> {
        let mut low_level = LowLevelGpio::new(PinOffset::Mio(I::OFFSET));
        low_level.configure_as_input_floating()?;
        Ok(Self(low_level))
    }

    pub fn new_for_emio(pin: emio::EmioPin) -> Result<Self, PinIsOutputOnly> {
        let mut low_level = LowLevelGpio::new(PinOffset::new_for_emio(pin.offset()).unwrap());
        low_level.configure_as_input_floating()?;
        Ok(Self(low_level))
    }

    pub fn is_high(&self) -> bool {
        self.0.is_high()
    }

    pub fn is_low(&self) -> bool {
        self.0.is_low()
    }
}

impl embedded_hal::digital::ErrorType for Input {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_low())
    }
}

/// IO peripheral pin.
pub struct IoPeriphPin {
    pin: LowLevelGpio,
    mux_conf: MuxCfg,
}

impl IoPeriphPin {
    /// Constructor for IO peripheral pins where only the multiplexer and pullup configuration
    /// need to be changed.
    pub fn new(pin: impl MioPinMarker, mux_conf: MuxCfg, pullup: Option<bool>) -> Self {
        let mut low_level = LowLevelGpio::new(PinOffset::Mio(pin.offset()));
        low_level.configure_as_io_periph_pin(mux_conf, pullup);
        Self {
            pin: low_level,
            mux_conf,
        }
    }

    /// Constructor to fully configure an IO peripheral pin with a specific MIO pin configuration.
    pub fn new_with_full_config(
        pin: impl MioPinMarker,
        config: zynq7000::slcr::mio::Config,
    ) -> Self {
        let mut low_level = LowLevelGpio::new(PinOffset::Mio(pin.offset()));
        low_level.set_mio_pin_config(config);
        Self {
            pin: low_level,
            mux_conf: MuxCfg::new(
                config.l0_sel(),
                config.l1_sel(),
                config.l2_sel(),
                config.l3_sel(),
            ),
        }
    }

    /// Constructor to fully configure an IO peripheral pin with a specific MIO pin configuration.
    pub fn new_with_full_config_and_unlocked_slcr(
        pin: impl MioPinMarker,
        slcr: &mut zynq7000::slcr::MmioSlcr<'static>,
        config: zynq7000::slcr::mio::Config,
    ) -> Self {
        let mut low_level = LowLevelGpio::new(PinOffset::Mio(pin.offset()));
        low_level.set_mio_pin_config_with_unlocked_slcr(slcr, config);
        Self {
            pin: low_level,
            mux_conf: MuxCfg::new(
                config.l0_sel(),
                config.l1_sel(),
                config.l2_sel(),
                config.l3_sel(),
            ),
        }
    }
}

impl IoPinProvider for IoPeriphPin {
    #[inline]
    fn mode(&self) -> PinMode {
        PinMode::MioIoPeriph(self.mux_conf)
    }

    #[inline]
    fn offset(&self) -> PinOffset {
        self.pin.offset()
    }
}
