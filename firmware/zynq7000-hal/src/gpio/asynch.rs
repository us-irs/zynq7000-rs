//! # Asynchronous support module
use core::{convert::Infallible, future::poll_fn, sync::atomic::AtomicBool, task::Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::ll::{LowLevelGpio, PinOffset};

static EVENT_WAKERS_MIO_BANK_0: [AtomicWaker; 32] = [const { AtomicWaker::new() }; 32];
static EVENT_WAKERS_MIO_BANK_1: [AtomicWaker; 32] = [const { AtomicWaker::new() }; 32];
static EVENT_WAKERS_EMIO_BANK_0: [AtomicWaker; 32] = [const { AtomicWaker::new() }; 32];
static EVENT_WAKERS_EMIO_BANK_1: [AtomicWaker; 32] = [const { AtomicWaker::new() }; 32];

static EVENT_SIGNAL_MIO_BANK_0: [AtomicBool; 32] = [const { AtomicBool::new(false) }; 32];
static EVENT_SIGNAL_MIO_BANK_1: [AtomicBool; 32] = [const { AtomicBool::new(false) }; 32];
static EVENT_SIGNAL_EMIO_BANK_0: [AtomicBool; 32] = [const { AtomicBool::new(false) }; 32];
static EVENT_SIGNAL_EMIO_BANK_1: [AtomicBool; 32] = [const { AtomicBool::new(false) }; 32];

/// Interrupt handler for GPIO events.
///
/// This handler is automatically registered for [crate::SpiInterrupt::Gpio] when using
/// [super::Pins::new] and enabling the registration. If this is not done, users need
/// to register this function manually for asynchronous pin events to work properly.
pub fn on_interrupt() {
    // SAFETY: Only the interrupt status and interrupt disable registers are written. Both are
    // write-1 registers, so other pins are not affected.
    let mut regs = unsafe { zynq7000::gpio::Registers::new_mmio_fixed() };
    let banks = [
        read_and_clear_interrupt_status(&mut regs.bank_0()),
        read_and_clear_interrupt_status(&mut regs.bank_1()),
        read_and_clear_interrupt_status(&mut regs.bank_2()),
        read_and_clear_interrupt_status(&mut regs.bank_3()),
    ];
    let wakers = [
        &EVENT_WAKERS_MIO_BANK_0,
        &EVENT_WAKERS_MIO_BANK_1,
        &EVENT_WAKERS_EMIO_BANK_0,
        &EVENT_WAKERS_EMIO_BANK_1,
    ];
    let signal = [
        &EVENT_SIGNAL_MIO_BANK_0,
        &EVENT_SIGNAL_MIO_BANK_1,
        &EVENT_SIGNAL_EMIO_BANK_0,
        &EVENT_SIGNAL_EMIO_BANK_1,
    ];

    for ((mut interrupt_status, waker_list), signal) in banks.into_iter().zip(wakers).zip(signal) {
        let mut offset = 0_usize;
        while interrupt_status > 0 {
            let bit_set = (interrupt_status & 1) != 0;
            if bit_set {
                signal[offset].store(true, core::sync::atomic::Ordering::Release);
                waker_list[offset].wake();
            }
            interrupt_status >>= 1;
            offset += 1;
        }
    }
}

fn read_and_clear_interrupt_status(bank: &mut zynq7000::gpio::MmioBankControlRegisters<'_>) -> u32 {
    // The status is also latched for disabled interrupts. Those belong to users which poll the
    // status register themselves.
    let enabled = !bank.read_interrupt_mask();
    let interrupt_status = bank.read_interrupt_status() & enabled;
    // A level interrupt stays asserted as long as the level is present, so it would fire again
    // right after clearing it. Disable it instead.
    let level_sensitive = !bank.read_interrupt_type().raw_value();
    bank.write_disable_interrupt(interrupt_status & level_sensitive);
    bank.write_interrupt_status(interrupt_status);
    interrupt_status
}

/// Level sensitivity variants.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LevelSensitivity {
    /// Active low.
    ActiveLow,
    /// Active high.
    ActiveHigh,
}

impl LevelSensitivity {
    /// Interrupt polarity register value for this sensitivity.
    #[inline]
    pub const fn polarity_bit(&self) -> zynq7000::gpio::Polarity {
        match self {
            LevelSensitivity::ActiveLow => zynq7000::gpio::Polarity::ActiveLowOrFallingEdge,
            LevelSensitivity::ActiveHigh => zynq7000::gpio::Polarity::ActiveHighOrRisingEdge,
        }
    }
}

/// Edge sensitivity variants.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EdgeSensitivity {
    /// Rising edge.
    Rising,
    /// Falling edge.
    Falling,
    /// Rising and falling edge.
    Any,
}

impl EdgeSensitivity {
    /// Interrupt polarity register value for this sensitivity.
    #[inline]
    pub const fn polarity_bit(&self) -> zynq7000::gpio::Polarity {
        match self {
            EdgeSensitivity::Rising => zynq7000::gpio::Polarity::ActiveHighOrRisingEdge,
            EdgeSensitivity::Falling => zynq7000::gpio::Polarity::ActiveLowOrFallingEdge,
            // The polarity is ignored if `INT_ANY` is set.
            EdgeSensitivity::Any => zynq7000::gpio::Polarity::ActiveLowOrFallingEdge,
        }
    }
}

/// Interrupt sensitivity.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sensitivity {
    /// Level sensitive.
    Level(LevelSensitivity),
    /// Edge sensitive.
    Edge(EdgeSensitivity),
}

/// Disables the pin interrupt when a wait completes or its future is dropped early.
struct DisableInterruptOnDrop(PinOffset);

impl Drop for DisableInterruptOnDrop {
    fn drop(&mut self) {
        LowLevelGpio::new(self.0).disable_interrupt();
    }
}

/// Asynchronous input pin driver.
///
/// This is a thin wrapper around a regular input pin which also provides an asynchronous event API.
/// It also implements the [embedded_hal_async::digital::Wait] trait.
#[derive(Debug)]
pub struct Input {
    /// Underlying input pin.
    pub input: super::Input,
}

impl From<super::Input> for Input {
    fn from(value: super::Input) -> Self {
        Self { input: value }
    }
}

impl Input {
    /// Constructor.
    pub fn new(input: super::Input) -> Self {
        Self { input }
    }

    /// Input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.input.is_high()
    }

    /// Input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.input.is_low()
    }

    /// Wait until the pin is high. Returns immediately if it already is.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        if self.is_high() {
            return;
        }
        self.wait_for_event(Sensitivity::Level(LevelSensitivity::ActiveHigh))
            .await
    }

    /// Wait until the pin is low. Returns immediately if it already is.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        if self.is_low() {
            return;
        }
        self.wait_for_event(Sensitivity::Level(LevelSensitivity::ActiveLow))
            .await
    }

    /// Wait for a rising edge.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.wait_for_event(Sensitivity::Edge(EdgeSensitivity::Rising))
            .await
    }

    /// Wait for a falling edge.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.wait_for_event(Sensitivity::Edge(EdgeSensitivity::Falling))
            .await
    }

    /// Wait for a rising or falling edge.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.wait_for_event(Sensitivity::Edge(EdgeSensitivity::Any))
            .await
    }

    async fn wait_for_event(&mut self, sensitivity: Sensitivity) {
        let _disable_on_drop = DisableInterruptOnDrop(self.input.0.offset());
        let (waker, signal) = self.waker_and_signal();
        // Reset before enabling, otherwise an event right after enabling would be erased.
        signal.store(false, core::sync::atomic::Ordering::Relaxed);
        self.input.0.enable_interrupt(sensitivity);
        poll_fn(|cx| {
            waker.register(cx.waker());
            if signal.swap(false, core::sync::atomic::Ordering::Acquire) {
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;
    }

    fn waker_and_signal(&self) -> (&'static AtomicWaker, &'static AtomicBool) {
        let offset = self.input.0.offset();
        let raw_offset = offset.offset();
        match offset.is_mio() {
            true => match raw_offset {
                (0..=31) => (
                    &EVENT_WAKERS_MIO_BANK_0[raw_offset],
                    &EVENT_SIGNAL_MIO_BANK_0[raw_offset],
                ),
                _ => {
                    let raw_offset = raw_offset - 32;
                    (
                        &EVENT_WAKERS_MIO_BANK_1[raw_offset],
                        &EVENT_SIGNAL_MIO_BANK_1[raw_offset],
                    )
                }
            },
            false => match raw_offset {
                (0..=31) => (
                    &EVENT_WAKERS_EMIO_BANK_0[raw_offset],
                    &EVENT_SIGNAL_EMIO_BANK_0[raw_offset],
                ),
                _ => {
                    let raw_offset = raw_offset - 32;
                    (
                        &EVENT_WAKERS_EMIO_BANK_1[raw_offset],
                        &EVENT_SIGNAL_EMIO_BANK_1[raw_offset],
                    )
                }
            },
        }
    }
}

impl embedded_hal_async::digital::Wait for Input {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

impl embedded_hal::digital::ErrorType for Input {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.input.is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.input.is_low())
    }
}
