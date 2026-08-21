//! System watchdog timer (SWDT) driver.
#![deny(missing_docs)]
use arbitrary_int::{u3, u12};
use zynq7000::slcr::WdtClockSelectRegister;

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{Mio27, Mio51};
use crate::{
    clocks::ArmClocks,
    gpio::{
        IoPeriphPin,
        mio::{Mio14, Mio15, Mio26, Mio38, Mio39, Mio50, Mio52, Mio53, MioPin, MuxConfig, Pin},
    },
    time::Hertz,
};

use zynq7000::swdt;

pub use arbitrary_int::u24;
pub use zynq7000::swdt::{IrqLengthCycles, PclkPrescaler};

/// SWDT pin mux configuration, valid for both the clock input and reset output pins.
pub const SWDT_MUX_CONF: MuxConfig = MuxConfig::new_with_l3(u3::new(0b011));

/// System Watchdog (SWDT) driver.
pub struct SystemWatchdog(swdt::MmioRegisters<'static>);

/// External clock input pin for the SWDT.
pub trait SwdtClockSource: MioPin {}
/// Reset output pin for the SWDT.
pub trait SwdtResetPin: MioPin {}

impl SwdtClockSource for Pin<Mio14> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SwdtClockSource for Pin<Mio26> {}
impl SwdtClockSource for Pin<Mio38> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SwdtClockSource for Pin<Mio50> {}
impl SwdtClockSource for Pin<Mio52> {}

impl SwdtResetPin for Pin<Mio15> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SwdtResetPin for Pin<Mio27> {}
impl SwdtResetPin for Pin<Mio39> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl SwdtResetPin for Pin<Mio51> {}
impl SwdtResetPin for Pin<Mio53> {}

/// SWDT clock configuration: the prescaler and the 24-bit counter restart value derived from it.
///
/// Implements [`Default`], so a struct literal only needs to set the fields it cares about and
/// fill in the rest with `..Default::default()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    /// Prescaler applied to PCLK to derive the watchdog counter clock.
    pub prescaler: swdt::PclkPrescaler,
    /// 24-bit counter restart value. The watchdog counter reloads with `0xNNNFFF`, where
    /// `NNN` are the upper 12 bits of this value.
    pub restart_value: u24,
}

impl Default for ClockConfig {
    /// A [`swdt::PclkPrescaler::Div8`] prescaler and the maximum restart value.
    fn default() -> Self {
        Self {
            prescaler: swdt::PclkPrescaler::Div8,
            restart_value: u24::new(0xFFFFFF),
        }
    }
}

impl ClockConfig {
    /// Calculate a [`ClockConfig`] for a watchdog timeout of `timeout_ms` milliseconds, using
    /// the internal CPU 1x clock as the SWDT input clock.
    ///
    /// See [`ClockConfig::calculate`] for how the timeout is converted into the prescaler and
    /// restart value.
    pub fn calculate_cpu1x(clocks: &ArmClocks, timeout_ms: u32) -> Self {
        Self::calculate(clocks.cpu_1x_clk(), timeout_ms)
    }

    /// Calculate a [`ClockConfig`] for a watchdog timeout of `timeout_ms` milliseconds, from an
    /// arbitrary SWDT input clock frequency, e.g. an external clock supplied via MIO or EMIO.
    ///
    /// Picks the smallest prescaler ([`swdt::PclkPrescaler::Div8`] first) for which the
    /// requested timeout still fits into the 24-bit counter. If `timeout_ms` is too large to fit
    /// even with [`swdt::PclkPrescaler::Div4096`], the restart value saturates at its maximum
    /// instead of panicking or overflowing, giving the longest timeout achievable with this input
    /// clock.
    pub fn calculate(input_clock: Hertz, timeout_ms: u32) -> Self {
        let input_hz = input_clock.to_Hz() as u64;
        let mut prescaler = swdt::PclkPrescaler::Div4096;
        let mut reload_count = 0u64;
        for candidate in swdt::PclkPrescaler::ALL {
            reload_count = (input_hz / candidate.divisor() as u64) * timeout_ms as u64 / 1000;
            if reload_count <= 0xFF_FFFF {
                prescaler = candidate;
                break;
            }
        }
        Self {
            prescaler,
            restart_value: u24::new(reload_count.min(0xFF_FFFF) as u32),
        }
    }
}

/// Watchdog configuration.
///
/// Implements [`Default`], so a struct literal only needs to set the fields it cares about and
/// fill in the rest with `..Default::default()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Prescaler and counter restart value. See [`ClockConfig::calculate`] to derive this from
    /// a desired timeout instead of setting it directly.
    pub clk_config: ClockConfig,
    /// Length of the pulse generated on the IRQ output.
    pub irq_len: swdt::IrqLengthCycles,
    /// Enables the IRQ output on watchdog expiry.
    pub irq_enable: bool,
    /// Enables the reset output on watchdog expiry.
    pub reset_enable: bool,
}

impl Default for Config {
    /// The default [`ClockConfig`], a [`swdt::IrqLengthCycles::ThirtyTwo`] IRQ pulse length,
    /// the IRQ output disabled, and the reset output enabled.
    fn default() -> Self {
        Self {
            clk_config: ClockConfig::default(),
            irq_len: swdt::IrqLengthCycles::ThirtyTwo,
            irq_enable: false,
            reset_enable: true,
        }
    }
}

impl SystemWatchdog {
    /// Wrap an already-configured SWDT register block without touching any configuration.
    ///
    /// Useful to get a [`SystemWatchdog`] handle for [`feed`](Self::feed) or
    /// [`expired`](Self::expired) inside an interrupt handler, where the watchdog was already
    /// armed by one of the `new_with_*` constructors elsewhere and must not be reconfigured.
    #[inline]
    pub fn new_unchecked(regs: zynq7000::swdt::MmioRegisters<'static>) -> Self {
        Self(regs)
    }

    /// Create a SWDT driver using the internal CPU 1x clock, with the reset output routed to
    /// EMIO (PL fabric) only. No MIO pin is configured.
    pub fn new_with_cpu1x_clk_emio_reset(
        mut regs: zynq7000::swdt::MmioRegisters<'static>,
        config: Config,
    ) -> Self {
        Self::generic_config(
            &mut regs,
            zynq7000::slcr::WdtClockSelect::InternalCpu1x,
            config,
        );

        Self(regs)
    }

    /// Create a SWDT driver using the internal CPU 1x clock, with the reset output routed to
    /// its MIO pin instead of EMIO.
    pub fn new_with_cpu1x_clk_mio_reset<R: SwdtResetPin>(
        mut regs: zynq7000::swdt::MmioRegisters<'static>,
        rst_pin: R,
        config: Config,
    ) -> Self {
        // MIO pins only support a pull-up, not a pull-down, and the reset output is
        // active-high, so a pull-up would be wrong here. Leave it unpulled instead.
        IoPeriphPin::new(rst_pin, SWDT_MUX_CONF, None);
        Self::generic_config(
            &mut regs,
            zynq7000::slcr::WdtClockSelect::InternalCpu1x,
            config,
        );

        Self(regs)
    }

    /// Create a SWDT driver using an external clock supplied via EMIO (PL fabric), with the
    /// reset output also routed to EMIO only. No MIO pin is configured.
    pub fn new_with_emio_clk_emio_reset(
        mut regs: zynq7000::swdt::MmioRegisters<'static>,
        config: Config,
    ) -> Self {
        Self::generic_config(
            &mut regs,
            zynq7000::slcr::WdtClockSelect::ExternalEmioOrMio,
            config,
        );

        Self(regs)
    }

    /// Create a SWDT driver using an external clock supplied via EMIO (PL fabric), with the
    /// reset output routed to its MIO pin instead of EMIO.
    pub fn new_with_emio_clk_mio_reset<R: SwdtResetPin>(
        mut regs: zynq7000::swdt::MmioRegisters<'static>,
        rst_pin: R,
        config: Config,
    ) -> Self {
        // MIO pins only support a pull-up, not a pull-down, and the reset output is
        // active-high, so a pull-up would be wrong here. Leave it unpulled instead.
        IoPeriphPin::new(rst_pin, SWDT_MUX_CONF, None);
        Self::generic_config(
            &mut regs,
            zynq7000::slcr::WdtClockSelect::ExternalEmioOrMio,
            config,
        );

        Self(regs)
    }

    /// Create a SWDT driver using an external clock supplied on an MIO pin, with the reset
    /// output routed to EMIO (PL fabric) only.
    pub fn new_with_mio_clk_emio_reset<P: SwdtClockSource>(
        mut regs: zynq7000::swdt::MmioRegisters<'static>,
        clk_pin: P,
        config: Config,
    ) -> Self {
        IoPeriphPin::new(clk_pin, SWDT_MUX_CONF, None);
        Self::generic_config(
            &mut regs,
            zynq7000::slcr::WdtClockSelect::ExternalEmioOrMio,
            config,
        );

        Self(regs)
    }

    /// Create a SWDT driver using an external clock supplied on an MIO pin, with the reset
    /// output also routed to its paired MIO pin instead of EMIO.
    pub fn new_with_mio_clk_mio_reset<P: SwdtClockSource, R: SwdtResetPin>(
        mut regs: zynq7000::swdt::MmioRegisters<'static>,
        clk_pin: P,
        rst_pin: R,
        config: Config,
    ) -> Self {
        IoPeriphPin::new(clk_pin, SWDT_MUX_CONF, None);
        // MIO pins only support a pull-up, not a pull-down, and the reset output is
        // active-high, so a pull-up would be wrong here. Leave it unpulled instead.
        IoPeriphPin::new(rst_pin, SWDT_MUX_CONF, None);
        Self::generic_config(
            &mut regs,
            zynq7000::slcr::WdtClockSelect::ExternalEmioOrMio,
            config,
        );

        Self(regs)
    }

    fn generic_config(
        regs: &mut zynq7000::swdt::MmioRegisters<'static>,
        wdt_clock_sel: zynq7000::slcr::WdtClockSelect,
        config: Config,
    ) {
        Self::disable_reg(regs);
        // Safety: We took ownership of the SWDT peripheral
        unsafe {
            crate::Slcr::with(|slcr| {
                slcr.write_wdt_clk_set(
                    WdtClockSelectRegister::builder()
                        .with_sel(wdt_clock_sel)
                        .build(),
                );
            });
        }
        regs.write_counter_config(
            zynq7000::swdt::CounterConfig::builder()
                .with_key(swdt::COUNTER_CONFIG_KEY)
                .with_restart_upper_bits(u12::new(
                    (config.clk_config.restart_value.value() >> 12) as u16,
                ))
                .with_clock_prescaler(config.clk_config.prescaler)
                .build(),
        );
        regs.write_control(
            zynq7000::swdt::Control::builder()
                .with_key(swdt::CONTROL_KEY)
                .with_should_be_four(u3::new(4))
                .with_irq_len(config.irq_len)
                .with_irq_enable(config.irq_enable)
                .with_reset_enable(config.reset_enable)
                .with_enable(true)
                .build(),
        );
        // Force the down-counter to reload from the restart value/prescaler just configured
        // above. Without this, the counter keeps running from whatever it held before (e.g. a
        // leftover value from a prior boot stage or run), making the first expiry after
        // (re-)configuring the watchdog land at an unpredictable time instead of matching the
        // configured timeout.
        regs.write_restart(swdt::Restart::RESTART);
    }

    /// Disable the watchdog.
    fn disable_reg(regs: &mut zynq7000::swdt::MmioRegisters<'static>) {
        regs.write_control(
            zynq7000::swdt::Control::builder()
                .with_key(swdt::CONTROL_KEY)
                .with_should_be_four(u3::new(4))
                .with_irq_len(swdt::IrqLengthCycles::Four)
                .with_irq_enable(false)
                .with_reset_enable(false)
                .with_enable(false)
                .build(),
        );
    }

    /// Disable the watchdog counter.
    #[inline]
    pub fn disable(&mut self) {
        Self::disable_reg(&mut self.0);
    }

    /// Restart (kick/feed) the watchdog counter, preventing it from expiring.
    ///
    /// This also resets the status field which indicates the watchdog expired.
    #[inline]
    pub fn feed(&mut self) {
        self.0.write_restart(swdt::Restart::RESTART);
    }

    /// Returns true if the watchdog counter has reached zero.
    #[inline]
    pub fn expired(&self) -> bool {
        self.0.read_status().watchdog_zero()
    }
}
