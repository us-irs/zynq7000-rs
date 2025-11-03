use arbitrary_int::{traits::Integer as _, u3, u6};
use zynq7000::{
    sdio::{SDIO_BASE_ADDR_0, SDIO_BASE_ADDR_1, SdClockDivisor},
    slcr::{clocks::SrcSelIo, reset::DualRefAndClockReset},
};

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27, Mio40,
    Mio41, Mio42, Mio43, Mio44, Mio45, Mio46, Mio47, Mio50, Mio51,
};
use crate::{
    clocks::{Clocks, IoClocks},
    gpio::{
        IoPeriphPin,
        mio::{
            Mio10, Mio11, Mio12, Mio13, Mio14, Mio15, Mio28, Mio29, Mio30, Mio31, Mio32, Mio33,
            Mio34, Mio35, Mio36, Mio37, Mio38, Mio39, Mio48, Mio49, MioPin, MuxConfig, Pin,
        },
    },
    slcr::Slcr,
    time::Hertz,
};

pub const MUX_CONF: MuxConfig = MuxConfig::new_with_l3(u3::new(0b100));

pub trait Sdio0ClockPin: MioPin {}
pub trait Sdio0CommandPin: MioPin {}
pub trait Sdio0Data0Pin: MioPin {}
pub trait Sdio0Data1Pin: MioPin {}
pub trait Sdio0Data2Pin: MioPin {}
pub trait Sdio0Data3Pin: MioPin {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0ClockPin for Pin<Mio16> {}
impl Sdio0ClockPin for Pin<Mio28> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0ClockPin for Pin<Mio40> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0CommandPin for Pin<Mio17> {}
impl Sdio0CommandPin for Pin<Mio29> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0CommandPin for Pin<Mio41> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data0Pin for Pin<Mio18> {}
impl Sdio0Data0Pin for Pin<Mio30> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data0Pin for Pin<Mio42> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data1Pin for Pin<Mio19> {}
impl Sdio0Data1Pin for Pin<Mio31> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data1Pin for Pin<Mio43> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data2Pin for Pin<Mio20> {}
impl Sdio0Data2Pin for Pin<Mio32> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data2Pin for Pin<Mio44> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data3Pin for Pin<Mio21> {}
impl Sdio0Data3Pin for Pin<Mio33> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio0Data3Pin for Pin<Mio45> {}

pub trait Sdio1ClockPin: MioPin {}
pub trait Sdio1CommandPin: MioPin {}
pub trait Sdio1Data0Pin: MioPin {}
pub trait Sdio1Data1Pin: MioPin {}
pub trait Sdio1Data2Pin: MioPin {}
pub trait Sdio1Data3Pin: MioPin {}

impl Sdio1ClockPin for Pin<Mio12> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1ClockPin for Pin<Mio24> {}
impl Sdio1ClockPin for Pin<Mio36> {}
impl Sdio1ClockPin for Pin<Mio48> {}

impl Sdio1CommandPin for Pin<Mio11> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1CommandPin for Pin<Mio23> {}
impl Sdio1CommandPin for Pin<Mio35> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1CommandPin for Pin<Mio47> {}

impl Sdio1Data0Pin for Pin<Mio10> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data0Pin for Pin<Mio22> {}
impl Sdio1Data0Pin for Pin<Mio34> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data0Pin for Pin<Mio46> {}

impl Sdio1Data1Pin for Pin<Mio13> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data1Pin for Pin<Mio25> {}
impl Sdio1Data1Pin for Pin<Mio37> {}
impl Sdio1Data1Pin for Pin<Mio49> {}

impl Sdio1Data2Pin for Pin<Mio14> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data2Pin for Pin<Mio26> {}
impl Sdio1Data2Pin for Pin<Mio38> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data2Pin for Pin<Mio50> {}

impl Sdio1Data2Pin for Pin<Mio15> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data3Pin for Pin<Mio27> {}
impl Sdio1Data3Pin for Pin<Mio39> {}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl Sdio1Data3Pin for Pin<Mio51> {}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SdioId {
    Sdio0,
    Sdio1,
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
                SdioId::Sdio0 => zynq7000::sdio::Registers::new_mmio_fixed_0(),
                SdioId::Sdio1 => zynq7000::sdio::Registers::new_mmio_fixed_1(),
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
            return Some(SdioId::Sdio0);
        } else if base_addr == SDIO_BASE_ADDR_1 {
            return Some(SdioId::Sdio1);
        }
        None
    }
}

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

pub struct SdioClockConfig {
    /// Selects the source clock for the SDIO peripheral reference clock.
    pub src_sel: SrcSelIo,
    /// Selects the divisor which divies the source clock to create the SDIO peripheral
    /// reference clock.
    pub ref_clock_divisor: u6,
    /// The SDIO peripheral reference clock is divided again to create the SDIO clock.
    pub sdio_clock_divisors: SdioDivisors,
}

impl SdioClockConfig {
    pub fn new(
        src_sel: SrcSelIo,
        ref_clock_divisor: u6,
        sdio_clock_divisors: SdioDivisors,
    ) -> Self {
        Self {
            src_sel,
            ref_clock_divisor,
            sdio_clock_divisors,
        }
    }

    pub fn calculate_for_io_clock(
        io_clocks: &IoClocks,
        target_ref_clock: Hertz,
        target_sdio_speed: Hertz,
    ) -> Option<Self> {
        let ref_clk = io_clocks.ref_clk();
        let io_ref_clock_divisor = ref_clk.raw().div_ceil(target_ref_clock.raw());
        if io_ref_clock_divisor > u6::MAX.as_u32() {
            return None;
        }
        let target_speed = ref_clk / io_ref_clock_divisor;

        let sdio_clock_divisors = SdioDivisors::calculate(target_speed, target_sdio_speed);
        Some(Self {
            src_sel: SrcSelIo::IoPll,
            ref_clock_divisor: u6::new(io_ref_clock_divisor as u8),
            sdio_clock_divisors,
        })
    }
}
/// SDIO low-level interface.
///
/// Basic building block for higher-level abstraction.
pub struct SdioLowLevel {
    id: SdioId,
    /// Register block. Direct public access is allowed to allow low-level operations.
    pub regs: zynq7000::sdio::MmioRegisters<'static>,
}

impl SdioLowLevel {
    /// Create a new SDIO low-level interface from the given register block.
    ///
    /// Returns [None] if the given registers block base address does not correspond to a valid
    /// Ethernet peripheral.
    pub fn new(regs: zynq7000::sdio::MmioRegisters<'static>) -> Option<Self> {
        let id = regs.id()?;
        Some(Self { id, regs })
    }

    /// Common SDIO clock configuration routine which should be called once before using the SDIO.
    ///
    /// This does NOT disable the clock, which should be done before changing the clock
    /// configuration. It also does NOT enable the clock.
    ///
    /// It will configure the SDIO peripheral clock as well as initializing the SD clock frequency
    /// divisor based on the initial phase divider specified in the [SdioDivisors] field of the
    /// configuration.
    pub fn configure_clock(&mut self, clock_config: &SdioClockConfig) {
        unsafe {
            Slcr::with(|slcr| {
                slcr.clk_ctrl().modify_sdio_clk_ctrl(|mut val| {
                    val.set_srcsel(clock_config.src_sel);
                    val.set_divisor(clock_config.ref_clock_divisor);
                    if self.id == SdioId::Sdio1 {
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
    pub fn enable_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sd_clock_enable(true);
            val
        });
    }

    #[inline]
    pub fn disable_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sd_clock_enable(false);
            val
        });
    }

    /// Reset the SDIO peripheral using the SLCR reset register for SDIO.
    pub fn reset(&mut self, cycles: u32) {
        reset(self.id, cycles);
    }
}

pub struct Sdio {
    ll: SdioLowLevel,
}

impl Sdio {
    pub fn new_for_sdio_0<
        Sdio0Clock: Sdio0ClockPin,
        Sdio0Command: Sdio0CommandPin,
        Sdio0Data0: Sdio0Data0Pin,
        Sdio0Data1: Sdio0Data1Pin,
        Sdio0Data2: Sdio0Data2Pin,
        Sdio0Data3: Sdio0Data3Pin,
    >(
        regs: zynq7000::sdio::MmioRegisters<'static>,
        clock_config: SdioClockConfig,
        clock_pin: Sdio0Clock,
        command_pin: Sdio0Command,
        data_pins: (Sdio0Data0, Sdio0Data1, Sdio0Data2, Sdio0Data3),
    ) -> Option<Self> {
        let id = regs.id()?;
        if id != SdioId::Sdio1 {
            return None;
        }
        Some(Self::new(
            regs,
            clock_config,
            clock_pin,
            command_pin,
            data_pins,
        ))
    }

    pub fn new_for_sdio_1<
        Sdio1Clock: Sdio1ClockPin,
        Sdio1Command: Sdio1CommandPin,
        Sdio1Data0: Sdio1Data0Pin,
        Sdio1Data1: Sdio1Data1Pin,
        Sdio1Data2: Sdio1Data2Pin,
        Sdio1Data3: Sdio1Data3Pin,
    >(
        regs: zynq7000::sdio::MmioRegisters<'static>,
        clock_config: SdioClockConfig,
        clock_pin: Sdio1Clock,
        command_pin: Sdio1Command,
        data_pins: (Sdio1Data0, Sdio1Data1, Sdio1Data2, Sdio1Data3),
    ) -> Option<Self> {
        let id = regs.id()?;
        if id != SdioId::Sdio1 {
            return None;
        }
        Some(Self::new(
            regs,
            clock_config,
            clock_pin,
            command_pin,
            data_pins,
        ))
    }

    fn new(
        regs: zynq7000::sdio::MmioRegisters<'static>,
        clock_config: SdioClockConfig,
        clock_pin: impl MioPin,
        command_pin: impl MioPin,
        data_pins: (impl MioPin, impl MioPin, impl MioPin, impl MioPin),
    ) -> Self {
        let mut ll = SdioLowLevel::new(regs).unwrap();
        Self::initialize(&mut ll, &clock_config);
        IoPeriphPin::new(clock_pin, MUX_CONF, None);
        IoPeriphPin::new(command_pin, MUX_CONF, None);
        IoPeriphPin::new(data_pins.0, MUX_CONF, None);
        IoPeriphPin::new(data_pins.1, MUX_CONF, None);
        IoPeriphPin::new(data_pins.2, MUX_CONF, None);
        IoPeriphPin::new(data_pins.3, MUX_CONF, None);
        Self { ll }
    }

    fn initialize(ll: &mut SdioLowLevel, clock_config: &SdioClockConfig) {
        ll.reset(10);
        // TODO: SW reset for all?
        // TODO: Internal clock?
        ll.disable_clock();
        ll.configure_clock(clock_config);
        ll.enable_clock();

        // TODO: There is probably some other configuration necessary.. the docs really are not
        // complete here..
        unsafe {}
    }

    #[inline]
    pub fn regs(&mut self) -> &mut zynq7000::sdio::MmioRegisters<'static> {
        &mut self.ll.regs
    }
}

/// Reset the SDIO peripheral using the SLCR reset register for SDIO.
///
/// Please note that this function will interfere with an already configured
/// SDIO instance.
#[inline]
pub fn reset(id: SdioId, cycles: u32) {
    let assert_reset = match id {
        SdioId::Sdio0 => DualRefAndClockReset::builder()
            .with_periph1_ref_rst(false)
            .with_periph0_ref_rst(true)
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        SdioId::Sdio1 => DualRefAndClockReset::builder()
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
            regs.reset_ctrl().write_sdio(DualRefAndClockReset::DEFAULT);
        });
    }
}
