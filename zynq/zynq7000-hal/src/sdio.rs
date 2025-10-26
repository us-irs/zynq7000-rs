use zynq7000::{
    sdio::SdClockDivisor,
    slcr::{clocks::SrcSelIo, reset::DualRefAndClockReset},
};

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27, Mio40,
    Mio41, Mio42, Mio43, Mio44, Mio45, Mio46, Mio47, Mio50, Mio51,
};
use crate::{
    clocks::Clocks,
    gpio::mio::{
        Mio10, Mio11, Mio12, Mio13, Mio14, Mio15, Mio28, Mio29, Mio30, Mio31, Mio32, Mio33, Mio34,
        Mio35, Mio36, Mio37, Mio38, Mio39, Mio48, Mio49, Pin,
    },
    slcr::Slcr,
    time::Hertz,
};

pub trait Sdio0ClockPin {}
pub trait Sdio0CommandPin {}
pub trait Sdio0Data0Pin {}
pub trait Sdio0Data1Pin {}
pub trait Sdio0Data2Pin {}
pub trait Sdio0Data3Pin {}

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

pub trait Sdio1ClockPin {}
pub trait Sdio1CommandPin {}
pub trait Sdio1Data0Pin {}
pub trait Sdio1Data1Pin {}
pub trait Sdio1Data2Pin {}
pub trait Sdio1Data3Pin {}

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

pub struct Sdio {}

impl Sdio {}

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
    pub fn calculate(src_sel: SrcSelIo, clocks: &Clocks, target_speed: Hertz) -> Self {
        const INIT_CLOCK_HZ: u32 = 400_000;
        let ref_clk = match src_sel {
            SrcSelIo::IoPll | SrcSelIo::IoPllAlt => clocks.io_clocks().ref_clk(),
            SrcSelIo::ArmPll => clocks.arm_clocks().ref_clk(),
            SrcSelIo::DdrPll => clocks.ddr_clocks().ref_clk(),
        };
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
}

pub struct SdioClockConfig {
    src_sel: SrcSelIo,
    divisors: SdioDivisors,
}

impl SdioClockConfig {
    pub fn new(src_sel: SrcSelIo, divisors: SdioDivisors) -> Self {
        Self { src_sel, divisors }
    }

    pub fn calculate_for_io_clock(clocks: &Clocks, target_speed: Hertz) -> Self {
        let divisors = SdioDivisors::calculate(SrcSelIo::IoPll, clocks, target_speed);
        Self {
            src_sel: SrcSelIo::IoPll,
            divisors,
        }
    }
}

/// Reset the UART peripheral using the SLCR reset register for UART.
///
/// Please note that this function will interfere with an already configured
/// UART instance.
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
                cortex_ar::asm::nop();
            }
            regs.reset_ctrl().write_sdio(DualRefAndClockReset::DEFAULT);
        });
    }
}
