//! Simple blinky app, showing a PAC variant and a HAL variant.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, digital::StatefulOutputPin};
use zynq7000_hal::{
    InteruptConfig,
    clocks::Clocks,
    gpio::{Output, PinState, mio},
    priv_tim::CpuPrivateTimer,
    time::Hertz,
};

pub const LIB: Lib = Lib::Hal;

// Define the clock frequency as a constant.
//
// Not required for the PAC mode, is required for clean delays in HAL mode.
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_333);

#[derive(Debug)]
pub enum Lib {
    Pac,
    Hal,
}

#[zynq7000_rt::entry]
fn main() -> ! {
    let dp = zynq7000_hal::init(zynq7000_hal::Config {
        init_l2_cache: true,
        level_shifter_config: Some(zynq7000_hal::LevelShifterConfig::EnableAll),
        interrupt_config: Some(InteruptConfig::AllInterruptsToCpu0),
    })
    .expect("Failed to initialize Zynq7000");

    defmt::println!("-- Zynq7000 defmt test application --");
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    defmt::info!("clocks {:?}", clocks);
    // Unwrap okay, we only call this once on core 0 here.
    let mut cpu_tim = CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();
    let mio_pins = mio::Pins::new(dp.gpio);
    let mut led = Output::new_for_mio(mio_pins.mio7, PinState::High);
    loop {
        defmt::info!("toggling LED!");
        zynq7000_hal::cache::clean_and_invalidate_data_cache();
        led.toggle().unwrap();
        cpu_tim.delay_ms(1000);
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {}

#[zynq7000_rt::exception(DataAbort)]
fn data_abort_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(Undefined)]
fn undefined_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(PrefetchAbort)]
fn prefetch_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

/// Panic handler
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        nop();
    }
}
