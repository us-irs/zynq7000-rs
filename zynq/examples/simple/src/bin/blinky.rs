//! Simple blinky app, showing a PAC variant and a HAL variant.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use embedded_hal::{delay::DelayNs, digital::StatefulOutputPin};
use zynq7000::Peripherals;
use zynq7000_hal::{
    clocks::Clocks,
    gpio::{Output, PinState, mio},
    l2_cache,
    priv_tim::CpuPrivateTimer,
    time::Hertz,
};
use zynq7000_rt as _;

pub const LIB: Lib = Lib::Hal;

/// One user LED is MIO7
const ZEDBOARD_LED_MASK: u32 = 1 << 7;

// Define the clock frequency as a constant.
//
// Not required for the PAC mode, is required for clean delays in HAL mode.
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_333);

#[derive(Debug)]
pub enum Lib {
    Pac,
    Hal,
}

/// Entry point (not called like a normal main function)
#[unsafe(no_mangle)]
pub extern "C" fn boot_core(cpu_id: u32) -> ! {
    if cpu_id != 0 {
        panic!("unexpected CPU ID {}", cpu_id);
    }
    main();
}

#[unsafe(export_name = "main")]
pub fn main() -> ! {
    l2_cache::init_with_defaults(&mut unsafe { zynq7000::l2_cache::L2Cache::new_mmio_fixed() });
    match LIB {
        Lib::Pac => {
            let mut gpio = unsafe { zynq7000::gpio::Gpio::new_mmio_fixed() };
            gpio.bank_0().modify_dirm(|v| v | ZEDBOARD_LED_MASK);
            gpio.bank_0().modify_out_en(|v| v | ZEDBOARD_LED_MASK);
            loop {
                gpio.modify_out_0(|v| v ^ ZEDBOARD_LED_MASK);
                for _ in 0..5_000_000 {
                    nop();
                }
            }
        }
        Lib::Hal => {
            let dp = Peripherals::take().unwrap();
            let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
            // Unwrap okay, we only call this once on core 0 here.
            let mut cpu_tim = CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();
            let mio_pins = mio::Pins::new(dp.gpio);
            let mut led = Output::new_for_mio(mio_pins.mio7, PinState::High);
            loop {
                led.toggle().unwrap();
                cpu_tim.delay_ms(1000);
            }
        }
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
