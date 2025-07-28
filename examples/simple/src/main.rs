//! Simple blinky app, showing a PAC variant and a HAL variant.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use embedded_hal::digital::StatefulOutputPin;
use zynq7000::PsPeripherals;
use zynq7000_hal::gpio::{Output, PinState, mio};
use zynq7000_rt as _;

/// One user LED is MIO7
const ZEDBOARD_LED_MASK: u32 = 1 << 7;

#[derive(Debug)]
pub enum Lib {
    Pac,
    Hal,
}

const LIB: Lib = Lib::Hal;

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
            let dp = PsPeripherals::take().unwrap();
            let mio_pins = mio::Pins::new(dp.gpio);
            let mut led = Output::new_for_mio(mio_pins.mio7, PinState::High);
            loop {
                led.toggle().unwrap();
                for _ in 0..5_000_000 {
                    nop();
                }
            }
        }
    }
}

#[zynq7000_rt::irq]
pub fn irq_handler() {}

#[unsafe(no_mangle)]
pub extern "C" fn _abort_handler() {
    loop {
        nop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _undefined_handler() {
    loop {
        nop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _prefetch_handler() {
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
