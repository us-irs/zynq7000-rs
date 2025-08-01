//! Simple blinky app, showing a PAC variant and a HAL variant.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use zynq7000_rt as _;

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
    loop {
        cortex_ar::asm::nop();
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
