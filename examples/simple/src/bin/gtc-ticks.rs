//! Example which uses the global timer for a simple tick counter.
#![no_std]
#![no_main]

use core::{panic::PanicInfo, sync::atomic::AtomicU64};
use cortex_ar::asm::nop;
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zynq7000_hal::{
    clocks::Clocks,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{Output, PinState, mio},
    gtc::Gtc,
    prelude::*,
    time::Hertz,
    uart::{ClkConfigRaw, Uart, UartConfig},
};

use zynq7000_rt as _;

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);

static MS_TICKS: AtomicU64 = AtomicU64::new(0);

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
    let dp = zynq7000::PsPeripherals::take().unwrap();
    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    // Set up the global interrupt controller.
    let mut gic = GicConfigurator::new_with_init(dp.gicc, dp.gicd);
    gic.enable_all_interrupts();
    gic.set_all_spi_interrupt_targets_cpu0();
    gic.enable();
    // Enable interrupt exception.
    unsafe { gic.enable_interrupts() };
    // Set up the UART, we are logging with it.
    let uart_clk_config = ClkConfigRaw::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut gtc = Gtc::new(dp.gtc, clocks.arm_clocks());
    let ticks = gtc.frequency_to_ticks(1000.Hz());
    gtc.set_auto_increment_value(ticks);
    gtc.set_comparator(gtc.read_timer() + ticks as u64);
    gtc.enable_auto_increment();
    gtc.enable_interrupt();
    gtc.enable();

    // This structure holds all MIO pins.
    let mio_pins = mio::Pins::new(dp.gpio);
    let mut uart = Uart::new_with_mio(
        dp.uart_1,
        UartConfig::new_with_clk_config(uart_clk_config),
        (mio_pins.mio48, mio_pins.mio49),
    )
    .unwrap();
    uart.write_all(b"-- Zynq 7000 GTC Ticks example --\n\r")
        .unwrap();
    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    let mut led = Output::new_for_mio(mio_pins.mio7, PinState::Low);
    loop {
        info!(
            "MS_TICKS: {}",
            MS_TICKS.load(core::sync::atomic::Ordering::Relaxed)
        );
        led.toggle().unwrap();
        for _ in 0..5_000_000 {
            nop();
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _irq_handler() {
    let mut gic_helper = GicInterruptHelper::new();
    let irq_info = gic_helper.acknowledge_interrupt();
    match irq_info.interrupt() {
        Interrupt::Sgi(_) => (),
        Interrupt::Ppi(ppi_interrupt) => {
            if ppi_interrupt == zynq7000_hal::gic::PpiInterrupt::GlobalTimer {
                MS_TICKS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
        }
        Interrupt::Spi(_spi_interrupt) => (),
        Interrupt::Invalid(_) => (),
        Interrupt::Spurious => (),
    }
    gic_helper.end_of_interrupt(irq_info);
}

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
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {info:?}");
    loop {}
}
