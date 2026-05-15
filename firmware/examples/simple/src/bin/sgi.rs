//! Example which uses the UART1 to send log messages.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::{panic::PanicInfo, sync::atomic::AtomicU64};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    gic::{self, Configurator, Interrupt, SgiInterrupt},
    gpio::{Output, PinState, mio},
    l2_cache,
    time::Hertz,
    uart::{ClockConfig, Config, Uart},
};

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);

static SGI_COUNTER: AtomicU64 = AtomicU64::new(0);

#[zynq7000_rt::entry]
fn main() -> ! {
    let mut dp = zynq7000::Peripherals::take().unwrap();
    l2_cache::init_with_defaults(&mut dp.l2c);

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    // Set up the global interrupt controller.
    let mut gic = Configurator::new_with_init(dp.gicc, dp.gicd);
    gic.enable_all_interrupts();
    gic.set_all_spi_interrupt_targets_cpu0();
    gic.set_all_sgi_interrupt_targets_cpu0();
    gic.enable();
    // Enable interrupt exception.
    unsafe { gic.enable_interrupts() };

    // Set up the UART, we are logging with it.
    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mio_pins = mio::Pins::new(dp.gpio);

    let mut uart = Uart::new_with_mio_for_uart_1(
        dp.uart_1,
        Config::new_with_clk_config(uart_clk_config),
        (mio_pins.mio48, mio_pins.mio49),
    )
    .unwrap();
    uart.write_all(b"-- Zynq 7000 Software Interrupt example --\n\r")
        .unwrap();
    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {boot_mode:?}");

    let mut led = Output::new_for_mio(mio_pins.mio7, PinState::Low);
    loop {
        let gtc = SGI_COUNTER.load(core::sync::atomic::Ordering::Relaxed);
        info!("Hello, world!");
        info!("SGI counter: {gtc}");
        gic.trigger_software_interrupt(SgiInterrupt::new(0).unwrap());
        led.toggle().unwrap();
        for _ in 0..5_000_000 {
            nop();
        }
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {
    let mut gic_helper = gic::InterruptGuard::new();
    let irq_info = gic_helper.interrupt_info();
    match irq_info.interrupt() {
        Interrupt::Sgi(_sgi) => {
            // TODO: Send ID to main thread.
            SGI_COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        Interrupt::Ppi(_ppi_interrupt) => (),
        Interrupt::Spi(_spi_interrupt) => (),
        Interrupt::Invalid(_) => (),
        Interrupt::Spurious => (),
    }
    gic_helper.end_of_interrupt(irq_info);
}

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
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {info:?}");
    loop {}
}
