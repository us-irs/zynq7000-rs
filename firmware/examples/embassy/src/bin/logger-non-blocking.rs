//! Example which uses the UART1 to send log messages.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::info;
use zynq7000::Peripherals;
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    generic_interrupt_handler,
    gic::Configurator,
    gpio::{Output, PinState, mio},
    gtc::GlobalTimerCounter,
    l2_cache,
    time::Hertz,
    uart::{self, ClockConfig, Config, TxAsync, Uart},
};

use zynq7000_rt as _;

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut dp = Peripherals::take().unwrap();
    l2_cache::init_with_defaults(&mut dp.l2c);

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    // Set up the global interrupt controller.
    let mut gic = Configurator::new_with_init(dp.gicc, dp.gicd);
    gic.enable_all_interrupts();
    gic.set_all_spi_interrupt_targets_cpu0();
    gic.enable();
    unsafe {
        gic.enable_interrupts();
    }
    // Set up global timer counter and embassy time driver.
    let gtc = GlobalTimerCounter::new(dp.gtc, clocks.arm_clocks());
    zynq7000_hal::time_driver_gtc::init(clocks.arm_clocks(), gtc);

    let mio_pins = mio::Pins::new(dp.gpio);

    // Set up the UART, we are logging with it.
    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = Uart::new_with_mio_for_uart_1(
        dp.uart_1,
        Config::new_with_clk_config(uart_clk_config),
        (mio_pins.mio48, mio_pins.mio49),
    )
    .unwrap();
    uart.write_all(b"-- Zynq 7000 Logging example --\n\r")
        .unwrap();
    uart.flush().unwrap();

    let (tx, _rx) = uart.split();
    // Safety: We are not forgetting any futures.
    let logger = unsafe { TxAsync::new(tx, true) };

    let mut log_runner =
        zynq7000_hal::log::asynch::init_with_uart_tx(log::LevelFilter::Trace, logger).unwrap();

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let led = Output::new_for_mio(mio_pins.mio7, PinState::Low);
    spawner.spawn(led_task(led).unwrap());
    spawner.spawn(hello_task().unwrap());

    log_runner.run().await
}

#[embassy_executor::task]
async fn led_task(mut mio_led: Output) {
    static ATOMIC_COUNTER: core::sync::atomic::AtomicUsize =
        core::sync::atomic::AtomicUsize::new(0);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        mio_led.toggle().unwrap();
        info!(
            "Toggling LED ({})",
            ATOMIC_COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed)
        );
        ticker.next().await;
    }
}
#[embassy_executor::task]
async fn hello_task() {
    static ATOMIC_COUNTER: core::sync::atomic::AtomicUsize =
        core::sync::atomic::AtomicUsize::new(0);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        info!(
            "Hello from another task ({})",
            ATOMIC_COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed)
        );
        ticker.next().await;
    }
}

#[zynq7000_rt::irq]
pub fn irq_handler() {
    // Safety: Called here once.
    let result = unsafe { generic_interrupt_handler() };
    if let Err(e) = result {
        panic!("Generic interrupt handler failed handling {:?}", e);
    }
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
    let mut uart = unsafe { uart::Uart::steal(uart::UartId::Uart1) };
    writeln!(uart, "panic: {}\r", info).ok();
    loop {}
}
