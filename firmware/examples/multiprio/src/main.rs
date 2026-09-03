#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_time::{Duration, Ticker};
use embedded_io::Write as _;
use log::error;
use zynq7000_hal::{
    clocks, generic_interrupt_handler,
    gic::{Priority, SgiInterrupt},
    gpio::{self, Output},
    gtc,
    time::Hertz,
    uart,
};

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_333);

static INTERUPT_EXECUTOR_LOW_PRIO: InterruptExecutor = InterruptExecutor::new();
static INTERUPT_EXECUTOR_MED_PRIO: InterruptExecutor = InterruptExecutor::new();

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let config = zynq7000_hal::Config::default();
    let periphs = zynq7000_hal::init(config).unwrap();
    let clocks = clocks::Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    let mut gic = unsafe { zynq7000_hal::gic::Configurator::steal() };
    gic.set_all_sgi_interrupt_targets_cpu0();

    // Set up global timer counter and embassy time driver.
    let gtc = gtc::GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_hal::time_driver_gtc::init(clocks.arm_clocks(), gtc);
    let mio_pins = gpio::mio::Pins::new(periphs.gpio);
    let led = gpio::Output::new_for_mio(mio_pins.mio7, gpio::PinState::Low);
    let sgi_interrupt_low_prio = SgiInterrupt::new(0).unwrap();
    let sgi_interrupt_med_prio = SgiInterrupt::new(1).unwrap();

    // Set up the UART, we are logging with it.
    let uart_clk_config = uart::ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::Uart::new_with_mio_for_uart_1(
        periphs.uart_1,
        uart::Config::new_with_clk_config(uart_clk_config),
        (mio_pins.mio48, mio_pins.mio49),
    )
    .unwrap();
    uart.write_all(b"-- Zynq 7000 Interrupt Executor --\n\r");
    uart.flush().unwrap();

    let (tx, _rx) = uart.split();
    let logger = uart::TxAsync::new(tx, true);
    let mut log_runner =
        zynq7000_hal::log::asynch::init_with_uart_tx(log::LevelFilter::Trace, logger)
            .expect("TX UART init failed");

    gic.set_sgi_interrupt_priority(sgi_interrupt_low_prio, Priority::P2);
    gic.set_sgi_interrupt_priority(sgi_interrupt_med_prio, Priority::P1);

    zynq7000_hal::register_interrupt(
        zynq7000_hal::Interrupt::Sgi(sgi_interrupt_low_prio),
        on_interrupt_low_prio,
    );
    zynq7000_hal::register_interrupt(
        zynq7000_hal::Interrupt::Sgi(sgi_interrupt_med_prio),
        on_interrupt_med_prio,
    );
    let spawner = INTERUPT_EXECUTOR_LOW_PRIO.start(sgi_interrupt_low_prio.as_u4());
    spawner.spawn(led_task(led).unwrap());
    let spawner = INTERUPT_EXECUTOR_MED_PRIO.start(sgi_interrupt_med_prio.as_u4());
    spawner.spawn(hello_task().unwrap());

    log_runner.run().await;
}

#[embassy_executor::task]
async fn led_task(mut mio_led: Output) {
    static ATOMIC_COUNTER: core::sync::atomic::AtomicUsize =
        core::sync::atomic::AtomicUsize::new(0);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        mio_led.toggle();
        log::info!(
            "Toggling LED ({})",
            ATOMIC_COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed)
        );
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn hello_task() {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        log::info!("Hello from the low priority task");
        ticker.next().await;
    }
}

unsafe fn on_interrupt_low_prio() {
    unsafe {
        INTERUPT_EXECUTOR_LOW_PRIO.on_interrupt();
    }
}

unsafe fn on_interrupt_med_prio() {
    unsafe {
        INTERUPT_EXECUTOR_MED_PRIO.on_interrupt();
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
    error!("Panic: {info:?}");
    loop {}
}
