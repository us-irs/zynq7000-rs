#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use arbitrary_int::u5;
use core::panic::PanicInfo;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write as _;
use log::error;
use zynq7000_hal::{
    InteruptConfig, clocks, generic_interrupt_handler,
    gic::SgiInterrupt,
    gpio::{self, Output},
    gtc,
    time::Hertz,
    uart,
};

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);

static INTERUPT_EXECUTOR_LOW_PRIO: InterruptExecutor = InterruptExecutor::new();
static INTERUPT_EXECUTOR_MED_PRIO: InterruptExecutor = InterruptExecutor::new();

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config {
        init_l2_cache: true,
        level_shifter_config: Some(zynq7000_hal::LevelShifterConfig::EnableAll),
        interrupt_config: Some(InteruptConfig::AllInterruptsToCpu0),
    })
    .unwrap();
    let clocks = clocks::Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    let mut gic = unsafe { zynq7000_hal::gic::Configurator::steal() };
    gic.set_all_sgi_interrupt_targets_cpu0();

    // Set up global timer counter and embassy time driver.
    let gtc = gtc::GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);
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
    uart.write_all(b"-- Zynq 7000 Interrupt Executor --\n\r")
        .unwrap();
    uart.flush().unwrap();

    let (tx, _rx) = uart.split();
    let mut logger = uart::TxAsync::new(tx, true);
    let log_reader = zynq7000_hal::log::asynch::init(log::LevelFilter::Trace).unwrap();

    gic.set_sgi_interrupt_priority(sgi_interrupt_low_prio, u5::new(2));
    gic.set_sgi_interrupt_priority(sgi_interrupt_med_prio, u5::new(1));

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

    let mut log_buf: [u8; 2048] = [0; 2048];
    loop {
        let read_bytes = log_reader.read(&mut log_buf).await;
        if read_bytes > 0 {
            // Unwrap okay, checked that size is larger than 0.
            logger.write(&log_buf[0..read_bytes]).unwrap().await;
        }
        Ticker::every(Duration::from_millis(1000)).next().await;
    }
}

#[embassy_executor::task]
async fn led_task(mut mio_led: Output) {
    static ATOMIC_COUNTER: core::sync::atomic::AtomicUsize =
        core::sync::atomic::AtomicUsize::new(0);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        mio_led.toggle().unwrap();
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
