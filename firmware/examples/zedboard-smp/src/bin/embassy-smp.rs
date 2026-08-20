//! Embassy hello-world for the Zedboard, running an independent executor on each core.
//!
//! CPU0 blinks a MIO LED and sends its state to CPU1 over a cross-core `embassy_sync::Signal`,
//! which mirrors it on an EMIO LED. CPU1 also runs its own heartbeat task using
//! `embassy_time::Timer`, entirely independently of CPU0, to exercise the GTC time driver's
//! per-core alarm state. Both cores log through the async UART logger, which CPU0 drains in
//! its own task.
#![no_std]
#![no_main]

use embassy_executor::Executor;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use embedded_io::Write as _;
use log::info;
use static_cell::StaticCell;
use zedboard_smp::{Handoff, PS_CLOCK_FREQUENCY};
use zynq7000_hal::{BootMode, clocks::Clocks, gpio, gtc, log::LevelFilter, uart};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard Embassy SMP example --\n\r";

/// Carries CPU0's LED state to CPU1's LED task. Only the latest state matters, so a `Signal`
/// fits better than a `Channel`. Sharing this between two independently-running executors is
/// safe: `critical_section` (multi-core feature) is a real cross-core spinlock, not just
/// IRQ-disable, and the `cortex_ar` executor wakes via a global `SEV` that reaches every core
/// sitting in `WFE`, regardless of which core produced the wakeup.
static LED_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// CPU1's EMIO LED, constructed on CPU0 and handed off through this static before CPU1 is
/// released via `start_core1`.
static EMIO_LED: Handoff<gpio::Output> = Handoff::new();

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

/// Entry point on CPU0, called by `zynq7000-rt`'s start-up code.
#[zynq7000_rt::entry]
fn main() -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config::default()).unwrap();
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    // Set up the global timer counter and embassy time driver. Both cores share this one
    // driver instance, but its alarm/queue state is tracked per-core internally, and each core
    // arms its own banked GTC comparator and interrupt.
    let gtc = gtc::GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_hal::time_driver_gtc::init(clocks.arm_clocks(), gtc);

    let mut gpio_pins = gpio::GpioPins::new(periphs.gpio);
    let mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);

    // Construct CPU1's LED here and hand it off through EMIO_LED. This has to happen before
    // start_core1 below.
    let emio_led = gpio::Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), gpio::PinState::Low);
    EMIO_LED.put(emio_led);

    let uart_clk_config = uart::ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::Uart::new_with_mio_for_uart_1(
        periphs.uart_1,
        uart::Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    uart.write_all(INIT_STRING.as_bytes());
    uart.flush();

    // Async logger: log calls append to a pipe under a critical section (a real cross-core
    // spinlock), so both cores can log through it. `log_task` below drains the pipe over the
    // UART on core0.
    let (tx, _rx) = uart.split();
    let log_runner = zynq7000_hal::log::asynch::init_with_uart_tx(
        LevelFilter::Trace,
        uart::TxAsync::new(tx, true),
    )
    .unwrap();

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    // Everything above touches SLCR/GPIO config, and we want to keep that on core0.
    zynq7000_rt::smp::start_core1();

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(core0_task(mio_led).unwrap());
        spawner.spawn(log_task(log_runner).unwrap());
    });
}

#[embassy_executor::task]
async fn core0_task(mut led: gpio::Output) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut count = 0;
    loop {
        led.toggle();
        info!("CPU0 heartbeat: {}", count);
        count += 1;
        LED_SIGNAL.signal(led.is_set_high());
        ticker.next().await;
    }
}

/// Drains the log pipe and writes it out over the UART.
#[embassy_executor::task]
async fn log_task(mut runner: zynq7000_hal::log::asynch::UartLoggerRunner) -> ! {
    runner.run().await
}

/// Entry point on CPU1, called by `zynq7000-rt`'s start-up code once CPU0 has released it via
/// [`zynq7000_rt::smp::start_core1`].
#[unsafe(no_mangle)]
pub extern "C" fn kmain_secondary() {
    zynq7000_hal::init_secondary_core(zynq7000_hal::SecondaryCoreConfig::default()).unwrap();

    let emio_led = EMIO_LED.take();

    let executor1 = EXECUTOR1.init(Executor::new());
    executor1.run(|spawner| {
        spawner.spawn(core1_led_task(emio_led).unwrap());
        spawner.spawn(core1_heartbeat_task().unwrap());
    });
}

/// Mirrors CPU0's MIO LED state on CPU1's EMIO LED, driven by the cross-core signal.
#[embassy_executor::task]
async fn core1_led_task(mut led: gpio::Output) -> ! {
    loop {
        match LED_SIGNAL.wait().await {
            true => led.set_high(),
            false => led.set_low(),
        }
    }
}

/// Runs entirely independently of CPU0, using CPU1's own banked GTC comparator and interrupt.
#[embassy_executor::task]
async fn core1_heartbeat_task() -> ! {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut count = 0u32;
    loop {
        info!("CPU1 heartbeat: {count}");
        count += 1;
        ticker.next().await;
    }
}
