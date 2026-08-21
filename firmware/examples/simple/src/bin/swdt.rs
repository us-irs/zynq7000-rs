//! Simple example which feeds the system watchdog timer (SWDT) periodically.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::{
    panic::PanicInfo,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use embedded_io::Write;
use log::info;
use zynq7000_hal::{
    clocks::Clocks,
    gpio::{Output, PinState, mio},
    priv_tim::CpuPrivateTimer,
    swdt,
    time::Hertz,
    uart,
};

/// False by default, because we want to show how the watchdog priodically elapsed.
const RESET_ON_EXPIRY: bool = false;
const WATCHDOG_PERIOD_MS: u32 = 2000;

const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_333);

/// Set by the SWDT ISR on expiry, and consumed (cleared) by the main loop, which does the
/// actual logging outside of interrupt context.
static EXPIRED: AtomicBool = AtomicBool::new(false);

/// Number of main loop iterations (each [`LOOP_DELAY_MS`] long) since the last expiry. A rough
/// stand-in for a real elapsed-time measurement, just to sanity-check the configured timeout.
static LOOP_COUNT: AtomicU32 = AtomicU32::new(0);

/// Main loop poll period.
const LOOP_DELAY_MS: u32 = 50;

const INIT_STRING: &str = "-- Zynq 7000 SWDT example --\n\r";

#[zynq7000_rt::entry]
fn main() -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config::default()).expect("HAL init failed");
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    // Unwrap okay, we only call this once on core 0 here.
    let mut cpu_tim = CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();
    let mio_pins = mio::Pins::new(periphs.gpio);
    let mut led = Output::new_for_mio(mio_pins.mio7, PinState::High);

    // Set up the UART and a blocking logger. The ISR itself only sets `EXPIRED`; the actual
    // logging happens down in the main loop, outside of interrupt context.
    let uart_clk_config = uart::ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::Uart::new_with_mio_for_uart_1(
        periphs.uart_1,
        uart::Config::new_with_clk_config(uart_clk_config),
        (mio_pins.mio48, mio_pins.mio49),
    )
    .unwrap();
    uart.write_all(INIT_STRING.as_bytes());

    zynq7000_hal::log::uart_blocking::init_with_locks(uart, log::LevelFilter::Trace);

    zynq7000_hal::register_interrupt(
        zynq7000_hal::Interrupt::Spi(zynq7000_hal::SpiInterrupt::Swdt),
        watchdog_interrupt,
    );

    // Use the internal CPU 1x clock and route the reset output only to EMIO, so no MIO pin or
    // PL design is required for this example to run. The IRQ (not the reset output) is enabled,
    // so it re-feeds itself from the ISR periodically.
    swdt::SystemWatchdog::new_with_cpu1x_clk_emio_reset(
        periphs.swdt,
        swdt::Config {
            irq_enable: true,
            reset_enable: RESET_ON_EXPIRY,
            clk_config: swdt::ClockConfig::calculate_cpu1x(clocks.arm_clocks(), WATCHDOG_PERIOD_MS),
            ..Default::default()
        },
    );

    loop {
        // Watchdog expiration is signalled by interrupt handler.
        if EXPIRED.swap(false, Ordering::Relaxed) {
            led.toggle();
            let loops = LOOP_COUNT.swap(0, Ordering::Relaxed);
            info!(
                "SWDT expired, ~{} ms since last expiry",
                loops * LOOP_DELAY_MS
            );
        }
        LOOP_COUNT.fetch_add(1, Ordering::Relaxed);
        cpu_tim.delay_ms(LOOP_DELAY_MS);
    }
}

fn watchdog_interrupt() {
    // Safety: The watchdog was already configured in `main`, we only feed it here and check for expiry.
    let mut wdt =
        swdt::SystemWatchdog::new_unchecked(unsafe { zynq7000::swdt::Registers::new_mmio_fixed() });
    if wdt.expired() {
        EXPIRED.store(true, Ordering::Relaxed);
    }
    wdt.feed();
}

#[zynq7000_rt::irq]
fn irq_handler() {
    // Safety: Called here once.
    let result = unsafe { zynq7000_hal::generic_interrupt_handler() };
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
    loop {
        nop();
    }
}
