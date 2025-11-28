//! Open-drain pin mode examples
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal::{delay::DelayNs, digital::StatefulOutputPin};
use embedded_io::Write;
use log::{error, info, warn};
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{Flex, Output, PinState, mio},
    gtc::GlobalTimerCounter,
    l2_cache,
    time::Hertz,
    uart::{ClockConfig, Config, Uart},
};

use zynq7000::Peripherals;
use zynq7000_rt as _;

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);

/// Entry point (not called like a normal main function)
#[unsafe(no_mangle)]
pub extern "C" fn boot_core(cpu_id: u32) -> ! {
    if cpu_id != 0 {
        panic!("unexpected CPU ID {}", cpu_id);
    }
    main();
}

/// Try to talk to a DHT22 sensor connected at MIO0.
const DHT22_AT_MIO0: bool = true;

/// Open drain pin testing. MIO9 needs to be tied to MIO14.
const OPEN_DRAIN_PINS_MIO9_TO_MIO14: bool = false;

#[embassy_executor::main]
#[unsafe(export_name = "main")]
async fn main(_spawner: Spawner) -> ! {
    let mut dp = Peripherals::take().unwrap();
    l2_cache::init_with_defaults(&mut dp.l2c);

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    // Set up the global interrupt controller.
    let mut gic = GicConfigurator::new_with_init(dp.gicc, dp.gicd);
    gic.enable_all_interrupts();
    gic.set_all_spi_interrupt_targets_cpu0();
    gic.enable();
    unsafe {
        gic.enable_interrupts();
    }
    let mio_pins = mio::Pins::new(dp.gpio);

    // Set up global timer counter and embassy time driver.
    let gtc = GlobalTimerCounter::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

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
    uart.write_all(b"-- Zynq 7000 DHT22 --\n\r").unwrap();

    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    let mut delay = Delay;

    let mut one_wire_pin = Flex::new_for_mio(mio_pins.mio0);
    one_wire_pin.configure_as_output_open_drain(PinState::High, true);

    if OPEN_DRAIN_PINS_MIO9_TO_MIO14 {
        let mut flex_pin_0 = Flex::new_for_mio(mio_pins.mio9);
        flex_pin_0.configure_as_output_open_drain(PinState::High, true);
        let mut flex_pin_1 = Flex::new_for_mio(mio_pins.mio14);
        flex_pin_1.configure_as_input_floating().unwrap();
        // Should be high because of pull up.
        info!(
            "Flex Pin 1 state (should be high): {}",
            flex_pin_1.is_high()
        );
        info!(
            "Flex Pin 0 state (should be high): {}",
            flex_pin_0.is_high()
        );
        flex_pin_0.set_low();
        info!("Flex Pin 1 state (should be low): {}", flex_pin_1.is_high());
        info!("Flex Pin 0 state (should be low): {}", flex_pin_0.is_high());
        flex_pin_0.set_high();
        delay.delay_us(5);
        info!(
            "Flex Pin 1 state (should be high): {}",
            flex_pin_1.is_high()
        );
        info!(
            "Flex Pin 0 state (should be high): {}",
            flex_pin_0.is_high()
        );

        flex_pin_1.configure_as_output_open_drain(PinState::Low, true);
        info!("Flex Pin 1 state (should be low): {}", flex_pin_1.is_high());
        info!("Flex Pin 0 state (should be low): {}", flex_pin_0.is_high());

        flex_pin_1.set_high();
        delay.delay_us(5);
        info!(
            "Flex Pin 1 state (should be high): {}",
            flex_pin_1.is_high()
        );
        info!(
            "Flex Pin 0 state (should be high): {}",
            flex_pin_0.is_high()
        );

        flex_pin_1.set_low();
        info!("Flex Pin 1 state (should be low): {}", flex_pin_1.is_high());
        info!("Flex Pin 0 state (should be low): {}", flex_pin_0.is_high());
    }

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led = Output::new_for_mio(mio_pins.mio7, PinState::Low);
    loop {
        if DHT22_AT_MIO0 {
            let result = dht_sensor::dht22::r#async::read(&mut delay, &mut one_wire_pin).await;
            match result {
                Ok(reading) => {
                    info!("Temperature: {} C", reading.temperature);
                    info!("Humidity: {} %", reading.relative_humidity);
                }
                Err(err) => {
                    warn!("Reading error: {err:?}");
                }
            }
        }
        led.toggle().unwrap();

        ticker.next().await;
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {
    let mut gic_helper = GicInterruptHelper::new();
    let irq_info = gic_helper.acknowledge_interrupt();
    match irq_info.interrupt() {
        Interrupt::Sgi(_) => (),
        Interrupt::Ppi(ppi_interrupt) => {
            if ppi_interrupt == zynq7000_hal::gic::PpiInterrupt::GlobalTimer {
                unsafe {
                    zynq7000_embassy::on_interrupt();
                }
            }
        }
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
