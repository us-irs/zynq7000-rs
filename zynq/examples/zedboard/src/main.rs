#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{BootMode, clocks, gic, gpio, gtc, uart};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard GPIO blinky example --\n\r";

/// Entry point (not called like a normal main function)
#[unsafe(no_mangle)]
pub extern "C" fn boot_core(cpu_id: u32) -> ! {
    if cpu_id != 0 {
        panic!("unexpected CPU ID {}", cpu_id);
    }
    main();
}

#[embassy_executor::main]
#[unsafe(export_name = "main")]
async fn main(_spawner: Spawner) -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config {
        init_l2_cache: true,
        level_shifter_config: Some(zynq7000_hal::LevelShifterConfig::EnableAll),
        interrupt_config: Some(zynq7000_hal::InteruptConfig::AllInterruptsToCpu0),
    })
    .unwrap();
    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = clocks::Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    let mut gpio_pins = gpio::GpioPins::new(periphs.gpio);

    // Set up global timer counter and embassy time driver.
    let gtc = gtc::GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    // Set up the UART, we are logging with it.
    let uart_clk_config = uart::ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::Uart::new_with_mio(
        periphs.uart_1,
        uart::Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    uart.write_all(INIT_STRING.as_bytes()).unwrap();
    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let mut mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
    let mut emio_leds: [gpio::Output; 8] = [
        gpio::Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(1).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(2).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(3).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(4).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(5).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(6).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(7).unwrap(), gpio::PinState::Low),
    ];
    loop {
        mio_led.toggle().unwrap();

        // Create a wave pattern for emio_leds
        for led in emio_leds.iter_mut() {
            led.toggle().unwrap();
            ticker.next().await; // Wait for the next ticker for each toggle
        }

        ticker.next().await; // Wait for the next cycle of the ticker
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {
    let mut gic_helper = gic::GicInterruptHelper::new();
    let irq_info = gic_helper.acknowledge_interrupt();
    match irq_info.interrupt() {
        gic::Interrupt::Sgi(_) => (),
        gic::Interrupt::Ppi(ppi_interrupt) => {
            if ppi_interrupt == gic::PpiInterrupt::GlobalTimer {
                unsafe {
                    zynq7000_embassy::on_interrupt();
                }
            }
        }
        gic::Interrupt::Spi(_spi_interrupt) => (),
        gic::Interrupt::Invalid(_) => (),
        gic::Interrupt::Spurious => (),
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
