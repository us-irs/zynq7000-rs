#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{BootMode, clocks, generic_interrupt_handler, gpio, gtc, uart};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard GPIO blinky example --\n\r";

const AXI_GPIO_BASE_ADDR: u32 = 0x4120_0000;

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config::default()).unwrap();
    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = clocks::Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    let mut gpio_pins = gpio::Pins::new(periphs.gpio, gpio::Config::default());

    // Set up global timer counter and embassy time driver.
    let gtc = gtc::GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_hal::time_driver_gtc::init(clocks.arm_clocks(), gtc);

    // Set up the UART, we are logging with it.
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

    zynq7000_hal::log::uart_blocking::init_with_busy_flag(uart, log::LevelFilter::Trace, false);

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let mut mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
    // LD0-LD5 are driven by EMIO, LD6/LD7 by AXI GPIO channel 1, see
    // zedboard-gateware/README.md's board resource table.
    let mut emio_leds: [gpio::Output; 6] = [
        gpio::Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(1).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(2).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(3).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(4).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(5).unwrap(), gpio::PinState::Low),
    ];
    let (axi_gpio, axi_gpio_pins) =
        unsafe { axi_gpio::AxiGpio::new_single_channel(AXI_GPIO_BASE_ADDR) };
    let mut axi_gpio_leds: [axi_gpio::Output; 2] = [
        axi_gpio.output_pin(axi_gpio_pins.p0, gpio::PinState::Low),
        axi_gpio.output_pin(axi_gpio_pins.p1, gpio::PinState::Low),
    ];
    loop {
        mio_led.toggle();

        // Create a wave pattern across LD0-LD5 (EMIO), continuing into LD6/LD7 (AXI GPIO).
        for led in emio_leds.iter_mut() {
            led.toggle();
            ticker.next().await; // Wait for the next ticker for each toggle
        }
        for led in axi_gpio_leds.iter_mut() {
            led.toggle();
            ticker.next().await;
        }

        ticker.next().await; // Wait for the next cycle of the ticker
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
