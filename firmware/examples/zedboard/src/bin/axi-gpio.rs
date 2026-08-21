//! AXI GPIO example.
//!
//! Exercises the `axi-gpio` crate directly, independent of the MIO/EMIO GPIO controller: channel
//! 1 drives LD6/LD7 (output), channel 2 reads SW7 (input). See
//! `zedboard-gateware/README.md`'s board resource table for the pin assignment.
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

const INIT_STRING: &str = "-- Zynq 7000 Zedboard AXI GPIO example --\n\r";

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

    let gpio_pins = gpio::GpioPins::new(periphs.gpio);

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

    // Channel 1 drives LD6/LD7, channel 2 reads SW7, see zedboard-gateware/README.md's board
    // resource table.
    let (axi_gpio, axi_gpio_ch1_pins, axi_gpio_ch2_pins) =
        unsafe { axi_gpio::AxiGpio::new_dual_channel(AXI_GPIO_BASE_ADDR) };
    let mut led6 = axi_gpio.output_pin(axi_gpio_ch1_pins.p0, gpio::PinState::Low);
    let mut led7 = axi_gpio.output_pin(axi_gpio_ch1_pins.p1, gpio::PinState::High);
    let switch_7 = axi_gpio.input_pin(axi_gpio_ch2_pins.p0);

    let mut last_switch_state = switch_7.is_high();
    info!("SW7 initial state: {}", last_switch_state);

    let mut ticker = Ticker::every(Duration::from_millis(200));
    loop {
        // Alternate LD6/LD7 to show AXI GPIO channel 1 output is working.
        led6.toggle();
        led7.toggle();

        // Poll SW7 (AXI GPIO channel 2 input) and log on change.
        let switch_state = switch_7.is_high();
        if switch_state != last_switch_state {
            info!("SW7 changed: {}", switch_state);
            last_switch_state = switch_state;
        }

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
    error!("Panic: {info:?}");
    loop {}
}
