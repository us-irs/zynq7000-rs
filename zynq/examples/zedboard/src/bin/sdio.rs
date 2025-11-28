#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::gpio::Input;
use zynq7000_hal::prelude::*;
use zynq7000_hal::{
    BootMode, clocks, gic, gpio, gtc,
    sdio::{Sdio, SdioClockConfig},
    uart,
};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard SDIO example --\n\r";

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

    let gpio_pins = gpio::GpioPins::new(periphs.gpio);

    // Set up global timer counter and embassy time driver.
    let gtc = gtc::GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

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
    uart.write_all(INIT_STRING.as_bytes()).unwrap();
    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    let sdio_clock_config =
        SdioClockConfig::calculate_for_io_clock(clocks.io_clocks(), 100.MHz(), 10.MHz()).unwrap();
    let sdio = Sdio::new_for_sdio_0(
        periphs.sdio_0,
        sdio_clock_config,
        gpio_pins.mio.mio40,
        gpio_pins.mio.mio41,
        (
            gpio_pins.mio.mio42,
            gpio_pins.mio.mio43,
            gpio_pins.mio.mio44,
            gpio_pins.mio.mio45,
        ),
    )
    .unwrap();
    let card_detect = Input::new_for_mio(gpio_pins.mio.mio47).unwrap();
    let write_protect = Input::new_for_mio(gpio_pins.mio.mio46).unwrap();
    // The card detect being active low makes sense according to the Zedboard docs. Not sure
    // about write-protect though.. It seems that write protect on means that the
    // the pin is pulled high.
    info!("Card detect state: {:?}", card_detect.is_low());
    info!("Write protect state: {:?}", write_protect.is_high());

    let capabilities = sdio.ll().capabilities();
    info!("SDIO Capabilities: {:?}", capabilities);

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let mut mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
    loop {
        mio_led.toggle().unwrap();

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
