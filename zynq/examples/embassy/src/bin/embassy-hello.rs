#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zynq7000_hal::{BootMode, InteruptConfig, clocks, gic, gpio, gtc, time::Hertz, uart};

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

#[embassy_executor::main]
#[unsafe(export_name = "main")]
async fn main(_spawner: Spawner) -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config {
        init_l2_cache: true,
        level_shifter_config: Some(zynq7000_hal::LevelShifterConfig::EnableAll),
        interrupt_config: Some(InteruptConfig::AllInterruptsToCpu0),
    })
    .unwrap();
    let clocks = clocks::Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    // Set up global timer counter and embassy time driver.
    let gtc = gtc::GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    let mio_pins = gpio::mio::Pins::new(periphs.gpio);

    // Set up the UART, we are logging with it.
    let uart_clk_config = uart::ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::Uart::new_with_mio(
        periphs.uart_1,
        uart::Config::new_with_clk_config(uart_clk_config),
        (mio_pins.mio48, mio_pins.mio49),
    )
    .unwrap();
    uart.write_all(b"-- Zynq 7000 Embassy Hello World --\n\r")
        .unwrap();
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

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led = gpio::Output::new_for_mio(mio_pins.mio7, gpio::PinState::Low);
    loop {
        info!("Hello, world!");
        led.toggle().unwrap();
        ticker.next().await;
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {
    let mut gic_helper = gic::GicInterruptHelper::new();
    let irq_info = gic_helper.acknowledge_interrupt();
    match irq_info.interrupt() {
        gic::Interrupt::Sgi(_) => (),
        gic::Interrupt::Ppi(ppi_interrupt) => {
            if ppi_interrupt == zynq7000_hal::gic::PpiInterrupt::GlobalTimer {
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
