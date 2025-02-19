//! PWM example which uses a PWM pin routed through EMIO.
//!
//! This example puts the PWM output on the EMIO channel of TTC0 channel 0.
//!
//! On the Zedboard, the PWM waveform output will be on the W12 pin of PMOD JB1. The Zedboard
//! reference FPGA design must be flashed onto the Zedboard for this to work.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::{digital::StatefulOutputPin, pwm::SetDutyCycle};
use embedded_io::Write;
use fugit::RateExtU32;
use log::{error, info};
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{Output, PinState, mio},
    gtc::Gtc,
    time::Hertz,
    uart::{ClkConfigRaw, Uart, UartConfig},
};

use zynq7000::PsPeripherals;
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
    let dp = PsPeripherals::take().unwrap();
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
    let gtc = Gtc::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    // Unwrap is okay, the address is definitely valid.
    let ttc_0 = zynq7000_hal::ttc::Ttc::new(dp.ttc_0).unwrap();
    let mut pwm =
        zynq7000_hal::ttc::Pwm::new_with_cpu_clk(ttc_0.ch0, clocks.arm_clocks(), 1000.Hz())
            .unwrap();
    pwm.set_duty_cycle_percent(50).unwrap();

    // Set up the UART, we are logging with it.
    let uart_clk_config = ClkConfigRaw::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = Uart::new_with_mio(
        dp.uart_1,
        UartConfig::new_with_clk_config(uart_clk_config),
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

    let boot_mode = BootMode::new();
    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led = Output::new_for_mio(mio_pins.mio7, PinState::Low);
    let mut current_duty = 0;
    loop {
        led.toggle().unwrap();

        pwm.set_duty_cycle_percent(current_duty).unwrap();
        info!("Setting duty cycle to {}%", current_duty);
        current_duty += 5;
        if current_duty > 100 {
            current_duty = 0;
        }

        ticker.next().await;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _irq_handler() {
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

#[unsafe(no_mangle)]
pub extern "C" fn _abort_handler() {
    loop {
        nop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _undefined_handler() {
    loop {
        nop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn _prefetch_handler() {
    loop {
        nop();
    }
}

/// Panic handler
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {:?}", info);
    loop {}
}
