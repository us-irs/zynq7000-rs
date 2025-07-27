//! Example which uses the UART1 to send log messages.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zynq7000::PsPeripherals;
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{Output, PinState, mio},
    gtc::Gtc,
    l2_cache,
    time::Hertz,
    uart::{ClkConfigRaw, TxAsync, Uart, UartConfig, on_interrupt_tx},
};

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

#[unsafe(export_name = "main")]
#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut dp = PsPeripherals::take().unwrap();
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
    // Set up global timer counter and embassy time driver.
    let gtc = Gtc::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    let mio_pins = mio::Pins::new(dp.gpio);

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
    uart.write_all(b"-- Zynq 7000 Logging example --\n\r")
        .unwrap();
    uart.flush().unwrap();
    let (tx, _rx) = uart.split();
    let mut logger = TxAsync::new(tx);

    zynq7000_hal::log::rb::init(log::LevelFilter::Trace);

    let boot_mode = BootMode::new();
    info!("Boot mode: {:?}", boot_mode);

    let led = Output::new_for_mio(mio_pins.mio7, PinState::Low);
    spawner.spawn(led_task(led)).unwrap();
    let mut log_buf: [u8; 2048] = [0; 2048];
    let frame_queue = zynq7000_hal::log::rb::get_frame_queue();
    loop {
        let next_frame_len = frame_queue.receive().await;
        zynq7000_hal::log::rb::read_next_frame(next_frame_len, &mut log_buf);
        logger.write(&log_buf[0..next_frame_len]).await;
    }
}

#[embassy_executor::task]
async fn led_task(mut mio_led: Output) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        mio_led.toggle().unwrap();
        info!("Toggling LED");
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
        Interrupt::Spi(spi_interrupt) => {
            if spi_interrupt == zynq7000_hal::gic::SpiInterrupt::Uart1 {
                on_interrupt_tx(zynq7000_hal::uart::UartId::Uart1);
            }
        }
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
    error!("Panic: {info:?}");
    loop {}
}
