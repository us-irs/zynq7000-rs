//! AXI GPIO async/interrupt-driven example.
//!
//! Exercises `axi-gpio`'s `asynch` module: channel 2's interrupt fires on any change of SW7
//! (the only input on that channel, see `zedboard-gateware/README.md`'s board resource table),
//! and `AsyncChannel::wait` resolves once `on_interrupt` acknowledges it from the ISR. Channel 1
//! drives LD6/LD7 on a separate task, just as a "this example is alive" indicator, independent
//! of the async path being tested.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{
    BootMode, Interrupt, SpiInterrupt, clocks, generic_interrupt_handler, gpio, gtc, uart,
};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard AXI GPIO async example --\n\r";

const AXI_GPIO_BASE_ADDR: u32 = 0x4120_0000;
const AXI_GPIO_INTERRUPT: SpiInterrupt = SpiInterrupt::Pl4;
const AXI_GPIO_WAKER_IDX: usize = 0;

static CHANNEL2_TOKEN: once_cell::sync::OnceCell<axi_gpio::asynch::ChannelToken> =
    once_cell::sync::OnceCell::new();

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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
    let (mut axi_gpio, axi_gpio_ch1_pins, axi_gpio_ch2_pins) =
        unsafe { axi_gpio::AxiGpio::new_dual_channel(AXI_GPIO_BASE_ADDR) };
    let led6 = axi_gpio.output_pin(axi_gpio_ch1_pins.p0, gpio::PinState::Low);
    let led7 = axi_gpio.output_pin(axi_gpio_ch1_pins.p1, gpio::PinState::High);
    spawner.spawn(led_task(led6, led7).unwrap());

    // Configures SW7 as an input. Not strictly needed given C_ALL_INPUTS_2 defaults channel 2 to
    // all-input at reset, but the async path doesn't touch tri-state itself, so this is what
    // actually establishes that.
    let _switch_7 = axi_gpio.input_pin(axi_gpio_ch2_pins.p0);

    let mut channel2_events = axi_gpio
        .event_channel(axi_gpio::ChannelId::Ch2, AXI_GPIO_WAKER_IDX)
        .expect("waker index unexpectedly invalid")
        .expect("channel 2 events already taken");

    // Register the ISR and publish the token before enabling the global interrupt, so it can't
    // fire with the token not yet set. `channel2_events` enables/disables channel 2's own
    // interrupt itself, for the lifetime of each `wait()` call, see `AsyncChannel`'s docs.
    zynq7000_hal::register_interrupt(Interrupt::Spi(AXI_GPIO_INTERRUPT), axi_gpio_interrupt);
    CHANNEL2_TOKEN
        .set(channel2_events.token())
        .expect("initializing channel 2 token failed");
    axi_gpio.enable_global_interrupt();

    info!("Waiting for channel 2 events (toggle SW7)...");
    loop {
        let snapshot = channel2_events.wait().await;
        info!("channel 2 event, SW7 = {}", snapshot.pins(0));
    }
}

#[embassy_executor::task]
async fn led_task(mut led6: axi_gpio::Output, mut led7: axi_gpio::Output) {
    let mut ticker = Ticker::every(Duration::from_millis(200));
    loop {
        led6.toggle();
        led7.toggle();
        ticker.next().await;
    }
}

pub fn axi_gpio_interrupt() {
    if let Some(token) = CHANNEL2_TOKEN.get() {
        unsafe { axi_gpio::asynch::on_interrupt(token) };
    }
}

#[zynq7000_rt::irq]
pub fn irq_handler() {
    // Safety: Called here once.
    let result = unsafe { generic_interrupt_handler() };
    if let Err(e) = result
        && e != Interrupt::Spurious
    {
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
