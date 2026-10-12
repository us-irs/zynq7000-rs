//! Asynchronous GPIO example using the five Zedboard push buttons.
//!
//! Every button uses a different part of the asynchronous GPIO API:
//!
//! | Button | API | LED behaviour |
//! |---|---|---|
//! | BTNC | `wait_for_rising_edge` | Toggles LD0 on press |
//! | BTNL | `wait_for_falling_edge` | Toggles LD1 on release |
//! | BTNR | `wait_for_high` and `wait_for_low` | LD2 is on while the button is pressed |
//! | BTNU | `wait_for_rising_edge` | Counts up, LD4 to LD7 show the count in binary |
//! | BTND | `wait_for_any_edge` | Counts down on release |
//!
//! LD3 blinks as a heartbeat.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::digital::OutputPin;
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{BootMode, clocks, generic_interrupt_handler, gpio, gtc, uart};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard asynchronous buttons example --\n\r";

const AXI_GPIO_BASE_ADDR: u32 = 0x4120_0000;

// EMIO offsets of the buttons, see the board resource table in zedboard-gateware/README.md.
const BTNC: usize = 24;
const BTND: usize = 25;
const BTNL: usize = 26;
const BTNR: usize = 27;
const BTNU: usize = 28;

/// Mechanical buttons bounce, so edges shortly after a press or release are ignored.
const DEBOUNCE_TIME: Duration = Duration::from_millis(20);

#[derive(Debug, Clone, Copy)]
enum CounterEvent {
    Up,
    Down,
}

static COUNTER_EVENTS: Channel<CriticalSectionRawMutex, CounterEvent, 4> = Channel::new();

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

    // The default configuration registers the GPIO interrupt handler required for the
    // asynchronous API.
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

    let mut button = |offset| {
        gpio::Input::new_for_emio(gpio_pins.emio.take(offset).unwrap())
            .unwrap()
            .into_async()
    };
    let btnc = button(BTNC);
    let btnd = button(BTND);
    let btnl = button(BTNL);
    let btnr = button(BTNR);
    let btnu = button(BTNU);

    // LD0-LD5 are driven by EMIO, LD6/LD7 by AXI GPIO channel 1.
    let mut emio_led = |offset| {
        gpio::Output::new_for_emio(gpio_pins.emio.take(offset).unwrap(), gpio::PinState::Low)
    };
    let ld0 = emio_led(0);
    let ld1 = emio_led(1);
    let ld2 = emio_led(2);
    let mut ld3 = emio_led(3);
    let ld4 = emio_led(4);
    let ld5 = emio_led(5);
    // SAFETY: The AXI GPIO block is only used here.
    let (axi_gpio, axi_gpio_pins) =
        unsafe { axi_gpio::AxiGpio::new_single_channel(AXI_GPIO_BASE_ADDR) };
    let ld6 = axi_gpio.output_pin(axi_gpio_pins.p0, gpio::PinState::Low);
    let ld7 = axi_gpio.output_pin(axi_gpio_pins.p1, gpio::PinState::Low);

    spawner.spawn(toggle_on_press_task(btnc, ld0).unwrap());
    spawner.spawn(toggle_on_release_task(btnl, ld1).unwrap());
    spawner.spawn(on_while_pressed_task(btnr, ld2).unwrap());
    spawner.spawn(count_up_task(btnu).unwrap());
    spawner.spawn(count_down_task(btnd).unwrap());
    spawner.spawn(counter_task([ld4, ld5], [ld6, ld7]).unwrap());

    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        ld3.toggle();
        ticker.next().await;
    }
}

async fn debounce() {
    Timer::after(DEBOUNCE_TIME).await;
}

#[embassy_executor::task]
async fn toggle_on_press_task(mut button: gpio::asynch::Input, mut led: gpio::Output) {
    loop {
        button.wait_for_rising_edge().await;
        led.toggle();
        info!("BTNC pressed, toggled LD0");
        debounce().await;
        button.wait_for_low().await;
        debounce().await;
    }
}

#[embassy_executor::task]
async fn toggle_on_release_task(mut button: gpio::asynch::Input, mut led: gpio::Output) {
    loop {
        button.wait_for_high().await;
        debounce().await;
        button.wait_for_falling_edge().await;
        led.toggle();
        info!("BTNL released, toggled LD1");
        debounce().await;
    }
}

#[embassy_executor::task]
async fn on_while_pressed_task(mut button: gpio::asynch::Input, mut led: gpio::Output) {
    loop {
        button.wait_for_high().await;
        led.set_high();
        info!("BTNR pressed, LD2 on");
        debounce().await;
        button.wait_for_low().await;
        led.set_low();
        info!("BTNR released, LD2 off");
        debounce().await;
    }
}

#[embassy_executor::task]
async fn count_up_task(mut button: gpio::asynch::Input) {
    loop {
        button.wait_for_rising_edge().await;
        COUNTER_EVENTS.send(CounterEvent::Up).await;
        debounce().await;
        button.wait_for_low().await;
        debounce().await;
    }
}

#[embassy_executor::task]
async fn count_down_task(mut button: gpio::asynch::Input) {
    let mut pressed = button.is_high();
    loop {
        button.wait_for_any_edge().await;
        debounce().await;
        // Only the settled level after the debounce time counts, which filters out bounces.
        let now_pressed = button.is_high();
        if pressed && !now_pressed {
            COUNTER_EVENTS.send(CounterEvent::Down).await;
        }
        pressed = now_pressed;
    }
}

/// Shows a 4 bit counter on LD4 to LD7.
#[embassy_executor::task]
async fn counter_task(mut emio_leds: [gpio::Output; 2], mut axi_gpio_leds: [axi_gpio::Output; 2]) {
    let mut count: u8 = 0;
    loop {
        count = match COUNTER_EVENTS.receive().await {
            CounterEvent::Up => (count + 1) % 16,
            CounterEvent::Down => (count + 15) % 16,
        };
        info!("Counter: {count}");
        let bit = |n: u8| gpio::PinState::from(count & (1 << n) != 0);
        emio_leds[0].set_state(bit(0)).unwrap();
        emio_leds[1].set_state(bit(1)).unwrap();
        axi_gpio_leds[0].set_state(bit(2)).unwrap();
        axi_gpio_leds[1].set_state(bit(3)).unwrap();
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
