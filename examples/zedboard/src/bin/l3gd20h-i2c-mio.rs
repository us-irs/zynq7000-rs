//! PS I2C example using a L3GD20H sensor.
//!
//! External HW connections:
//!
//!  - SCK pin to JE4 (MIO 12)
//!  - SDA pin to JE1 (MIO 13)
//!  - SDO / SA0 pin to JE3 (MIO 11) to select I2C address.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_io::Write;
use l3gd20::i2c::I2cAddr;
use log::{error, info};
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    configure_level_shifter,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{GpioPins, Output, PinState},
    gtc::Gtc,
    i2c,
    time::Hertz,
    uart,
};

use zynq7000::{PsPeripherals, slcr::LevelShifterConfig};
use zynq7000_rt as _;

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);
const I2C_ADDR_SEL: I2cAddr = I2cAddr::Sa0Low;

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
    // Enable PS-PL level shifters.
    configure_level_shifter(LevelShifterConfig::EnableAll);
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

    let mut gpio_pins = GpioPins::new(dp.gpio);

    // Set up global timer counter and embassy time driver.
    let gtc = Gtc::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    // Set up the UART, we are logging with it.
    let uart_clk_config = uart::ClkConfigRaw::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::Uart::new_with_mio(
        dp.uart_1,
        uart::UartConfig::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    uart.write_all(b"-- Zynq 7000 Zedboard I2C L3GD20H example --\n\r")
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

    let pin_sel = match I2C_ADDR_SEL {
        I2cAddr::Sa0Low => PinState::Low,
        I2cAddr::Sa0High => PinState::High,
    };
    let _sa0_pin = Output::new_for_mio(gpio_pins.mio.mio11, pin_sel);
    // The CS pin must be pulled high.
    let _cs_pin = Output::new_for_mio(gpio_pins.mio.mio10, PinState::High);

    let clk_config = i2c::calculate_divisors(
        clocks.arm_clocks().cpu_1x_clk(),
        i2c::I2cSpeed::Normal100kHz,
    )
    .unwrap();
    let i2c = i2c::I2c::new_with_mio(
        dp.i2c_1,
        clk_config,
        (gpio_pins.mio.mio12, gpio_pins.mio.mio13),
    )
    .unwrap();
    let mut l3gd20 = l3gd20::i2c::L3gd20::new(i2c, l3gd20::i2c::I2cAddr::Sa0Low).unwrap();
    let who_am_i = l3gd20.who_am_i().unwrap();
    info!("L3GD20 WHO_AM_I: 0x{:02X}", who_am_i);

    let mut delay = Delay;
    let mut ticker = Ticker::every(Duration::from_millis(400));
    let mut mio_led = Output::new_for_mio(gpio_pins.mio.mio7, PinState::Low);

    let mut emio_leds: [Output; 8] = [
        Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(1).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(2).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(3).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(4).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(5).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(6).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(7).unwrap(), PinState::Low),
    ];
    for (idx, led) in emio_leds.iter_mut().enumerate() {
        if idx % 2 == 0 {
            led.set_high();
        } else {
            led.set_low();
        }
    }

    // Power up time for the sensor to get good readings.
    delay.delay_ms(50).await;
    loop {
        mio_led.toggle().unwrap();

        let measurements = l3gd20.all().unwrap();
        info!("L3GD20: {measurements:?}");
        info!("L3GD20 Temp: {:?}", measurements.temp_celcius());
        for led in emio_leds.iter_mut() {
            led.toggle().unwrap();
        }

        ticker.next().await; // Wait for the next cycle of the ticker
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
    error!("Panic: {info:?}");
    loop {}
}
