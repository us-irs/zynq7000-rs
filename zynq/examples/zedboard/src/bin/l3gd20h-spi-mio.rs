//! PS SPI example using a L3GD20H sensor.
//!
//! External HW connections:
//!
//!  - SCK pin to JE4 (MIO 12)
//!  - MOSI pin to JE2 (MIO 10)
//!  - MISO pin to JE3 (MIO 11)
//!  - SS pin to JE1 (MIO 13)
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_io::Write;
use log::{error, info};
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    configure_level_shifter,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{GpioPins, Output, PinState},
    gtc::GlobalTimerCounter,
    l2_cache,
    spi::{self, SpiAsync, SpiId, SpiWithHwCs, SpiWithHwCsAsync, on_interrupt},
    time::Hertz,
    uart::{self, TxAsync, on_interrupt_tx},
};

use zynq7000::{Peripherals, slcr::LevelShifterConfig, spi::DelayControl};
use zynq7000_rt as _;

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);

const DEBUG_SPI_CLK_CONFIG: bool = false;
const BLOCKING: bool = false;

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
async fn main(spawner: Spawner) -> ! {
    let mut dp = Peripherals::take().unwrap();
    l2_cache::init_with_defaults(&mut dp.l2c);

    // Enable PS-PL level shifters.
    configure_level_shifter(LevelShifterConfig::EnableAll);

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let mut clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    // SPI reference clock must be larger than the CPU 1x clock.
    let spi_ref_clk_div = spi::calculate_largest_allowed_spi_ref_clk_divisor(&clocks)
        .unwrap()
        .value()
        - 1;
    spi::configure_spi_ref_clk(&mut clocks, arbitrary_int::u6::new(spi_ref_clk_div as u8));

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
    let gtc = GlobalTimerCounter::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    // Set up the UART, we are logging with it.
    let uart_clk_config = uart::ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::Uart::new_with_mio_for_uart_1(
        dp.uart_1,
        uart::Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    uart.write_all(b"-- Zynq 7000 Zedboard SPI L3GD20H example --\n\r")
        .unwrap();
    zynq7000_hal::log::rb::init(log::LevelFilter::Trace);

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    if DEBUG_SPI_CLK_CONFIG {
        info!(
            "SPI Clock Information: CPU 1x: {:?}, IO Ref Clk: {:?}, SPI Ref Clk: {:?}, DIV: {:?}",
            clocks.arm_clocks().cpu_1x_clk(),
            clocks.io_clocks().ref_clk(),
            clocks.io_clocks().spi_clk(),
            spi_ref_clk_div
        );
    }

    let mut spi = spi::Spi::new_one_hw_cs(
        dp.spi_1,
        clocks.io_clocks(),
        spi::Config::new(
            // 10 MHz maximum rating of the sensor.
            zynq7000::spi::BaudDivSel::By64,
            //l3gd20::MODE,
            embedded_hal::spi::MODE_3,
            spi::SlaveSelectConfig::AutoWithAutoStart,
        ),
        (
            gpio_pins.mio.mio12,
            gpio_pins.mio.mio10,
            gpio_pins.mio.mio11,
        ),
        gpio_pins.mio.mio13,
    )
    .unwrap();
    let mod_id = spi.regs().read_mod_id();
    assert_eq!(mod_id, spi::MODULE_ID);
    assert!(spi.sclk() <= Hertz::from_raw(10_000_000));
    let min_delay = (spi.sclk().raw() * 5) / 1_000_000_000;
    spi.inner().configure_delays(
        DelayControl::builder()
            .with_inter_word_cs_deassert(0)
            .with_between_cs_assertion(0)
            .with_inter_word(0)
            .with_cs_to_first_bit(min_delay as u8)
            .build(),
    );

    let mio_led = Output::new_for_mio(gpio_pins.mio.mio7, PinState::Low);
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
        if idx.is_multiple_of(2) {
            led.set_high();
        } else {
            led.set_low();
        }
    }

    spawner.spawn(logger_task(uart)).unwrap();
    if BLOCKING {
        blocking_application(mio_led, emio_leds, spi).await;
    } else {
        non_blocking_application(mio_led, emio_leds, spi).await;
    }
}

#[embassy_executor::task]
pub async fn logger_task(uart: uart::Uart) {
    let (tx, _) = uart.split();
    let mut tx_async = TxAsync::new(tx);
    let frame_queue = zynq7000_hal::log::rb::get_frame_queue();
    let mut log_buf: [u8; 2048] = [0; 2048];
    loop {
        let next_frame_len = frame_queue.receive().await;
        zynq7000_hal::log::rb::read_next_frame(next_frame_len, &mut log_buf);
        tx_async.write(&log_buf[0..next_frame_len]).await;
    }
}

pub async fn blocking_application(
    mut mio_led: Output,
    mut emio_leds: [Output; 8],
    spi: spi::Spi,
) -> ! {
    let mut delay = Delay;
    let spi_dev = SpiWithHwCs::new(spi, spi::ChipSelect::Slave0, delay.clone());
    let mut l3gd20 = l3gd20::spi::L3gd20::new(spi_dev).unwrap();
    let who_am_i = l3gd20.who_am_i().unwrap();
    info!("L3GD20 WHO_AM_I: 0x{who_am_i:02X}");

    let mut ticker = Ticker::every(Duration::from_millis(400));

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

pub async fn non_blocking_application(
    mut mio_led: Output,
    mut emio_leds: [Output; 8],
    spi: spi::Spi,
) -> ! {
    let mut delay = Delay;
    let spi_async = SpiAsync::new(spi);
    let spi_dev = SpiWithHwCsAsync::new(spi_async, spi::ChipSelect::Slave0, delay.clone());
    let mut l3gd20 = l3gd20::asynchronous::spi::L3gd20::new(spi_dev)
        .await
        .unwrap();
    let who_am_i = l3gd20.who_am_i().await.unwrap();
    info!("L3GD20 WHO_AM_I: 0x{who_am_i:02X}");

    let mut ticker = Ticker::every(Duration::from_millis(400));

    // Power up time for the sensor to get good readings.
    delay.delay_ms(50).await;
    loop {
        mio_led.toggle().unwrap();

        let measurements = l3gd20.all().await.unwrap();
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
        Interrupt::Spi(spi_interrupt) => {
            if spi_interrupt == zynq7000_hal::gic::SpiInterrupt::Spi1 {
                on_interrupt(SpiId::Spi1);
            } else if spi_interrupt == zynq7000_hal::gic::SpiInterrupt::Uart1 {
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
