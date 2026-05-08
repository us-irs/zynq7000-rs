#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Ticker};
use embedded_graphics::{Drawable as _, geometry::Point};
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_async::delay::DelayNs as _;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_io::Write;
use log::{error, info};
use ssd1306::{Ssd1306Async, prelude::*};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{
    BootMode, clocks, gic, gpio, gtc,
    spi::{self, SpiAsync},
    time::Hertz,
    uart::{self, TxAsync},
};

use embedded_graphics::image::Image;
use tinybmp::Bmp;

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard OLED example --\n\r";

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config {
        init_l2_cache: true,
        level_shifter_config: Some(zynq7000_hal::LevelShifterConfig::EnableAll),
        interrupt_config: Some(zynq7000_hal::InteruptConfig::AllInterruptsToCpu0),
    })
    .unwrap();
    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let mut clocks = clocks::Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    let target_spi_ref_clock = clocks.arm_clocks().cpu_1x_clk() * 2;
    // SPI reference clock must be larger than the CPU 1x clock. Also, taking the largest value
    // actually seems to be problematic. We take 200 MHz here, which is significantly larger than
    // the CPU 1x clock which is around 110 MHz.
    spi::configure_spi_ref_clock(&mut clocks, target_spi_ref_clock);

    assert!(
        clocks.io_clocks().spi_clk().to_raw()
            > (clocks.arm_clocks().cpu_1x_clk().to_raw() as f32 * 1.2) as u32,
        "SPI reference clock must be larger than CPU 1x clock"
    );

    let mut gpio_pins = gpio::GpioPins::new(periphs.gpio);

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
    uart.flush().unwrap();

    // Safety: We are not multi-threaded yet.
    let log_reader = zynq7000_hal::log::asynch::init(log::LevelFilter::Trace)
        .expect("Failed to initialize async logger");

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);
    info!(
        "SPI reference clock speed: {:?}",
        clocks.io_clocks().spi_clk()
    );

    spawner.spawn(logger_task(uart, log_reader).unwrap());

    let mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
    let emio_leds: [gpio::Output; 8] = [
        gpio::Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(1).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(2).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(3).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(4).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(5).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(6).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(7).unwrap(), gpio::PinState::Low),
    ];
    spawner.spawn(blinky_task(mio_led, emio_leds).unwrap());

    let dc_pin = gpio::Output::new_for_emio(gpio_pins.emio.take(11).unwrap(), gpio::PinState::High);

    let mut reset_pin =
        gpio::Output::new_for_emio(gpio_pins.emio.take(12).unwrap(), gpio::PinState::High);
    let mut oled_vdd_switch =
        gpio::Output::new_for_emio(gpio_pins.emio.take(13).unwrap(), gpio::PinState::High);
    let mut oled_vbat_switch =
        gpio::Output::new_for_emio(gpio_pins.emio.take(14).unwrap(), gpio::PinState::High);
    Delay.delay_ms(100).await;

    let spi = spi::Spi::new_for_emio(
        periphs.spi_0,
        spi::Config::calculate_for_io_clock(
            Hertz::MHz(8),
            clocks.io_clocks(),
            embedded_hal::spi::MODE_0,
            spi::SlaveSelectConfig::AutoCsManualStart,
        ),
    )
    .expect("Failed to initialize SPI");
    let spi_asynch = SpiAsync::new(spi);
    let exclusive_device = ExclusiveDevice::new(spi_asynch, DummyPin::new_high(), NoDelay)
        .expect("Failed to create exclusive SPI device");
    let spi_if = SPIInterface::new(exclusive_device, dc_pin);
    let mut ssd1306 = Ssd1306Async::new(spi_if, DisplaySize128x32, DisplayRotation::Rotate180);

    oled_vdd_switch.set_low();
    oled_vbat_switch.set_low();
    Delay.delay_ms(100).await;

    ssd1306
        .reset(&mut reset_pin, &mut embassy_time::Delay {})
        .await
        .expect("display reset error");
    let mut display = ssd1306.into_buffered_graphics_mode();
    display.init().await.expect("display init error");

    // Include the BMP file data.
    let ferris_data = include_bytes!("../../assets/ferris-flat-happy-small.bmp");
    let rust_logo_data = include_bytes!("../../assets/rust-logo-single-path.bmp");
    // Parse the BMP file.
    let bmp_rust = Bmp::from_slice(rust_logo_data).expect("BMP loading error");
    let bmp_ferris = Bmp::from_slice(ferris_data).expect("BMP loading error");
    // Draw the image with the top left corner at (10, 20) by wrapping it in
    // an embedded-graphics `Image`.
    Image::new(&bmp_rust, Point::new(0, 0))
        .draw(&mut display)
        .expect("image drawing error");
    Image::new(&bmp_ferris, Point::new(32, 0))
        .draw(&mut display)
        .expect("image drawing error");
    display.flush().await.unwrap();
    let mut ticker = Ticker::every(Duration::from_millis(50));
    let mut ferris = FerrisMovement::new(32, Direction::Right, 32, 76);

    loop {
        Image::new(&bmp_ferris, Point::new(ferris.pos as i32, 0))
            .draw(&mut display)
            .expect("image drawing error");
        display.flush().await.expect("flush error");
        ferris.step();
        ticker.next().await;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Right,
    Left,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FerrisMovement {
    pub pos: u16,
    pub dir: Direction,
    pub left_threshold: u16,
    pub right_threshold: u16,
}

impl FerrisMovement {
    /// Creates a new movement controller.
    /// Assumes: left_threshold <= right_threshold and pos is within that range.
    pub fn new(pos: u16, dir: Direction, left_threshold: u16, right_threshold: u16) -> Self {
        Self {
            pos,
            dir,
            left_threshold,
            right_threshold,
        }
    }

    /// Move one tick and "bounce" between thresholds.
    pub fn step(&mut self) {
        match self.dir {
            Direction::Right => {
                if self.pos >= self.right_threshold {
                    self.dir = Direction::Left; // flip at right boundary
                } else {
                    self.pos += 1;
                }
            }
            Direction::Left => {
                if self.pos <= self.left_threshold {
                    self.dir = Direction::Right; // flip at left boundary
                } else {
                    self.pos -= 1;
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn logger_task(
    uart: uart::Uart,
    reader: embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, 4096>,
) -> ! {
    let (tx, _) = uart.split();
    let mut tx_async = TxAsync::new(tx);
    let mut log_buf: [u8; 2048] = [0; 2048];
    loop {
        let read_bytes = reader.read(&mut log_buf).await;
        if read_bytes > 0 {
            tx_async.write(&log_buf[0..read_bytes]).unwrap().await;
        }
    }
}

#[embassy_executor::task]
pub async fn blinky_task(mut mio_led: gpio::Output, mut emio_leds: [gpio::Output; 8]) {
    let mut ticker = Ticker::every(Duration::from_millis(200));
    loop {
        mio_led.toggle().unwrap();

        // Create a wave pattern for emio_leds
        for led in emio_leds.iter_mut() {
            led.toggle().unwrap();
            ticker.next().await; // Wait for the next ticker for each toggle
        }
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
            if ppi_interrupt == gic::PpiInterrupt::GlobalTimer {
                unsafe {
                    zynq7000_embassy::on_interrupt();
                }
            }
        }
        gic::Interrupt::Spi(spi_interrupt) => match spi_interrupt {
            gic::SpiInterrupt::Uart1 => {
                zynq7000_hal::uart::tx_async::on_interrupt_tx(uart::UartId::Uart1);
            }
            gic::SpiInterrupt::Spi0 => {
                zynq7000_hal::spi::asynch::on_interrupt(spi::SpiId::Spi0);
            }
            _ => {
                log::warn!("Unhandled SPI interrupt: {:?}", spi_interrupt);
            }
        },
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
