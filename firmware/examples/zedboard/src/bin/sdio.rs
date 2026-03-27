#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::error;
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::gpio::Input;
use zynq7000_hal::prelude::*;
use zynq7000_hal::sd::SdClockConfig;
use zynq7000_hal::{BootMode, clocks, gic, gpio, gtc, sd::SdCardUninit, uart};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard SDIO example --\n\r";

// These are off by default because they write to the SD card as well.
const LOW_LEVEL_TESTS: bool = false;
const SDMMC_RS_TESTS: bool = false;

#[derive(Debug, Clone, Copy)]
pub struct DummyTimeSource;

impl embedded_sdmmc::TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp::from_calendar(1970, 1, 1, 0, 0, 0).unwrap()
    }
}

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
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
        SdClockConfig::calculate_for_io_clock(clocks.io_clocks(), 100.MHz(), 10.MHz()).unwrap();
    log::info!("SDIO clock config: {:?}", sdio_clock_config);
    let sd_card_uninit = SdCardUninit::new_for_sdio_0(
        periphs.sdio_0,
        sdio_clock_config,
        // On the zedboard, the bank has a 1.8 V voltage which is shifted up to 3.3 V by a
        // level shifter.
        zynq7000_hal::sd::IoType::LvCmos18,
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
    log::info!("Card detect state: {:?}", card_detect.is_low());
    log::info!("Write protect state: {:?}", write_protect.is_high());

    let capabilities = sd_card_uninit.ll().capabilities();
    log::debug!("SDIO Capabilities: {:?}", capabilities);

    let present_state = sd_card_uninit.ll().read_present_state();
    log::debug!("SD present state: {:?}", present_state);

    let boot_mode = BootMode::new_from_regs();
    log::info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let mut mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);

    let sd_result = sd_card_uninit.initialize();
    let sd_card = match sd_result {
        Ok(card) => {
            log::info!("SD card info: {:?}", card.card_info());
            card
        }
        Err(e) => {
            panic!("SDIO init error: {e:?}");
        }
    };

    let mut buf: [u8; 4096] = [0; 4096];

    if LOW_LEVEL_TESTS {
        log::info!("doing SD card low-level tests");

        let mut cache_buf: [u8; 4096] = [0; 4096];

        // cache the data, will be written back later..
        sd_card
            .read_multiple_blocks(&mut cache_buf, 0x1000)
            .unwrap();

        let mut write_data: [u8; 4096] = [0; 4096];
        for chunk in write_data.chunks_mut(u8::MAX as usize) {
            for (idx, byte) in chunk.iter_mut().enumerate() {
                *byte = idx as u8;
            }
        }
        sd_card.write_multiple_blocks(&write_data, 0x1000).unwrap();

        sd_card.read_multiple_blocks(&mut buf, 0x1000).unwrap();
        for chunk in buf.chunks(u8::MAX as usize) {
            for (idx, byte) in chunk.iter().enumerate() {
                assert_eq!(idx as u8, *byte);
            }
        }

        sd_card.write_multiple_blocks(&cache_buf, 0x1000).unwrap();

        log::info!("SD card low-level tests success");
    }

    buf.fill(0);

    if SDMMC_RS_TESTS {
        log::info!("doing SD card embedded-sdmmc-rs tests");

        // Now let's look for volumes (also known as partitions) on our block device.
        // To do this we need a Volume Manager. It will take ownership of the block device.
        let volume_mgr = embedded_sdmmc::VolumeManager::new(sd_card, DummyTimeSource);
        // Try and access Volume 0 (i.e. the first partition).
        // The volume object holds information about the filesystem on that volume.
        let volume0 = volume_mgr
            .open_volume(embedded_sdmmc::VolumeIdx(0))
            .unwrap();

        // Open the root directory (mutably borrows from the volume).
        let mut current_dir = volume0.open_root_dir().unwrap();
        log::info!("iterating root directory");
        current_dir
            .iterate_dir(|entry| {
                log::info!("{:?}", entry);
                core::ops::ControlFlow::Continue(())
            })
            .unwrap();

        let new_file = current_dir
            .open_file_in_dir("__T.TXT", embedded_sdmmc::Mode::ReadWriteCreateOrTruncate)
            .unwrap();
        let string = "test string\n";
        new_file.write(string.as_bytes()).unwrap();
        new_file.close().unwrap();

        let read_new = current_dir
            .open_file_in_dir("__T.TXT", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();
        assert_eq!(read_new.length(), string.len() as u32);
        read_new.read(&mut buf).unwrap();
        let buf_as_str = core::str::from_utf8(&buf[0..string.len()]).unwrap();
        assert_eq!(buf_as_str, string);
        read_new.close().unwrap();

        current_dir.delete_entry_in_dir("__T.TXT").unwrap();

        assert_eq!(
            current_dir.find_directory_entry("__T.TXT").unwrap_err(),
            embedded_sdmmc::Error::NotFound
        );

        if current_dir.find_directory_entry("_TDIR").is_ok() {
            current_dir.delete_entry_in_dir("_TDIR").unwrap();
        }

        current_dir.make_dir_in_dir("_TDIR").unwrap();
        current_dir.change_dir("_TDIR").unwrap();
        current_dir.change_dir("..").unwrap();
        current_dir.delete_entry_in_dir("_TDIR").unwrap();

        current_dir.close().unwrap();

        log::info!("SD card embedded-sdmmc-rs success");
    }

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
