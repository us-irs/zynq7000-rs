//! QSPI flasher for the Zedboard. Assumes that external scripting took care of transferring
//! a boot binary to RAM.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use arbitrary_int::{traits::Integer as _, u2};
use core::panic::PanicInfo;
use embedded_hal::{delay::DelayNs as _, digital::StatefulOutputPin as _};
use embedded_io::Write as _;
use log::{error, info};
use zedboard_bsp::qspi_spansion;
use zynq7000_boot_image::BootHeader;
use zynq7000_hal::{
    BootMode, LevelShifterConfig, clocks, gpio, prelude::*, priv_tim, qspi, time::Hertz, uart,
};
use zynq7000_rt as _;

// Define the clock frequency as a constant.
//
// Not required for the PAC mode, is required for clean delays in HAL mode.
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_333);

// TODO: Make this configurable somehow?
const BOOT_BIN_BASE_ADDR: usize = 0x1000_0000;
const BOOT_BIN_SIZE_ADDR: usize = 0x900_000;

// Maximum of 16 MB is allowed for now.
const MAX_BOOT_BIN_SIZE: usize = 16 * 1024 * 1024;

const VERIFY_PROGRAMMING: bool = true;

#[allow(dead_code)]
const QSPI_DEV_COMBINATION: qspi::QspiDeviceCombination = qspi::QspiDeviceCombination {
    vendor: qspi::QspiVendor::WinbondAndSpansion,
    operating_mode: qspi::OperatingMode::FastReadQuadOutput,
    two_devices: false,
};

const INIT_STRING: &str = "-- Zynq 7000 Zedboard QSPI flasher --\n\r";

#[zynq7000_rt::entry]
fn main() -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config {
        init_l2_cache: true,
        level_shifter_config: Some(LevelShifterConfig::EnableAll),
        interrupt_config: Some(zynq7000_hal::InteruptConfig::AllInterruptsToCpu0),
    })
    .unwrap();
    let clocks = clocks::Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    // Unwrap okay, we only call this once on core 0 here.
    let mut timer = priv_tim::CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();

    let gpio_pins = gpio::GpioPins::new(periphs.gpio);

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
            log::LevelFilter::Info,
            false,
        )
    };

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let qspi_clock_config =
        qspi::ClockConfig::calculate_with_loopback(qspi::SrcSelIo::IoPll, &clocks, 100.MHz())
            .expect("QSPI clock calculation failed");
    let qspi = qspi::Qspi::new_single_qspi_with_feedback(
        periphs.qspi,
        qspi_clock_config,
        qspi::MODE_0,
        qspi::IoType::LvCmos33,
        gpio_pins.mio.mio1,
        (
            gpio_pins.mio.mio2,
            gpio_pins.mio.mio3,
            gpio_pins.mio.mio4,
            gpio_pins.mio.mio5,
        ),
        gpio_pins.mio.mio6,
        gpio_pins.mio.mio8,
    );

    let qspi_io_mode = qspi.into_io_mode(false);

    let mut spansion_qspi = qspi_spansion::QspiSpansionS25Fl256SIoMode::new(
        qspi_io_mode,
        qspi_spansion::Config {
            set_quad_bit_if_necessary: true,
            latency_config: Some(u2::ZERO),
            clear_write_protection: true,
        },
    );

    let mut boot_bin_slice = unsafe {
        core::slice::from_raw_parts(BOOT_BIN_BASE_ADDR as *const _, BootHeader::FIXED_SIZED_PART)
    };
    // This perform some basic validity checks.
    let _boot_header = BootHeader::new(&boot_bin_slice[0..BootHeader::FIXED_SIZED_PART])
        .expect("failed to parse boot header");
    let boot_bin_size =
        unsafe { core::ptr::read_volatile(BOOT_BIN_SIZE_ADDR as *const u32) as usize };
    if boot_bin_size == 0 || boot_bin_size > MAX_BOOT_BIN_SIZE {
        panic!(
            "boot binary size read at address {:#x} is invalid: found {}, must be in range [0, {}]",
            BOOT_BIN_SIZE_ADDR, boot_bin_size, MAX_BOOT_BIN_SIZE
        );
    }
    boot_bin_slice =
        unsafe { core::slice::from_raw_parts(BOOT_BIN_BASE_ADDR as *const _, boot_bin_size) };
    info!(
        "flashing boot binary with {} bytes to QSPI address 0x0",
        boot_bin_size
    );

    let mut current_addr = 0;
    let mut read_buf = [0u8; qspi_spansion::PAGE_SIZE];
    let mut next_checkpoint = 0.05;
    while current_addr < boot_bin_size {
        if current_addr % 0x10000 == 0 {
            log::debug!("Erasing sector at address {:#x}", current_addr);
            match spansion_qspi.erase_sector(current_addr as u32) {
                Ok(()) => {}
                Err(e) => {
                    error!(
                        "failed to erase sector at address {:#x}: {:?}",
                        current_addr, e
                    );
                    panic!("QSPI erase failed");
                }
            }
        }
        let write_size = core::cmp::min(qspi_spansion::PAGE_SIZE, boot_bin_size - current_addr);
        let write_slice = &boot_bin_slice[current_addr..current_addr + write_size];
        log::debug!("Programming address {:#x}", current_addr);
        match spansion_qspi.program_page(current_addr as u32, write_slice) {
            Ok(()) => {}
            Err(e) => {
                log::error!(
                    "failed to write data to QSPI at address {:#x}: {:?}",
                    current_addr,
                    e
                );
                panic!("QSPI write failed");
            }
        }
        if VERIFY_PROGRAMMING {
            spansion_qspi.read_page_fast_read(
                current_addr as u32,
                &mut read_buf[0..write_size],
                true,
            );
            if &read_buf[0..write_size] != write_slice {
                error!(
                    "data verification failed at address {:#x}: wrote {:x?}, read {:x?}",
                    current_addr,
                    &write_slice[0..core::cmp::min(16, write_size)],
                    &read_buf[0..core::cmp::min(16, write_size)]
                );
                panic!("QSPI data verification failed");
            }
        }
        current_addr += write_size;
        if current_addr as f32 / boot_bin_size as f32 >= next_checkpoint {
            log::info!("Write progress {} %", libm::roundf(next_checkpoint * 100.0));
            next_checkpoint += 0.05;
        }
    }
    info!("flashing done");

    let mut mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
    loop {
        mio_led.toggle().unwrap();

        timer.delay_ms(500);
    }
}

#[zynq7000_rt::irq]
pub fn irq_handler() {}

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
fn panic(_info: &PanicInfo) -> ! {
    loop {
        nop();
    }
}
