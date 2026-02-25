//! Simple FSBL for the Zedboard.
//!
//!This first variant is simplistic. It is currently only capable of QSPI boot. It searches for a
//! bitstream and ELF file inside the boot binary, flashes them and jumps to the ELF app.
//!
//! It can be easily adapted to other boards by changing the static DDR/DDRIOB configuration
//! and the used QSPI memory driver.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use arbitrary_int::traits::Integer as _;
use arbitrary_int::{u2, u6};
use core::panic::PanicInfo;
use embedded_io::Write as _;
use log::{error, info};
use zedboard_bsp::qspi_spansion::{self, QspiSpansionS25Fl256SLinearMode};
use zynq7000_boot_image::DestinationDevice;
use zynq7000_hal::priv_tim;
use zynq7000_hal::{
    BootMode,
    clocks::{
        Clocks,
        pll::{PllConfig, configure_arm_pll, configure_io_pll},
    },
    ddr::{DdrClockSetupConfig, configure_ddr_for_ddr3, memtest},
    devcfg, gic, gpio, l2_cache,
    prelude::*,
    qspi::{self, QSPI_START_ADDRESS},
    time::Hertz,
    uart::{ClockConfig, Config, Uart},
};

// PS clock input frequency.
const PS_CLK: Hertz = Hertz::from_raw(33_333_333);

/// 1600 MHz.
const ARM_CLK: Hertz = Hertz::from_raw(1_600_000_000);
/// 1000 MHz.
const IO_CLK: Hertz = Hertz::from_raw(1_000_000_000);

/// DDR frequency for the MT41K128M16JT-125 device.
const DDR_FREQUENCY: Hertz = Hertz::from_raw(533_333_333);

/// 1067 MHz.
const DDR_CLK: Hertz = Hertz::from_raw(2 * DDR_FREQUENCY.raw());

const PERFORM_DDR_MEMTEST: bool = false;

#[derive(Debug, PartialEq, Eq)]
pub enum BootMethod {
    Qspi,
    SdCard,
}

pub const BOOT_METHOD: BootMethod = BootMethod::Qspi;

pub const ELF_BASE_ADDR: usize = 0x100000;

/// 8 MB reserved for application ELF.
pub const BOOT_BIN_STAGING_OFFSET: usize = 8 * 1024 * 1024;

#[zynq7000_rt::entry]
fn main() -> ! {
    let boot_mode = BootMode::new_from_regs();
    // The unwraps are okay here, the provided clock frequencies are standard values also used
    // by other Xilinx tools.
    configure_arm_pll(
        boot_mode,
        PllConfig::new_from_target_clock(PS_CLK, ARM_CLK).unwrap(),
    );
    configure_io_pll(
        boot_mode,
        PllConfig::new_from_target_clock(PS_CLK, IO_CLK).unwrap(),
    );

    let mut periphs = zynq7000::Peripherals::take().unwrap();

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLK).unwrap();

    let gpio_pins = gpio::GpioPins::new(periphs.gpio);
    let mio_pins = gpio_pins.mio;
    // Set up the UART, we are logging with it.
    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut logger_uart = Uart::new_with_mio_for_uart_1(
        periphs.uart_1,
        Config::new_with_clk_config(uart_clk_config),
        (mio_pins.mio48, mio_pins.mio49),
    )
    .unwrap();
    logger_uart
        .write_all(b"-- Zedboard Rust FSBL --\n\r")
        .unwrap();
    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            logger_uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    // Set up the global interrupt controller.
    let mut gic = gic::GicConfigurator::new_with_init(periphs.gicc, periphs.gicd);
    gic.enable_all_interrupts();
    gic.set_all_spi_interrupt_targets_cpu0();
    gic.enable();
    // Enable interrupt exception.
    unsafe { gic.enable_interrupts() };

    info!("Configuring DDR..");
    configure_ddr_for_ddr3(
        periphs.ddrc,
        boot_mode,
        DdrClockSetupConfig {
            ps_clk: PS_CLK,
            ddr_clk: DDR_CLK,
            ddr_3x_div: u6::new(2),
            ddr_2x_div: u6::new(3),
        },
        &zedboard_bsp::ddriob_config_autogen::DDRIOB_CONFIG_SET_ZEDBOARD,
        &zedboard_bsp::ddrc_config_autogen::DDRC_CONFIG_ZEDBOARD,
    );
    info!("DDR init done.");

    info!("L2 cache init..");
    // Set up the L2 cache now that the DDR is in normal operation mode.
    l2_cache::init_with_defaults(&mut periphs.l2c);
    info!("L2 cache init done.");

    let priv_tim = priv_tim::CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();

    if PERFORM_DDR_MEMTEST {
        let ddr_base_addr = 0x100000;
        info!("performing DDR memory test..");
        unsafe {
            memtest::walking_zero_test(ddr_base_addr, 64).expect("walking one test failed");
            memtest::walking_one_test(ddr_base_addr, 64).expect("walking zero test failed");
            memtest::checkerboard_test(ddr_base_addr, 64).expect("checkerboard test failed");
        }
        info!("DDR memory test success.");
    }

    if BOOT_METHOD == BootMethod::Qspi {
        let qspi_clock_config = qspi::ClockConfig::calculate_with_loopback(
            zynq7000::slcr::clocks::SrcSelIo::IoPll,
            &clocks,
            100.MHz(),
        )
        .expect("QSPI clock calculation failed");
        let qspi = qspi::Qspi::new_single_qspi_with_feedback(
            periphs.qspi,
            qspi_clock_config,
            embedded_hal::spi::MODE_0,
            qspi::IoType::LvCmos33,
            mio_pins.mio1,
            (mio_pins.mio2, mio_pins.mio3, mio_pins.mio4, mio_pins.mio5),
            mio_pins.mio6,
            mio_pins.mio8,
        );

        let qspi_io_mode = qspi.into_io_mode(false);
        let spansion_qspi = qspi_spansion::QspiSpansionS25Fl256SIoMode::new(
            qspi_io_mode,
            qspi_spansion::Config {
                set_quad_bit_if_necessary: true,
                latency_config: Some(u2::ZERO),
                clear_write_protection: true,
            },
        );
        let spansion_lqspi =
            spansion_qspi.into_linear_addressed(qspi_spansion::QSPI_DEV_COMBINATION_REV_F.into());
        qspi_boot(spansion_lqspi, priv_tim);
    }
    loop {
        aarch32_cpu::asm::nop();
    }
}

fn qspi_boot(mut qspi: QspiSpansionS25Fl256SLinearMode, _priv_tim: priv_tim::CpuPrivateTimer) -> ! {
    let boot_bin_base_addr = ELF_BASE_ADDR + BOOT_BIN_STAGING_OFFSET;
    let mut boot_header_slice = unsafe {
        core::slice::from_raw_parts_mut(
            boot_bin_base_addr as *mut u8,
            zynq7000_boot_image::FIXED_BOOT_HEADER_SIZE,
        )
    };
    let read_guard = qspi.read_guard();
    // Currently, only boot.bin at address 0x0 of the QSPI is supported.
    unsafe {
        core::ptr::copy_nonoverlapping(
            QspiSpansionS25Fl256SLinearMode::BASE_ADDR as *mut u8,
            boot_header_slice.as_mut_ptr(),
            zynq7000_boot_image::FIXED_BOOT_HEADER_SIZE,
        );
    }
    drop(read_guard);

    let boot_header = zynq7000_boot_image::BootHeader::new(boot_header_slice).unwrap();
    let fsbl_offset = boot_header.source_offset();
    boot_header_slice =
        unsafe { core::slice::from_raw_parts_mut(boot_bin_base_addr as *mut u8, fsbl_offset) };

    // Read the rest of the boot header metadata.
    let read_guard = qspi.read_guard();
    unsafe {
        core::ptr::copy_nonoverlapping(
            (QspiSpansionS25Fl256SLinearMode::BASE_ADDR
                + zynq7000_boot_image::FIXED_BOOT_HEADER_SIZE) as *mut u8,
            boot_header_slice[zynq7000_boot_image::FIXED_BOOT_HEADER_SIZE..].as_mut_ptr(),
            fsbl_offset - zynq7000_boot_image::FIXED_BOOT_HEADER_SIZE,
        );
    }
    drop(read_guard);

    let boot_header = zynq7000_boot_image::BootHeader::new_unchecked(boot_header_slice);

    let mut name_buf: [u8; 256] = [0; 256];
    let mut opt_jump_addr = None;
    for (index, image_header) in boot_header.image_header_iterator().unwrap().enumerate() {
        let name = image_header.image_name(&mut name_buf).unwrap();
        if index == 0 {
            if !name.contains("fsbl") {
                log::warn!("first image name did not contain FSBL string");
            }
            // Skip the FSBL. It is probably currently running, and we do not want to re-flash it,
            // which would also lead to a self-overwrite.
            log::info!("skipping FSBL image");
            continue;
        }

        let _read_guard = qspi.read_guard();

        for (partition_index, partition) in image_header
            .partition_header_iterator(boot_header_slice)
            .unwrap()
            .enumerate()
        {
            let section_attrs = partition.section_attributes();
            if section_attrs.destination_device().is_err() {
                log::error!(
                    "invalid destination device ID {}",
                    section_attrs.destination_device().unwrap_err()
                );
                continue;
            }
            let dest_dev = section_attrs.destination_device().unwrap();
            match dest_dev {
                DestinationDevice::Pl => {
                    info!("Loading image '{name}' to PL (FPGA)..");
                    // Load the bitstream directly from linear mapped QSPI memory.
                    let boot_bin_slice = unsafe {
                        core::slice::from_raw_parts(
                            (QSPI_START_ADDRESS
                                + partition
                                    .data_offset()
                                    .expect("invalid PL partition data offset"))
                                as *const _,
                            partition.total_partition_length().unwrap(),
                        )
                    };
                    // The DMA will read from the linear mapped QSPI directly, so it
                    // has to be configured for reads using the guard!
                    devcfg::configure_bitstream_non_secure(true, boot_bin_slice)
                        .expect("unexpected unaligned address");
                    log::info!("loaded bitstream successfully");
                }
                DestinationDevice::Ps => {
                    // Load the bitstream directly from linear mapped QSPI memory.
                    let load_addr = partition.destination_load_address();
                    if load_addr < 0x10_0000 {
                        panic!("invalid load address which is not located in DDR memory");
                    }
                    log::info!(
                        "Loading partition {partition_index} for '{name}' to PS with load address {load_addr}.."
                    );

                    let source_slice = unsafe {
                        core::slice::from_raw_parts(
                            (QSPI_START_ADDRESS
                                + partition
                                    .data_offset()
                                    .expect("invalid PS partition data offset"))
                                as *const _,
                            partition.total_partition_length().unwrap(),
                        )
                    };
                    let target_slice = unsafe {
                        core::slice::from_raw_parts_mut(
                            load_addr as *mut u8,
                            partition.total_partition_length().unwrap(),
                        )
                    };

                    // Copy from the linear mapped QSPI to DDR,
                    target_slice.copy_from_slice(source_slice);

                    match &mut opt_jump_addr {
                        Some(current) => *current = core::cmp::min(*current, load_addr),
                        None => opt_jump_addr = Some(load_addr),
                    }
                    log::info!("load success");
                }
                _ => {
                    error!("Unsupported destination device {dest_dev:?}");
                    continue;
                }
            }
        }
    }

    match opt_jump_addr {
        Some(jump_addr) => {
            log::info!("jumping to address {}", jump_addr);
            zynq7000_hal::log::uart_blocking::flush();

            // Some clean up and preparation for jumping to the user application.
            zynq7000_hal::cache::clean_and_invalidate_data_cache();
            aarch32_cpu::register::TlbIAll::write();
            aarch32_cpu::register::BpIAll::write();
            aarch32_cpu::asm::dsb();
            aarch32_cpu::asm::isb();

            let jump_func: extern "C" fn() -> ! = unsafe { core::mem::transmute(jump_addr) };
            jump_func();
        }
        None => panic!("did not find application elf to boot inside boot binary!"),
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
