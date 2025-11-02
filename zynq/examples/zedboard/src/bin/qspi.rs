#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zedboard_bsp::qspi_spansion;
use zynq7000_hal::{BootMode, clocks, gic, gpio, gtc, prelude::*, qspi, uart};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard QSPI example --\n\r";
const QSPI_DEV_COMBINATION: qspi::QspiDeviceCombination = qspi::QspiDeviceCombination {
    vendor: qspi::QspiVendor::WinbondAndSpansion,
    operating_mode: qspi::OperatingMode::FastReadQuadOutput,
    two_devices: false,
};

/// Entry point (not called like a normal main function)
#[unsafe(no_mangle)]
pub extern "C" fn boot_core(cpu_id: u32) -> ! {
    if cpu_id != 0 {
        panic!("unexpected CPU ID {}", cpu_id);
    }
    main();
}

const ERASE_PROGRAM_READ_TEST: bool = false;

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
    let mut uart = uart::Uart::new_with_mio(
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

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let qspi_clock_config =
        qspi::ClockConfig::calculate_with_loopback(qspi::SrcSelIo::IoPll, &clocks, 100.MHz())
            .expect("QSPI clock calculation failed");
    let qspi = qspi::Qspi::new_single_qspi_with_feedback(
        periphs.qspi,
        qspi_clock_config,
        embedded_hal::spi::MODE_0,
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

    let mut spansion_qspi = qspi_spansion::QspiSpansionS25Fl256SIoMode::new(qspi_io_mode, true);

    let rdid = spansion_qspi.read_rdid_extended();
    info!(
        "QSPI Info: manufacturer ID: {:?}, interface type: {:?}, density: {:?}, sector arch: {:?}, model number: {:?}",
        rdid.base_id().manufacturer_id(),
        rdid.base_id().memory_interface_type(),
        rdid.base_id().density(),
        rdid.sector_arch(),
        rdid.model_number()
    );
    let cr1 = spansion_qspi.read_configuration_register();
    info!("QSPI Configuration Register 1: {:?}", cr1);

    let mut write_buf: [u8; u8::MAX as usize + 1] = [0x0; u8::MAX as usize + 1];
    for (idx, byte) in write_buf.iter_mut().enumerate() {
        *byte = idx as u8;
    }
    let mut read_buf = [0u8; 256];

    if ERASE_PROGRAM_READ_TEST {
        info!("performing erase, program, read test");
        spansion_qspi
            .erase_sector(0x10000)
            .expect("erasing sector failed");
        spansion_qspi.read_page_fast_read(0x10000, &mut read_buf, true);
        for read in read_buf.iter() {
            assert_eq!(*read, 0xFF);
        }
        read_buf.fill(0);
        spansion_qspi.program_page(0x10000, &write_buf).unwrap();
        spansion_qspi.read_page_fast_read(0x10000, &mut read_buf, true);
        for (read, written) in read_buf.iter().zip(write_buf.iter()) {
            assert_eq!(read, written);
        }
        info!("test successful");
    }

    let mut spansion_lqspi = spansion_qspi.into_linear_addressed(QSPI_DEV_COMBINATION.into());

    let guard = spansion_lqspi.read_guard();
    unsafe {
        core::ptr::copy_nonoverlapping(
            (qspi::QSPI_START_ADDRESS + 0x10000) as *const u8,
            read_buf.as_mut_ptr(),
            256,
        );
    }
    drop(guard);
    if ERASE_PROGRAM_READ_TEST {
        for (read, written) in read_buf.iter().zip(write_buf.iter()) {
            assert_eq!(
                read, written,
                "linear read failure, read and write missmatch"
            );
        }
    }

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let mut mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
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
