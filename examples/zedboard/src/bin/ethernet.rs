#![no_std]
#![no_main]

use core::{mem::MaybeUninit, panic::PanicInfo};
use cortex_ar::asm::nop;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    configure_level_shifter,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{GpioPins, Output, PinState},
    gtc::Gtc,
    uart::{ClkConfigRaw, Uart, UartConfig},
};

use zynq7000::{PsPeripherals, slcr::LevelShifterConfig};
use zynq7000_rt::{self as _, mmu::section_attrs::SHAREABLE_DEVICE, mmu_l1_table_mut};

const INIT_STRING: &str = "-- Zynq 7000 Zedboard Ethernet Example --\n\r";

#[unsafe(link_section = ".uncached")]
static RX_DESCRIPTORS: static_cell::ConstStaticCell<
    MaybeUninit<[zynq7000_hal::eth::rx_descr::Descriptor; 32]>,
> = static_cell::ConstStaticCell::new(MaybeUninit::uninit());

#[unsafe(link_section = ".uncached")]
static TX_DESCRIPTORS: static_cell::ConstStaticCell<
    MaybeUninit<[zynq7000_hal::eth::tx_descr::Descriptor; 32]>,
> = static_cell::ConstStaticCell::new(MaybeUninit::uninit());

#[repr(align(32))]
#[derive(Debug, Clone, Copy)]
pub struct AlignedBuffer(pub [u8; zynq7000_hal::eth::MTU]);

const NUM_RX_BUFS: usize = 32;
const NUM_TX_BUFS: usize = 32;

static ETH_RX_BUFS: static_cell::ConstStaticCell<[AlignedBuffer; NUM_RX_BUFS]> =
    static_cell::ConstStaticCell::new([AlignedBuffer([0; zynq7000_hal::eth::MTU]); NUM_RX_BUFS]);
static ETH_TX_BUFS: static_cell::ConstStaticCell<[AlignedBuffer; NUM_TX_BUFS]> =
    static_cell::ConstStaticCell::new([AlignedBuffer([0; zynq7000_hal::eth::MTU]); NUM_TX_BUFS]);

/// See memory.x file. 1 MB starting at this address will be configured as uncached memory using the
/// MMU.
const UNCACHED_ADDR: u32 = 0x4000000;

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
    // Configure the uncached memory region using the MMU.
    mmu_l1_table_mut().update(UNCACHED_ADDR, SHAREABLE_DEVICE);

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
    let uart_clk_config = ClkConfigRaw::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = Uart::new_with_mio(
        dp.uart_1,
        UartConfig::new_with_clk_config(uart_clk_config),
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

    let boot_mode = BootMode::new();
    let rx_bufs = ETH_RX_BUFS.take();

    let rx_descr = RX_DESCRIPTORS.take();
    let tx_descr = TX_DESCRIPTORS.take();
    rx_descr.write([const { zynq7000_hal::eth::rx_descr::Descriptor::new() }; 32]);
    tx_descr.write([const { zynq7000_hal::eth::tx_descr::Descriptor::new() }; 32]);
    let mut rx_descr_ref =
        zynq7000_hal::eth::rx_descr::DescriptorListRef::new(unsafe { &mut *rx_descr.as_mut_ptr() });
    let mut tx_descr_ref =
        zynq7000_hal::eth::tx_descr::DescriptorListRef::new(unsafe { &mut *tx_descr.as_mut_ptr() });
    rx_descr_ref.init();
    tx_descr_ref.init();
    for (index, rx_buf) in rx_bufs.iter().enumerate() {
        rx_descr_ref.set_rx_buf_address(index, rx_buf.0.as_ptr() as u32);
    }

    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(200));

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
    loop {
        mio_led.toggle().unwrap();

        // Create a wave pattern for emio_leds
        for led in emio_leds.iter_mut() {
            led.toggle().unwrap();
            ticker.next().await; // Wait for the next ticker for each toggle
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
    error!("Panic: {:?}", info);
    loop {}
}
