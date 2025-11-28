//! This example application shows usage of the non-blocking APIs offered for
//! various UART peripherals which are part of the PS or the sample bitstream.
//!
//! This example ONLY works with the provided `zedboard-fpga-design`.
//! The example FPGA design contains the following components relevant for this design
//!
//! - An AXI UARTLite peripheral, with the interrupt signal connected to the PS
//! - An AXI UART16550 peripheral, with the interrupt signal connected to the PS
//! - A custom UART multiplexer component. This multiplexer has various configuration modes
//!   configurable via 3 EMIO pins:
//!    1. Route UART0 RX to pin JA1 and TX to JA2.
//!    2. Route AXI UARTLite RX to pin JA1 and TX to JA2.
//!    3. Route AXI UART16550 RX to pin JA1 and TX to JA2.
//!    4. Route UART0 to AXI UARTLite.
//!    5. Route UART0 to AXI 16550.
//!    6. Route AXI UARTLite to AXI 16550.
//! - Options 4, 5 and 6 allow to test and show the non-blocking API without the need for external
//!   hardware.
//!
//! This example runs one embassy task for each UART, which periodically sends variable sized
//! string content out via its TX port. Each UART peripheral also permanently listens to all
//! incoming RX data via interrupts. Received data will be sent from the interrupt handler
//! to the main thread and will be printed to the main console on UART 1.
#![no_std]
#![no_main]
extern crate alloc;

use aarch32_cpu::asm::nop;
use alloc::format;
use axi_uart16550::AxiUart16550;
use axi_uartlite::AxiUartlite;
use core::{cell::RefCell, panic::PanicInfo};
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_alloc::LlffHeap as Heap;
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write as _;
use heapless::spsc::Queue;
use log::{error, info, warn};
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    configure_level_shifter,
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{GpioPins, Output, PinState},
    gtc::GlobalTimerCounter,
    l2_cache,
    time::Hertz,
    uart::{ClockConfig, Config, Uart},
};

pub enum UartMode {
    Uart0ToUartlite,
    Uart0ToUart16550,
    UartliteToUart16550,
}

const UART_MODE: UartMode = UartMode::Uart0ToUart16550;
const INIT_STRING: &str = "-- Zynq 7000 Zedboard non-blocking UART example --\n\r";

#[global_allocator]
static HEAP: Heap = Heap::empty();

use zynq7000::{Peripherals, slcr::LevelShifterConfig};
use zynq7000_rt as _;

// Define the clock frequency as a constant
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);

const AXI_UARTLITE_BASE_ADDR: u32 = 0x42C0_0000;
const AXI_UAR16550_BASE_ADDR: u32 = 0x43C0_0000;

pub const UARTLITE_PL_INT_ID: usize = 0;
pub const UART16550_PL_INT_ID: usize = 1;

const RB_SIZE: usize = 512;

// These queues are used to send all data received in the UART interrupt handlers to the main
// processing thread.
static QUEUE_UART_0: static_cell::ConstStaticCell<heapless::spsc::Queue<u8, RB_SIZE>> =
    static_cell::ConstStaticCell::new(Queue::new());
static QUEUE_UARTLITE: static_cell::ConstStaticCell<heapless::spsc::Queue<u8, RB_SIZE>> =
    static_cell::ConstStaticCell::new(Queue::new());
static QUEUE_UART16550: static_cell::ConstStaticCell<heapless::spsc::Queue<u8, RB_SIZE>> =
    static_cell::ConstStaticCell::new(Queue::new());

// Those are all used by the interrupt handler, so we have to do the Mutex dance.
static RX_UART_0: Mutex<RefCell<Option<zynq7000_hal::uart::Rx>>> = Mutex::new(RefCell::new(None));

static UART_0_PROD: Mutex<RefCell<Option<heapless::spsc::Producer<'static, u8, RB_SIZE>>>> =
    Mutex::new(RefCell::new(None));
static UARTLITE_PROD: Mutex<RefCell<Option<heapless::spsc::Producer<'static, u8, RB_SIZE>>>> =
    Mutex::new(RefCell::new(None));
static UART16550_PROD: Mutex<RefCell<Option<heapless::spsc::Producer<'static, u8, RB_SIZE>>>> =
    Mutex::new(RefCell::new(None));

/// Entry point (not called like a normal main function)
#[unsafe(no_mangle)]
pub extern "C" fn boot_core(cpu_id: u32) -> ! {
    if cpu_id != 0 {
        panic!("unexpected CPU ID {}", cpu_id);
    }
    main();
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum UartSel {
    Uart0 = 0b000,
    Uartlite = 0b001,
    Uart16550 = 0b010,
    Uart0ToUartlite = 0b011,
    Uart0ToUart16550 = 0b100,
    UartliteToUart16550 = 0b101,
}

pub struct UartMultiplexer {
    sel_pins: [Output; 3],
}

impl UartMultiplexer {
    pub fn new(mut sel_pins: [Output; 3]) -> Self {
        for pin in sel_pins.iter_mut() {
            pin.set_low();
        }
        Self { sel_pins }
    }

    pub fn select(&mut self, sel: UartSel) {
        // TODO: A pin group switcher would be nice to do this in one go.
        match sel {
            UartSel::Uart0 => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_low();
            }
            UartSel::Uartlite => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_high();
            }
            UartSel::Uart16550 => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_high();
                self.sel_pins[0].set_low();
            }
            UartSel::Uart0ToUartlite => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_high();
                self.sel_pins[0].set_high();
            }
            UartSel::Uart0ToUart16550 => {
                self.sel_pins[2].set_high();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_low();
            }
            UartSel::UartliteToUart16550 => {
                self.sel_pins[2].set_high();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_high();
            }
        }
    }
}

#[embassy_executor::main]
#[unsafe(export_name = "main")]
async fn main(spawner: Spawner) -> ! {
    let mut dp = Peripherals::take().unwrap();
    l2_cache::init_with_defaults(&mut dp.l2c);

    // Enable PS-PL level shifters.
    configure_level_shifter(LevelShifterConfig::EnableAll);

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    // Set up the global interrupt controller.
    let mut gic = GicConfigurator::new_with_init(dp.gicc, dp.gicd);
    gic.enable_all_interrupts();
    gic.set_all_spi_interrupt_targets_cpu0();
    // AXI UARTLite documentation mentions that a rising-edge sensitive interrupt is generated,
    // but the GIC configures high-level sensitivity for the PL interrupts by default. We
    // need to change it to edge sensitivity.
    gic.set_pl_interrupt_sensitivity(UARTLITE_PL_INT_ID, zynq7000_hal::gic::SpiSensitivity::Edge)
        .unwrap();
    gic.enable();
    unsafe {
        gic.enable_interrupts();
    }
    let mut gpio_pins = GpioPins::new(dp.gpio);

    // Set up global timer counter and embassy time driver.
    let gtc = GlobalTimerCounter::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    // Set up the UART, we are logging with it.
    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut log_uart = Uart::new_with_mio_for_uart_1(
        dp.uart_1,
        Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    log_uart.write_all(INIT_STRING.as_bytes()).unwrap();

    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        // 10 kB heap.
        const HEAP_SIZE: usize = 1024 * 10;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
    }

    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            log_uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    // Set up UART multiplexing before creating and configuring the UARTs.
    let mut uart_mux = UartMultiplexer::new([
        Output::new_for_emio(gpio_pins.emio.take(8).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(9).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(10).unwrap(), PinState::Low),
    ]);
    match UART_MODE {
        UartMode::Uart0ToUartlite => uart_mux.select(UartSel::Uart0ToUartlite),
        UartMode::Uart0ToUart16550 => uart_mux.select(UartSel::Uart0ToUart16550),
        UartMode::UartliteToUart16550 => uart_mux.select(UartSel::UartliteToUart16550),
    }

    // UART0 routed through EMIO to PL pins.
    let uart_0 =
        Uart::new_with_emio(dp.uart_0, Config::new_with_clk_config(uart_clk_config)).unwrap();
    // Safety: Valid address of AXI UARTLITE.
    let mut uartlite = unsafe { AxiUartlite::new(AXI_UARTLITE_BASE_ADDR) };
    // We need to call this before splitting the structure, because the interrupt signal is
    // used for both TX and RX, so the API is only exposed for this structure.
    uartlite.enable_interrupt();

    let (clk_config, error) =
        axi_uart16550::ClockConfig::new_autocalc_with_error(clocks.pl_clocks()[0], 115200).unwrap();
    assert!(error < 0.02);
    let _uart_16550 = unsafe {
        AxiUart16550::new(
            AXI_UAR16550_BASE_ADDR,
            axi_uart16550::UartConfig::new_with_clk_config(clk_config),
        )
    };

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mio_led = Output::new_for_mio(gpio_pins.mio.mio7, PinState::Low);
    let emio_leds: [Output; 8] = [
        Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(1).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(2).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(3).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(4).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(5).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(6).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(7).unwrap(), PinState::Low),
    ];

    let (uart_0_tx, mut uart_0_rx) = uart_0.split();
    let (uartlite_tx, _uartlite_rx) = uartlite.split();
    let (uart_16550_tx, mut uart_16550_rx) = _uart_16550.split();
    let (uart_0_prod, mut uart_0_cons) = QUEUE_UART_0.take().split();
    let (uartlite_prod, mut uartlite_cons) = QUEUE_UARTLITE.take().split();
    let (uart16550_prod, mut uart16550_cons) = QUEUE_UART16550.take().split();
    // Use our helper function to start RX handling.
    uart_0_rx.start_interrupt_driven_reception();
    // Use our helper function to start RX handling.
    uart_16550_rx.start_interrupt_driven_reception();
    critical_section::with(|cs| {
        UART_0_PROD.borrow(cs).borrow_mut().replace(uart_0_prod);
        UARTLITE_PROD.borrow(cs).borrow_mut().replace(uartlite_prod);
        UART16550_PROD
            .borrow(cs)
            .borrow_mut()
            .replace(uart16550_prod);
        RX_UART_0.borrow(cs).borrow_mut().replace(uart_0_rx);
    });
    spawner.spawn(led_task(mio_led, emio_leds)).unwrap();

    match UART_MODE {
        UartMode::Uart0ToUartlite => {
            spawner.spawn(uartlite_task(uartlite_tx)).unwrap();
            spawner.spawn(uart_0_task(uart_0_tx)).unwrap();
        }
        UartMode::Uart0ToUart16550 => {
            spawner.spawn(uart_0_task(uart_0_tx)).unwrap();
            spawner.spawn(uart_16550_task(uart_16550_tx)).unwrap();
        }
        UartMode::UartliteToUart16550 => {
            spawner.spawn(uartlite_task(uartlite_tx)).unwrap();
            spawner.spawn(uart_16550_task(uart_16550_tx)).unwrap();
        }
    }
    let mut read_buf: [u8; RB_SIZE] = [0; RB_SIZE];

    let mut ticker = Ticker::every(Duration::from_millis(200));
    loop {
        let read_bytes = uart_0_cons.len();
        (0..read_bytes).for_each(|i| {
            read_buf[i] = unsafe { uart_0_cons.dequeue_unchecked() };
        });
        if read_bytes > 0 {
            info!("Received {read_bytes} bytes on UART0");
            info!(
                "Data: {:?}",
                core::str::from_utf8(&read_buf[0..read_bytes]).unwrap()
            );
        }

        let read_bytes = uartlite_cons.len();
        (0..read_bytes).for_each(|i| {
            read_buf[i] = unsafe { uartlite_cons.dequeue_unchecked() };
        });
        if read_bytes > 0 {
            info!("Received {read_bytes} bytes on UARTLITE");
            info!(
                "Data: {:?}",
                core::str::from_utf8(&read_buf[0..read_bytes]).unwrap()
            );
        }

        let read_bytes = uart16550_cons.len();
        (0..read_bytes).for_each(|i| {
            read_buf[i] = unsafe { uart16550_cons.dequeue_unchecked() };
        });
        if read_bytes > 0 {
            info!("Received {read_bytes} bytes on UART16550");
            info!(
                "Data: {:?}",
                core::str::from_utf8(&read_buf[0..read_bytes]).unwrap()
            );
        }
        ticker.next().await; // Wait for the next cycle of the ticker
    }
}

fn build_print_string(prefix: &str, base_str: &str) -> alloc::string::String {
    format!("{prefix} {base_str}\n\r")
}

#[embassy_executor::task]
async fn led_task(mut mio_led: Output, mut emio_leds: [Output; 8]) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led_idx = 0;
    loop {
        mio_led.toggle().unwrap();

        emio_leds[led_idx].toggle().unwrap();
        led_idx += 1;
        if led_idx >= emio_leds.len() {
            led_idx = 0;
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn uartlite_task(uartlite: axi_uartlite::Tx) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let str0 = build_print_string("UARTLITE:", "Hello World");
    let str1 = build_print_string(
        "UARTLITE:",
        "Long Hello which should completely fill the FIFO",
    );
    let mut idx = 0;
    let print_strs = [str0.as_bytes(), str1.as_bytes()];
    let mut tx_async = axi_uartlite::tx_async::TxAsync::new(uartlite, 0).unwrap();
    loop {
        tx_async.write(print_strs[idx]).await;
        idx += 1;
        if idx == 2 {
            idx = 0;
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn uart_0_task(uart_tx: zynq7000_hal::uart::Tx) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut tx_async = zynq7000_hal::uart::TxAsync::new(uart_tx);
    let str0 = build_print_string("UART0:", "Hello World");
    let str1 = build_print_string(
        "UART0:",
        "Long Hello which should completely fill the 64 byte FIFO of the UART0 peripheral",
    );
    let mut idx = 0;
    let print_strs = [str0.as_bytes(), str1.as_bytes()];
    loop {
        tx_async.write(print_strs[idx]).await;
        idx += 1;
        if idx == 2 {
            idx = 0;
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn uart_16550_task(uart_tx: axi_uart16550::Tx) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut tx_async = axi_uart16550::TxAsync::new(uart_tx, 0, embassy_time::Delay).unwrap();
    let str0 = build_print_string("UART16550:", "Hello World");
    let str1 = build_print_string(
        "UART16550:",
        "Long Hello which should completely fill the FIFO",
    );
    let mut idx = 0;
    let print_strs = [str0.as_bytes(), str1.as_bytes()];
    loop {
        tx_async.write(print_strs[idx]).await;
        idx += 1;
        if idx == 2 {
            idx = 0;
        }

        ticker.next().await;
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {
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
        Interrupt::Spi(spi_interrupt) => match spi_interrupt {
            zynq7000_hal::gic::SpiInterrupt::Pl0 => {
                on_interrupt_axi_uartlite();
            }
            zynq7000_hal::gic::SpiInterrupt::Pl1 => {
                on_interrupt_axi_16550();
            }
            zynq7000_hal::gic::SpiInterrupt::Uart0 => {
                on_interrupt_uart_0();
            }

            _ => (),
        },
        Interrupt::Invalid(_) => (),
        Interrupt::Spurious => (),
    }
    gic_helper.end_of_interrupt(irq_info);
}

fn on_interrupt_axi_uartlite() {
    let mut buf = [0; axi_uartlite::FIFO_DEPTH];
    let mut rx = unsafe { axi_uartlite::Rx::steal(AXI_UARTLITE_BASE_ADDR as usize) };
    // Handle RX first: Empty the FIFO content into local buffer.
    let read_bytes = rx.on_interrupt_rx(&mut buf);
    // Handle TX next: Handle pending asynchronous TX operations.
    let mut tx = unsafe { axi_uartlite::Tx::steal(AXI_UARTLITE_BASE_ADDR as usize) };
    axi_uartlite::on_interrupt_tx(&mut tx, 0);
    // Send received RX data to main task.
    if read_bytes > 0 {
        critical_section::with(|cs| {
            let mut prod_ref_mut = UARTLITE_PROD.borrow(cs).borrow_mut();
            let prod = prod_ref_mut.as_mut().unwrap();
            (0..read_bytes).for_each(|i| {
                prod.enqueue(buf[i]).expect("failed to enqueue byte");
            });
        });
    }
}

fn on_interrupt_axi_16550() {
    let mut buf = [0; axi_uart16550::FIFO_DEPTH];
    let mut read_bytes = 0;
    let mut rx = unsafe { axi_uart16550::Rx::steal(AXI_UAR16550_BASE_ADDR as usize) };
    let iir = rx.read_iir();
    if let Ok(int_id) = iir.int_id() {
        match int_id {
            axi_uart16550::registers::InterruptId2::ReceiverLineStatus => {
                let errors = rx.on_interrupt_receiver_line_status(iir);
                warn!("Receiver line status error: {errors:?}");
            }
            axi_uart16550::registers::InterruptId2::RxDataAvailable
            | axi_uart16550::registers::InterruptId2::CharTimeout => {
                read_bytes = rx.on_interrupt_data_available_or_char_timeout(int_id, &mut buf);
            }
            axi_uart16550::registers::InterruptId2::ThrEmpty => {
                let mut tx = unsafe { axi_uart16550::Tx::steal(AXI_UAR16550_BASE_ADDR as usize) };
                axi_uart16550::tx_async::on_interrupt_tx(&mut tx, 0);
            }
            axi_uart16550::registers::InterruptId2::ModemStatus => (),
        }
    }
    // Send received RX data to main task.
    if read_bytes > 0 {
        critical_section::with(|cs| {
            let mut prod_ref_mut = UART16550_PROD.borrow(cs).borrow_mut();
            let prod = prod_ref_mut.as_mut().unwrap();
            (0..read_bytes).for_each(|i| {
                prod.enqueue(buf[i]).expect("failed to enqueue byte");
            });
        });
    }
}

fn on_interrupt_uart_0() {
    let mut buf = [0; zynq7000_hal::uart::FIFO_DEPTH];
    let mut read_bytes = 0;
    // Handle RX first: Empty the FIFO content into local buffer.
    critical_section::with(|cs| {
        let mut uart0 = RX_UART_0.borrow(cs).borrow_mut();
        read_bytes = uart0
            .as_mut()
            .unwrap()
            .on_interrupt(&mut buf, true)
            .read_bytes();
    });
    // Handle TX next: Handle pending asynchronous TX operations.
    zynq7000_hal::uart::on_interrupt_tx(zynq7000_hal::uart::UartId::Uart0);
    // Send received RX data to main task.
    if read_bytes > 0 {
        critical_section::with(|cs| {
            let mut prod_ref_mut = UART_0_PROD.borrow(cs).borrow_mut();
            let prod = prod_ref_mut.as_mut().unwrap();
            (0..read_bytes).for_each(|i| {
                prod.enqueue(buf[i]).expect("failed to enqueue byte");
            });
        });
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
