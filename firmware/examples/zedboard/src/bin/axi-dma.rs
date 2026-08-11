#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use axi_dma::{
    DmaReaderInterruptDriven, DmaWriterAsync, DoubleBufferHelper, SimpleDmaReader, SimpleDmaWriter,
};
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use embassy_time::{Duration, Ticker};
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{
    BootMode, Interrupt, cache, clocks, generic_interrupt_handler, gpio, gtc, uart,
};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 AXI DMA example --\n\r";
const BASE_ADDR_DMAC: usize = 0x4040_0000;
const AXI_DMA_TX_INTERRUPT: zynq7000_hal::SpiInterrupt = zynq7000_hal::SpiInterrupt::Pl2;
const AXI_DMA_RX_INTERRUPT: zynq7000_hal::SpiInterrupt = zynq7000_hal::SpiInterrupt::Pl3;
const BUFFERS_IN_OCM: bool = true;
const PRINT_BUF_PERIODIC: bool = false;
const BUFFER_LEN: usize = 128;
const DOUBLE_BUFFER_LEN: usize = 256;
/// A few [`DOUBLE_BUFFER_LEN`] chunks of slack for [`RX_DATA_PIPE`].
const RX_PIPE_LEN: usize = DOUBLE_BUFFER_LEN * 4;

static DOUBLE_BUFFER_HELPER: DoubleBufferHelper = DoubleBufferHelper::new();
static DMA_READ_TOKEN: once_cell::sync::OnceCell<axi_dma::DmaInterruptRxToken> =
    once_cell::sync::OnceCell::new();
static DMA_WRITE_TOKEN: once_cell::sync::OnceCell<axi_dma::DmaTxToken> =
    once_cell::sync::OnceCell::new();
/// RX data, from [`interrupt_handler_axi_dma_rx`] to the main loop's `try_read`.
static RX_DATA_PIPE: Pipe<CriticalSectionRawMutex, RX_PIPE_LEN> = Pipe::new();

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

    let mut gpio_pins = gpio::GpioPins::new(periphs.gpio);

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

    // If the reset timeouts, there is a high chance the base address is incorrect.
    let mut dma = axi_dma::DmaController::new(BASE_ADDR_DMAC)
        .expect("DMA controller reset timeout. Is the base address correct?");
    let mut writer = dma.take_simple_writer().unwrap();
    let mut reader = dma.take_simple_reader().unwrap();

    blocking_test(&mut writer, &mut reader);

    zynq7000_hal::register_interrupt(
        zynq7000_hal::Interrupt::Spi(AXI_DMA_TX_INTERRUPT),
        interrupt_handler_axi_dma_tx,
    );
    zynq7000_hal::register_interrupt(
        zynq7000_hal::Interrupt::Spi(AXI_DMA_RX_INTERRUPT),
        interrupt_handler_axi_dma_rx,
    );
    static DOUBLE_BUFFER: static_cell::ConstStaticCell<
        [cache::AlignedBuffer<DOUBLE_BUFFER_LEN>; 2],
    > = static_cell::ConstStaticCell::new([cache::AlignedBuffer([0; DOUBLE_BUFFER_LEN]); 2]);
    let [buf0, buf1] = DOUBLE_BUFFER.take();

    DOUBLE_BUFFER_HELPER
        .init(&mut buf0.0, &mut buf1.0)
        .expect("initializing double buffer helper failed");
    let mut isr_reader = unsafe { dma.steal_reader_interrupt_driven() };
    DMA_READ_TOKEN
        .set(isr_reader.token())
        .expect("initializing DMA read token failed");
    isr_reader
        .start(unsafe { DOUBLE_BUFFER_HELPER.buffer_0() })
        .expect("starting ISR driven DMA failed");

    let mut writer_async = unsafe { dma.steal_writer_async(0) }.unwrap();
    let token = writer_async.token();
    DMA_WRITE_TOKEN
        .set(token)
        .expect("initializing DMA write token faild");

    let mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
    let emio_led = gpio::Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), gpio::PinState::Low);
    spawner.spawn(led_task(mio_led, emio_led).unwrap());

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let mut rx_buf = [0u8; DOUBLE_BUFFER_LEN];
    #[unsafe(link_section = ".ocm.bss")]
    static TX_DATA_OCM: static_cell::ConstStaticCell<cache::AlignedBuffer<BUFFER_LEN>> =
        static_cell::ConstStaticCell::new(cache::AlignedBuffer([0; BUFFER_LEN]));
    let tx_data = TX_DATA_OCM.take();
    let mut start_value: u8 = 0;
    let mut once = true;
    loop {
        let mut value = start_value;
        for byte in tx_data.0.iter_mut() {
            *byte = value;
            value = value.wrapping_add(1);
        }

        // `tx_data` is in OCM (outer-non-cacheable), so only inner cleaning is needed.
        cache::clean_data_cache_range_inner(tx_data.0.as_ptr() as u32, tx_data.0.len())
            .expect("tx buffer not cache-line aligned");
        writer_async
            .write_all(&tx_data.0)
            .await
            .expect("async write failed");

        if PRINT_BUF_PERIODIC {
            log::debug!("ASYNC TX: Written {} bytes", BUFFER_LEN);
        }
        let pipe_was_full = RX_DATA_PIPE.is_full();
        value = start_value;

        // The data arrives in a pipe, cache invalidation is done in the reception ISR for us.
        if let Ok(n) = RX_DATA_PIPE.try_read(&mut rx_buf) {
            for &byte in &rx_buf[..n] {
                assert_eq!(byte, value, "DMA echo mismatch");
                value = value.wrapping_add(1);
            }
            if PRINT_BUF_PERIODIC {
                log::debug!("RX pipe: {} bytes: {:x?}", n, &rx_buf[..n]);
            }
        }
        if pipe_was_full {
            log::warn!("RX pipe was full");
        }
        if once {
            log::info!("Async DMA transfer OK");
            once = false;
        }
        start_value = start_value.wrapping_add(1);
        ticker.next().await; // Wait for the next cycle of the ticker
    }
}

#[embassy_executor::task]
async fn led_task(mut mio_led: gpio::Output, mut emio_led: gpio::Output) {
    let mut ticker = Ticker::every(Duration::from_millis(200));
    loop {
        mio_led.toggle();
        emio_led.toggle();
        ticker.next().await;
    }
}

/// Blocking smoke test. Arms S2MM first so it can't race the MM2S send, then blocks on
/// `wait()`. `BUFFERS_IN_OCM` picks OCM (`.ocm.bss`, outer-non-cacheable) vs. DDR buffers.
fn blocking_test(writer: &mut SimpleDmaWriter, reader: &mut SimpleDmaReader) {
    static RX_BUF_DDR: static_cell::ConstStaticCell<cache::AlignedBuffer<BUFFER_LEN>> =
        static_cell::ConstStaticCell::new(cache::AlignedBuffer([0; BUFFER_LEN]));
    static TX_BUF_DDR: static_cell::ConstStaticCell<cache::AlignedBuffer<BUFFER_LEN>> =
        static_cell::ConstStaticCell::new(cache::AlignedBuffer([0; BUFFER_LEN]));
    #[unsafe(link_section = ".ocm.bss")]
    static RX_BUF_OCM: static_cell::ConstStaticCell<cache::AlignedBuffer<BUFFER_LEN>> =
        static_cell::ConstStaticCell::new(cache::AlignedBuffer([0; BUFFER_LEN]));
    #[unsafe(link_section = ".ocm.bss")]
    static TX_BUF_OCM: static_cell::ConstStaticCell<cache::AlignedBuffer<BUFFER_LEN>> =
        static_cell::ConstStaticCell::new(cache::AlignedBuffer([0; BUFFER_LEN]));

    // Buffers are zeroed by the startup code's `.bss`/`.ocm.bss` clear, which runs before the
    // MMU/cache is enabled, so `take()` below hands out already-correct memory without a
    // runtime write. That avoids ever caching a dirty zero-fill that a later write-back could
    // use to clobber the freshly-DMA'd data.
    let rx_buf = if BUFFERS_IN_OCM {
        RX_BUF_OCM.take()
    } else {
        RX_BUF_DDR.take()
    };
    let read_transfer = reader.start_read(&mut rx_buf.0).unwrap();

    let tx_buf = if BUFFERS_IN_OCM {
        TX_BUF_OCM.take()
    } else {
        TX_BUF_DDR.take()
    };
    // Incrementing pattern, not a flat fill.
    for (i, byte) in tx_buf.0.iter_mut().enumerate() {
        *byte = i as u8;
    }
    if BUFFERS_IN_OCM {
        cache::clean_data_cache_range_inner(tx_buf.0.as_ptr() as u32, tx_buf.0.len())
    } else {
        cache::clean_data_cache_range(tx_buf.0.as_ptr() as u32, tx_buf.0.len())
    }
    .expect("tx buffer not cache-line aligned");
    writer.write(&tx_buf.0).unwrap();
    info!("wrote {} bytes via MM2S", tx_buf.0.len());

    let received = read_transfer.wait();
    // Invalidate the whole buffer, not just `received`: a short frame can leave its length not
    // cache-line-aligned.
    if BUFFERS_IN_OCM {
        cache::invalidate_data_cache_range_inner(received.as_ptr() as u32, BUFFER_LEN)
    } else {
        cache::invalidate_data_cache_range(received.as_ptr() as u32, BUFFER_LEN)
    }
    .expect("rx buffer not cache-line aligned");
    info!("received {} bytes via S2MM", received.len());
    let expected: [u8; BUFFER_LEN] = core::array::from_fn(|i| i as u8);
    assert_eq!(
        received,
        expected.as_slice(),
        "received data does not match the transmitted pattern"
    );
}

/// Writes as much of `data` into `RX_DATA_PIPE` as fits. `Pipe::try_write` only writes a
/// single contiguous chunk of its ring buffer, so it can return short purely because of the
/// buffer's wrap point, not because it's actually full. Loop past that, as the crate's own
/// docs advise.
fn write_to_rx_pipe(data: &[u8]) -> usize {
    let mut written = 0;
    while written < data.len() {
        match RX_DATA_PIPE.try_write(&data[written..]) {
            Ok(n) => written += n,
            Err(_) => break,
        }
    }
    written
}

pub fn interrupt_handler_axi_dma_tx() {
    // GIC/CPU interrupts are already globally enabled by `zynq7000_hal::init()` before this
    // handler is registered, and the token is only set a few lines later in `main()`, so the
    // interrupt can in principle fire before it's set.
    if let Some(write_token) = DMA_WRITE_TOKEN.get() {
        unsafe { DmaWriterAsync::on_interrupt(write_token) };
    }
}

pub fn interrupt_handler_axi_dma_rx() {
    let Some(read_token) = DMA_READ_TOKEN.get() else {
        return;
    };
    match unsafe {
        DmaReaderInterruptDriven::on_interrupt_double_buffered(read_token, &DOUBLE_BUFFER_HELPER)
    } {
        Ok(Some(data)) => {
            // Invalidate the whole slot, not just `data`: a short frame can leave its length not
            // cache-line-aligned.
            cache::invalidate_data_cache_range(data.as_ptr() as u32, DOUBLE_BUFFER_LEN)
                .expect("rx buffer not cache-line aligned");
            let written = write_to_rx_pipe(data);
            if written < data.len() {
                log::warn!("RX_DATA_PIPE full, dropped {} bytes", data.len() - written);
            }
        }
        Ok(None) => (),
        Err(e) => {
            log::warn!("DMA transfer error: {}", e);
        }
    }
}

#[zynq7000_rt::irq]
pub fn irq_handler() {
    // Safety: Called here once.
    let result = unsafe { generic_interrupt_handler() };
    if let Err(e) = result
        && e != Interrupt::Spurious
    {
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
