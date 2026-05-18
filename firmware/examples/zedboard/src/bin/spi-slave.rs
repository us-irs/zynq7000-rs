#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::{cell::RefCell, panic::PanicInfo, sync::atomic::AtomicU8};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_async::delay::DelayNs as _;
use embedded_io::Write;
use embedded_io_async::Read as _;
use log::info;
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000::spi::FifoWrite;
use zynq7000_hal::{
    BootMode, clocks, generic_interrupt_handler, gpio, gtc,
    log::asynch::UartLoggerRunner,
    spi::{self, SpiAsync},
    uart::{self, TxAsync},
};

use zynq7000_rt as _;

#[derive(Debug, PartialEq, Eq)]
pub enum TestType {
    Blocking,
    Async,
}

const TEST_TYPE: TestType = TestType::Async;

const INIT_STRING: &str = "-- Zynq 7000 SPI slave example --\n\r";

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
    Delay.delay_ms(1).await;

    let (tx, _rx) = uart.split();
    let tx_asynch = TxAsync::new(tx, true);
    let logger_runner =
        zynq7000_hal::log::asynch::init_with_uart_tx(log::LevelFilter::Trace, tx_asynch)
            .expect("Failed to initialize async logger");
    spawner.spawn(logger_task(logger_runner).unwrap());

    // This pin is used to select between a routing from SPI0 to OLED, or SPI0 to SPI1. Low selects
    // the SPI0 to OLED interface.
    let _spi_mux_pin =
        gpio::Output::new_for_emio(gpio_pins.emio.take(15).unwrap(), gpio::PinState::High);

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

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

    // Enable loopback.
    spi::enable_spi0_to_spi1_loopback();

    let mut spi_slave = spi::SpiSlave::new(
        spi::SpiId::Spi1,
        periphs.spi_1,
        spi::SlaveConfig {
            mode: spi::MODE_0,
            enable_modefail: true,
        },
    );
    zynq7000_hal::register_interrupt(spi_slave.interrupt_id(), spi_interrupt_handler);

    let spi_master = spi::Spi::new_for_emio(
        periphs.spi_0,
        spi::Config::new(
            // 10 MHz maximum rating of the sensor.
            zynq7000::spi::BaudDivSel::By64,
            // l3gd20::MODE,
            embedded_hal::spi::MODE_0,
            spi::SlaveSelectConfig::AutoCsManualStart,
        ),
    )
    .unwrap();
    let cs_pin = spi::ChipSelectPin::new(spi_master.id(), spi::ChipSelect::Slave0);

    // We pre-load the FIFO with a fixed, known sequence.
    for i in 0..spi::FIFO_DEPTH / 2 {
        spi_slave.write_tx_data(i as u8);
    }
    log::info!("SPI slave initialized, enabling interrupts and peripheral");
    spi_slave.enable_interrupts(true);
    spi_slave.enable();

    static RX_DATA_PIPE: static_cell::ConstStaticCell<
        embassy_sync::pipe::Pipe<CriticalSectionRawMutex, 256>,
    > = static_cell::ConstStaticCell::new(embassy_sync::pipe::Pipe::new());

    let pipe = RX_DATA_PIPE.take();
    let (mut slave_rx_reader, writer) = pipe.split();
    critical_section::with(|cs| {
        RX_DATA_WRITER.borrow(cs).replace(Some(writer));
    });

    if TEST_TYPE == TestType::Blocking {
        let mut spi_master =
            embedded_hal_bus::spi::ExclusiveDevice::new(spi_master, cs_pin, embassy_time::Delay)
                .expect("creating exlusive SPI master failed");

        let current_rx_val = blocking_tests_small(&mut spi_master, &mut slave_rx_reader).await;
        log::info!("blocking tests with small data blocks done");
        Delay.delay_ms(10).await;
        blocking_tests_large(&mut spi_master, current_rx_val, &mut slave_rx_reader).await;
        log::info!("blocking tests with large data blocks done");
    } else {
        let spi_master = SpiAsync::new(spi_master);
        let mut spi_master =
            embedded_hal_bus::spi::ExclusiveDevice::new(spi_master, cs_pin, embassy_time::Delay)
                .expect("creating exlusive SPI master failed");
        let current_rx_val =
            blocking_tests_small_async(&mut spi_master, &mut slave_rx_reader).await;
        log::info!("async tests with small data blocks done");
        Delay.delay_ms(10).await;
        blocking_tests_large_async(&mut spi_master, current_rx_val, &mut slave_rx_reader).await;
        log::info!("blocking tests with large data blocks done");
    }

    log::info!("SPI slave tests done");

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        ticker.next().await; // Wait for the next cycle of the ticker
    }
}

pub async fn blocking_tests_small(
    spi_master: &mut impl embedded_hal::spi::SpiDevice,
    slave_rx_reader: &mut embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, 256>,
) -> u8 {
    let mut tx_buf = [0; 64];
    let mut buf = [0; 64];

    // Write test.
    // We should read back 0,1,2,3 here but the sent values are ignored.
    spi_master.write(&[1, 2, 3, 4]).unwrap();
    slave_rx_reader.read_exact(&mut buf[0..4]).await.unwrap();
    assert_eq!(
        &buf[0..4],
        &[1, 2, 3, 4],
        "slave did not receive the expected data"
    );

    // Read test.
    spi_master.read(&mut buf[0..4]).unwrap();
    // Now we expect 4,5,6,7 sent by the SPI slave
    assert_eq!(
        &buf[0..4],
        &[4, 5, 6, 7],
        "slave did not send the expected data"
    );
    // Slave should receive dummy data
    slave_rx_reader.read_exact(&mut buf[0..4]).await.unwrap();
    assert_eq!(
        &buf[0..4],
        &[0, 0, 0, 0],
        "slave did not receive the expected data"
    );

    // Transfer test.
    for (i, item) in tx_buf.iter_mut().enumerate().take(16) {
        *item = (i * 2) as u8;
    }
    spi_master
        .transfer(&mut buf[0..16], &tx_buf[0..16])
        .unwrap();
    for i in 8..24 {
        assert_eq!(
            buf[i - 8],
            i as u8,
            "slave did not receive the expected data"
        );
    }
    slave_rx_reader.read_exact(&mut buf[0..16]).await.unwrap();
    for (i, item) in buf.iter().enumerate().take(16) {
        assert_eq!(*item, (i * 2) as u8, "slave did not send the expected data");
    }

    // Transfer in place test.
    for (i, item) in buf.iter_mut().enumerate().take(16) {
        *item = (i * 2) as u8;
    }
    spi_master.transfer_in_place(&mut buf[0..16]).unwrap();
    for i in 24..40 {
        assert_eq!(
            buf[i - 24],
            i as u8,
            "slave did not receive the expected data"
        );
    }
    slave_rx_reader.read_exact(&mut buf[0..16]).await.unwrap();
    for (i, item) in buf.iter().enumerate().take(16) {
        assert_eq!(*item, (i * 2) as u8, "slave did not send the expected data");
    }
    40
}

pub async fn blocking_tests_large(
    spi_master: &mut impl embedded_hal::spi::SpiDevice,
    mut current_rx_val: u8,
    slave_rx_reader: &mut embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, 256>,
) -> u8 {
    const BUF_LEN: usize = 164;
    let mut buf = [0; BUF_LEN];

    // Write test.
    for (i, item) in buf.iter_mut().enumerate() {
        *item = i as u8;
    }
    spi_master.write(&buf).unwrap();
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for (i, item) in buf.iter().enumerate() {
        assert_eq!(*item, i as u8, "slave did not receive the expected data");
    }
    current_rx_val = current_rx_val.wrapping_add(buf.len() as u8);

    // Read test.
    spi_master.read(&mut buf).unwrap();
    for item in &buf {
        assert_eq!(
            *item, current_rx_val,
            "slave did not send the expected data"
        );
        current_rx_val = current_rx_val.wrapping_add(1);
    }
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for item in &buf {
        assert_eq!(*item, 0, "slave did not receive the expected data");
    }

    // Transfer test.
    let mut tx_buf = [0; BUF_LEN];
    for (i, item) in tx_buf.iter_mut().enumerate() {
        *item = i as u8;
    }
    spi_master.transfer(&mut buf, &tx_buf).unwrap();
    for item in &buf {
        assert_eq!(
            *item, current_rx_val,
            "slave did not send the expected data"
        );
        current_rx_val = current_rx_val.wrapping_add(1);
    }
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for (i, item) in buf.iter().enumerate() {
        assert_eq!(*item, i as u8, "slave did not receive the expected data");
    }

    // Transfer in place test.
    for (i, item) in buf.iter_mut().enumerate() {
        *item = i as u8;
    }
    spi_master.transfer_in_place(&mut buf).unwrap();
    for item in &buf {
        assert_eq!(
            *item, current_rx_val,
            "slave did not send the expected data"
        );
        current_rx_val = current_rx_val.wrapping_add(1);
    }
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for (i, item) in buf.iter().enumerate() {
        assert_eq!(*item, i as u8, "slave did not receive the expected data");
    }
    current_rx_val
}

pub async fn blocking_tests_small_async(
    spi_master: &mut impl embedded_hal_async::spi::SpiDevice,
    slave_rx_reader: &mut embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, 256>,
) -> u8 {
    let mut tx_buf = [0; 64];
    let mut buf = [0; 64];

    // Write test.
    // We should read back 0,1,2,3 here but the sent values are ignored.
    spi_master.write(&[1, 2, 3, 4]).await.unwrap();
    slave_rx_reader.read_exact(&mut buf[0..4]).await.unwrap();
    assert_eq!(
        &buf[0..4],
        &[1, 2, 3, 4],
        "slave did not receive the expected data"
    );

    // Read test.
    spi_master.read(&mut buf[0..4]).await.unwrap();
    // Now we expect 4,5,6,7 sent by the SPI slave
    assert_eq!(
        &buf[0..4],
        &[4, 5, 6, 7],
        "slave did not send the expected data"
    );
    // Slave should receive dummy data
    slave_rx_reader.read_exact(&mut buf[0..4]).await.unwrap();
    assert_eq!(
        &buf[0..4],
        &[0, 0, 0, 0],
        "slave did not receive the expected data"
    );

    // Transfer test.
    for (i, item) in tx_buf.iter_mut().enumerate().take(16) {
        *item = (i * 2) as u8;
    }
    spi_master
        .transfer(&mut buf[0..16], &tx_buf[0..16])
        .await
        .unwrap();
    for i in 8..24 {
        assert_eq!(
            buf[i - 8],
            i as u8,
            "slave did not receive the expected data"
        );
    }
    slave_rx_reader.read_exact(&mut buf[0..16]).await.unwrap();
    for (i, item) in buf.iter().enumerate().take(16) {
        assert_eq!(*item, (i * 2) as u8, "slave did not send the expected data");
    }

    // Transfer in place test.
    for (i, item) in buf.iter_mut().enumerate().take(16) {
        *item = (i * 2) as u8;
    }
    spi_master.transfer_in_place(&mut buf[0..16]).await.unwrap();
    for i in 24..40 {
        assert_eq!(
            buf[i - 24],
            i as u8,
            "slave did not receive the expected data"
        );
    }
    slave_rx_reader.read_exact(&mut buf[0..16]).await.unwrap();
    for (i, item) in buf.iter().enumerate().take(16) {
        assert_eq!(*item, (i * 2) as u8, "slave did not send the expected data");
    }
    40
}

pub async fn blocking_tests_large_async(
    spi_master: &mut impl embedded_hal_async::spi::SpiDevice,
    mut current_rx_val: u8,
    slave_rx_reader: &mut embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, 256>,
) -> u8 {
    const BUF_LEN: usize = 164;
    let mut buf = [0; BUF_LEN];

    // Write test.
    for (i, item) in buf.iter_mut().enumerate() {
        *item = i as u8;
    }
    spi_master.write(&buf).await.unwrap();
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for (i, &item) in buf.iter().enumerate() {
        assert_eq!(item, i as u8, "slave did not receive the expected data");
    }
    current_rx_val = current_rx_val.wrapping_add(buf.len() as u8);

    // Read test.
    spi_master.read(&mut buf).await.unwrap();
    for &item in &buf {
        assert_eq!(item, current_rx_val, "slave did not send the expected data");
        current_rx_val = current_rx_val.wrapping_add(1);
    }
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for &item in &buf {
        assert_eq!(item, 0, "slave did not receive the expected data");
    }

    // Transfer test.
    let mut tx_buf = [0; BUF_LEN];
    for (i, item) in tx_buf.iter_mut().enumerate() {
        *item = i as u8;
    }
    spi_master.transfer(&mut buf, &tx_buf).await.unwrap();
    for &item in &buf {
        assert_eq!(item, current_rx_val, "slave did not send the expected data");
        current_rx_val = current_rx_val.wrapping_add(1);
    }
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for (i, &item) in buf.iter().enumerate() {
        assert_eq!(item, i as u8, "slave did not receive the expected data");
    }

    // Transfer in place test.
    for (i, item) in buf.iter_mut().enumerate() {
        *item = i as u8;
    }
    spi_master.transfer_in_place(&mut buf).await.unwrap();
    for &item in &buf {
        assert_eq!(item, current_rx_val, "slave did not send the expected data");
        current_rx_val = current_rx_val.wrapping_add(1);
    }
    slave_rx_reader
        .read_exact(&mut buf[0..BUF_LEN])
        .await
        .unwrap();
    for (i, &item) in buf.iter().enumerate() {
        assert_eq!(item, i as u8, "slave did not receive the expected data");
    }
    current_rx_val
}

#[embassy_executor::task]
pub async fn logger_task(mut logger_task: UartLoggerRunner) {
    logger_task.run().await;
}

static RX_DATA_WRITER: critical_section::Mutex<
    RefCell<Option<embassy_sync::pipe::Writer<'static, CriticalSectionRawMutex, 256>>>,
> = critical_section::Mutex::new(RefCell::new(None));

unsafe fn spi_interrupt_handler() {
    static CURRENT_TX_VAL: AtomicU8 = AtomicU8::new((spi::FIFO_DEPTH / 2) as u8);

    let mut buf = [0; 64];
    let mut current_val = CURRENT_TX_VAL.load(core::sync::atomic::Ordering::Relaxed);
    let mut spi = unsafe { spi::SpiLowLevel::steal(spi::SpiId::Spi1) };

    let status = spi.read_interrupt_status();
    spi.write_interrupt_status(status);

    let mut index = 0;
    while spi.read_interrupt_status().rx_not_empty() && index < buf.len() {
        buf[index] = spi.read_rx_data().value();
        index += 1;
    }
    while !spi.read_interrupt_status().tx_full() {
        spi.write_tx_data(FifoWrite::new(current_val));
        current_val = current_val.wrapping_add(1);
    }
    CURRENT_TX_VAL.store(current_val, core::sync::atomic::Ordering::Relaxed);

    critical_section::with(|cs| {
        let opt_writer = RX_DATA_WRITER.borrow(cs).borrow();
        if let Some(writer) = opt_writer.as_ref() {
            // In a real application, you would read the received data from the SPI peripheral here.
            // For this example, we just write a dummy value to the pipe to demonstrate the concept.
            let _ = writer.try_write(&buf[0..index]);
        }
    });
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
pub fn irq_handler() {
    // Safety: Called here once.
    let result = unsafe { generic_interrupt_handler() };
    if let Err(e) = result {
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
    let mut uart = unsafe { uart::Uart::steal(uart::UartId::Uart1) };
    writeln!(uart, "panic: {}\r", info).ok();
    loop {}
}
