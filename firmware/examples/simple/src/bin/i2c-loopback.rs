//! PS I2C loopback example.
//!
//! I2C0 is the master and I2C1 is a slave, connected internally with the SLCR loopback. No
//! external wiring is required. The test sets the data the slave sends and checks the data the
//! slave received.
//!
//! Only the UART used for logging is board specific.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use arbitrary_int::u7;
use core::{
    panic::PanicInfo,
    sync::atomic::{AtomicU32, Ordering},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use embedded_hal::i2c::{NoAcknowledgeSource, Operation};
use log::{error, info};
use zynq7000::i2c::{InterruptControl, ReadWrite};
use zynq7000_hal::{
    BootMode, clocks::Clocks, generic_interrupt_handler, gpio, i2c, priv_tim::CpuPrivateTimer,
    time::Hertz, uart,
};

use zynq7000_rt as _;

const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);
const INIT_STRING: &str = "-- Zynq 7000 Zedboard I2C loopback example --\n\r";

const SLAVE_ID: i2c::I2cId = i2c::I2cId::I2c1;
const SLAVE_ADDR: u8 = 0x42;
const UNUSED_ADDR: u8 = 0x10;
const BUF_SIZE: usize = 64;
/// The slave refills the TX FIFO when the DATA interrupt signals two bytes left.
const TX_REFILL: usize = i2c::FIFO_DEPTH - 2;

/// Bytes the slave sends when the master reads. Bytes which are queued but not read by the
/// master are dropped.
static SLAVE_TX: Pipe<CriticalSectionRawMutex, BUF_SIZE> = Pipe::new();
/// Bytes the slave received.
static SLAVE_RX: Pipe<CriticalSectionRawMutex, BUF_SIZE> = Pipe::new();
/// Number of slave interrupts, to spot interrupt storms.
static SLAVE_IRQS: AtomicU32 = AtomicU32::new(0);
/// Incremented whenever the slave sees the STOP of a transfer.
static SLAVE_TRANSFERS: AtomicU32 = AtomicU32::new(0);

#[zynq7000_rt::entry]
fn main() -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config::default()).unwrap();

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    let gpio_pins = gpio::Pins::new(periphs.gpio, gpio::Config::default());

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

    i2c::enable_i2c0_to_i2c1_loopback();

    let clk_config =
        i2c::calculate_divisors(clocks.arm_clocks().cpu_1x_clk(), i2c::Speed::Normal100kHz)
            .unwrap();
    let mut slave = i2c::slave::I2cSlave::new_with_emio(
        periphs.i2c_1,
        i2c::slave::Config {
            address: u7::new(SLAVE_ADDR),
            clk_cfg: clk_config,
        },
    )
    .unwrap();
    // Keep SCL low instead of overflowing when the interrupt handler is late.
    slave.set_hold(true);
    zynq7000_hal::register_interrupt(slave.interrupt_id(), i2c_slave_interrupt_handler);
    slave.enable_interrupts(
        InterruptControl::new_with_raw_value(0)
            .with_complete(true)
            .with_data(true)
            .with_nack(true),
    );

    let mut master = i2c::I2c::new_with_emio(periphs.i2c_0, clk_config).unwrap();
    let mut timer = CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();
    run_transfer_tests(&mut master, &mut timer);

    loop {
        nop();
    }
}

fn run_transfer_tests(master: &mut i2c::I2c, timer: &mut CpuPrivateTimer) {
    let mut report = Report::default();
    let mut write_data = [0; 24];
    for (i, byte) in write_data.iter_mut().enumerate() {
        *byte = i as u8 * 3 + 1;
    }
    let mut tx_data = [0; BUF_SIZE];
    for (i, byte) in tx_data.iter_mut().enumerate() {
        *byte = (i as u8) ^ 0xA5;
    }

    reset_queues();
    report.check(
        "zero-length probe",
        2,
        slave_transfer(timer, || master.write(SLAVE_ADDR, &[])).and_then(|_| compare_slave_rx(&[])),
    );
    report.check(
        "zero-length probe of unused address",
        0,
        match master.write(UNUSED_ADDR, &[]) {
            Err(i2c::Error::Nack(NoAcknowledgeSource::Address)) => Ok(()),
            other => Err(Failure::Unexpected(other)),
        },
    );

    reset_queues();
    report.check(
        "write larger than FIFO",
        3,
        slave_transfer(timer, || master.write(SLAVE_ADDR, &write_data[..20]))
            .and_then(|_| compare_slave_rx(&write_data[..20])),
    );

    reset_queues();
    queue_slave_tx(&tx_data);
    let mut read = [0; 24];
    report.check(
        "read larger than FIFO",
        4,
        slave_transfer(timer, || master.read(SLAVE_ADDR, &mut read))
            .and_then(|_| compare(&read, &tx_data[..24])),
    );

    reset_queues();
    queue_slave_tx(&tx_data);
    let mut read = [0; 4];
    report.check(
        "write_read within FIFO",
        4,
        slave_transfer(timer, || {
            master.write_read(SLAVE_ADDR, &write_data[..2], &mut read)
        })
        .and_then(|_| compare_slave_rx(&write_data[..2]))
        .and_then(|_| compare(&read, &tx_data[..4])),
    );

    reset_queues();
    report.check(
        "two writes",
        3,
        slave_transfer(timer, || {
            master.transaction(
                SLAVE_ADDR,
                &mut [
                    Operation::Write(&write_data[..5]),
                    Operation::Write(&write_data[5..20]),
                ],
            )
        })
        .and_then(|_| compare_slave_rx(&write_data[..20])),
    );

    reset_queues();
    queue_slave_tx(&tx_data);
    let mut read = [0; 20];
    let (read_0, read_1) = read.split_at_mut(3);
    let result = slave_transfer(timer, || {
        master.transaction(
            SLAVE_ADDR,
            &mut [
                Operation::Write(&write_data[..1]),
                Operation::Read(read_0),
                Operation::Read(read_1),
            ],
        )
    });
    report.check(
        "two reads",
        5,
        result
            .and_then(|_| compare_slave_rx(&write_data[..1]))
            .and_then(|_| compare(&read, &tx_data[..20])),
    );

    report.check(
        "write after read is rejected",
        0,
        match master.transaction(
            SLAVE_ADDR,
            &mut [Operation::Read(&mut [0]), Operation::Write(&[0])],
        ) {
            Err(i2c::Error::WriteAfterRead) => Ok(()),
            other => Err(Failure::Unexpected(other)),
        },
    );

    report.summary();
}

/// Runs a master transfer and waits until the slave interrupt handler has seen its end.
fn slave_transfer(
    timer: &mut CpuPrivateTimer,
    transfer: impl FnOnce() -> Result<(), i2c::Error>,
) -> Result<(), Failure> {
    let transfers = SLAVE_TRANSFERS.load(Ordering::Relaxed);
    transfer().map_err(Failure::Bus)?;
    // 10 ms timeout.
    for _ in 0..1000 {
        if SLAVE_TRANSFERS.load(Ordering::Relaxed) != transfers {
            return Ok(());
        }
        timer.delay_us(10);
    }
    Err(Failure::SlaveTimeout)
}

fn compare(read: &[u8], expected: &[u8]) -> Result<(), Failure> {
    if read.len() != expected.len() {
        return Err(Failure::LengthMismatch {
            read: read.len(),
            expected: expected.len(),
        });
    }
    match read.iter().zip(expected).position(|(r, e)| r != e) {
        None => Ok(()),
        Some(offset) => Err(Failure::DataMismatch {
            offset,
            read: read[offset],
            expected: expected[offset],
        }),
    }
}

fn reset_queues() {
    SLAVE_TX.clear();
    SLAVE_RX.clear();
}

/// Queues the bytes the slave sends on the next read.
fn queue_slave_tx(tx: &[u8]) {
    SLAVE_TX
        .try_write(tx)
        .expect("writing TX data to queue failed");
}

fn compare_slave_rx(expected: &[u8]) -> Result<(), Failure> {
    let mut rx = [0; BUF_SIZE];
    let len = SLAVE_RX.try_read(&mut rx).unwrap_or(0);
    compare(&rx[..len], expected)
}

/// Controller behavior this relies on, TRM p.643 f.:
///
/// - Received bytes are in the RX FIFO. DATA is set when only two free slots are left, COMP at
///   the end of the write, which is the STOP or a repeated start.
/// - When the master reads, the slave holds SCL low and sets DATA until the TX FIFO is filled.
///   DATA is set again when only two bytes are left. [zynq7000::i2c::Status::slave_rw] shows the
///   direction.
/// - A read ends with NACK. COMP is only set if the TX FIFO happened to be empty.
fn i2c_slave_interrupt_handler() {
    // Safety: The slave was configured in main and is only used here afterwards.
    let mut slave = unsafe { i2c::slave::I2cSlave::steal(SLAVE_ID) };
    SLAVE_IRQS.fetch_add(1, Ordering::Relaxed);
    let isr = slave.read_interrupt_status();
    slave.clear_interrupts(isr);

    // Bytes written by the master. Collected regardless of the R/W bit, because after a
    // write_read the bit may already show the read while written bytes are still in the FIFO.
    let mut rx = [0; i2c::FIFO_DEPTH];
    let mut rx_len = 0;
    while slave.read_status().rx_valid() && rx_len < rx.len() {
        rx[rx_len] = slave.read_data();
        rx_len += 1;
    }
    let _ = SLAVE_RX.try_write(&rx[..rx_len]);
    if slave.read_status().slave_rw() == ReadWrite::Read {
        if isr.nack() {
            // The master ends a read with a NACK. Bytes left in the FIFO are dropped.
            slave.clear_fifo();
        } else if isr.data() {
            // The FIFO needs data.
            let mut tx = [0; TX_REFILL];
            let len = SLAVE_TX.try_read(&mut tx).unwrap_or(0);
            tx[..len].iter().for_each(|byte| slave.write_data(*byte));
        }
    }
    // COMP is also set at a repeated start, where the bus stays active.
    if (isr.complete() || isr.nack()) && !slave.read_status().bus_active() {
        SLAVE_TRANSFERS.fetch_add(1, Ordering::Relaxed);
    }
}

enum Failure {
    Bus(i2c::Error),
    Unexpected(Result<(), i2c::Error>),
    DataMismatch {
        offset: usize,
        read: u8,
        expected: u8,
    },
    LengthMismatch {
        read: usize,
        expected: usize,
    },
    SlaveTimeout,
    TooManyInterrupts {
        max: u32,
    },
}

impl core::fmt::Display for Failure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Failure::Bus(e) => write!(f, "{:?}", e),
            Failure::Unexpected(result) => write!(f, "unexpected result {:?}", result),
            Failure::DataMismatch {
                offset,
                read,
                expected,
            } => write!(
                f,
                "byte {}: read {:#04x}, expected {:#04x}",
                offset, read, expected
            ),
            Failure::LengthMismatch { read, expected } => {
                write!(f, "length {}, expected {}", read, expected)
            }
            Failure::TooManyInterrupts { max } => {
                write!(f, "too many slave interrupts, expected at most {}", max)
            }
            Failure::SlaveTimeout => write!(f, "slave did not see the end of the transfer"),
        }
    }
}

#[derive(Default)]
struct Report {
    passed: usize,
    total: usize,
    slave_irqs: u32,
}

impl Report {
    /// `max_irqs` catches interrupt storms. The expected counts follow from the controller
    /// behavior described at [i2c_slave_interrupt_handler], one more is allowed.
    fn check(&mut self, name: &str, max_irqs: u32, result: Result<(), Failure>) {
        self.total += 1;
        let slave_irqs = SLAVE_IRQS.load(Ordering::Relaxed);
        let irqs = slave_irqs - self.slave_irqs;
        self.slave_irqs = slave_irqs;
        let result = result.and_then(|_| {
            if irqs > max_irqs {
                return Err(Failure::TooManyInterrupts { max: max_irqs });
            }
            Ok(())
        });
        match result {
            Ok(()) => {
                self.passed += 1;
                info!("[ok] {} ({} slave interrupts)", name, irqs);
            }
            Err(failure) => error!("[failed] {} ({} slave interrupts): {}", name, irqs, failure),
        }
    }

    fn summary(&self) {
        if self.passed == self.total {
            info!("all {} transfer tests passed", self.total);
        } else {
            error!(
                "{} of {} transfer tests failed",
                self.total - self.passed,
                self.total
            );
        }
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
    error!("Panic: {info:?}");
    loop {}
}
