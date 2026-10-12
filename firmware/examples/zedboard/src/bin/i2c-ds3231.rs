//! PS I2C example using a DS3231 RTC.
//!
//! Runs a few one-shot transfer tests which cover the corner cases of the I2C driver. Then it uses
//! the `ds323x` driver crate to print the time and temperature once per second.
//!
//! External HW connections, using the MIO PMOD JE:
//!
//!  - SCL to JE4 (MIO 12)
//!  - SDA to JE1 (MIO 13)
//!  - GND to JE5
//!  - VCC to JE6 (3.3 V)
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use ds323x::{DateTimeAccess, Ds323x, NaiveDate};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::i2c::{NoAcknowledgeSource, Operation};
use log::{debug, error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{
    BootMode, clocks::Clocks, generic_interrupt_handler, gpio, gtc::GlobalTimerCounter, i2c, uart,
};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard I2C DS3231 example --\n\r";

const DS3231_ADDR: u8 = 0x68;
/// No device on the bus uses this address.
const UNUSED_ADDR: u8 = 0x10;
const NUM_REGS: usize = 19;
const REG_SECONDS: u8 = 0x00;
const REG_ALARM_1_SECONDS: u8 = 0x07;
/// Registers 0x00 to 0x10 are writable. Temperature registers are read-only.
const NUM_WRITABLE_REGS: usize = 0x11;
/// Alarm 1 and alarm 2 registers. Unlike control and status, they do not change on their own.
const NUM_ALARM_REGS: usize = 7;

/// Set to `true` to set the time once at startup, after the transfer tests.
const SET_TIME: bool = false;

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config::default()).unwrap();

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    let gpio_pins = gpio::Pins::new(periphs.gpio, gpio::Config::default());

    // Set up global timer counter and embassy time driver.
    let gtc = GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
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

    let clk_config =
        i2c::calculate_divisors(clocks.arm_clocks().cpu_1x_clk(), i2c::Speed::Normal100kHz)
            .unwrap();
    let mut i2c = i2c::I2c::new_with_mio(
        periphs.i2c_1,
        clk_config,
        (gpio_pins.mio.mio12, gpio_pins.mio.mio13),
    )
    .unwrap();

    run_transfer_tests(&mut i2c);

    let mut rtc = Ds323x::new_ds3231(i2c);
    if SET_TIME {
        let time = NaiveDate::from_ymd_opt(2026, 1, 1)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        rtc.set_datetime(&time).unwrap();
        info!("Set time to {}", time);
    }

    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        match rtc
            .datetime()
            .and_then(|time| rtc.temperature().map(|temp| (time, temp)))
        {
            Ok((time, temp)) => info!("{}, {:.2} °C", time, temp),
            Err(e) => error!("DS3231 read failed: {:?}", e),
        }
        ticker.next().await;
    }
}

/// One-shot tests for the corner cases of the driver. Check these on a logic analyzer as well.
fn run_transfer_tests(i2c: &mut i2c::I2c) {
    if let Err(e) = i2c.write(DS3231_ADDR, &[]) {
        error!("DS3231 not found at address {:#04x}: {:?}", DS3231_ADDR, e);
        return;
    }
    let mut report = Report::default();
    report.check("zero-length probe", Ok(()));

    report.check(
        "zero-length probe of unused address",
        match i2c.write(UNUSED_ADDR, &[]) {
            Err(i2c::Error::Nack(NoAcknowledgeSource::Address)) => Ok(()),
            other => Err(Failure::Unexpected(other)),
        },
    );

    let mut regs = [0; NUM_REGS];
    report.check(
        "read larger than FIFO",
        i2c.write_read(DS3231_ADDR, &[REG_SECONDS], &mut regs)
            .map_err(Failure::Bus),
    );
    debug!("registers: {:02x?}", regs);

    // The registers are written back unchanged, so the clock loses less than a second.
    report.check(
        "two writes larger than FIFO",
        i2c.transaction(
            DS3231_ADDR,
            &mut [
                Operation::Write(&[REG_SECONDS]),
                Operation::Write(&regs[..NUM_WRITABLE_REGS]),
            ],
        )
        .map_err(Failure::Bus),
    );

    let mut alarms = [0; NUM_ALARM_REGS];
    let (alarms_0, alarms_1) = alarms.split_at_mut(3);
    let result = i2c.transaction(
        DS3231_ADDR,
        &mut [
            Operation::Write(&[REG_ALARM_1_SECONDS]),
            Operation::Read(alarms_0),
            Operation::Read(alarms_1),
        ],
    );
    let expected: [u8; NUM_ALARM_REGS] = regs[REG_ALARM_1_SECONDS as usize..][..NUM_ALARM_REGS]
        .try_into()
        .unwrap();
    report.check(
        "two reads",
        match result {
            Ok(()) if alarms == expected => Ok(()),
            Ok(()) => Err(Failure::DataMismatch {
                read: alarms,
                expected,
            }),
            Err(e) => Err(Failure::Bus(e)),
        },
    );

    // The DS3231 continues at its internal register pointer.
    report.check(
        "plain read",
        i2c.read(DS3231_ADDR, &mut [0]).map_err(Failure::Bus),
    );

    report.check(
        "write after read is rejected",
        match i2c.transaction(
            DS3231_ADDR,
            &mut [Operation::Read(&mut [0]), Operation::Write(&[0])],
        ) {
            Err(i2c::Error::WriteAfterRead) => Ok(()),
            other => Err(Failure::Unexpected(other)),
        },
    );

    report.summary();
}

enum Failure {
    Bus(i2c::Error),
    Unexpected(Result<(), i2c::Error>),
    DataMismatch {
        read: [u8; NUM_ALARM_REGS],
        expected: [u8; NUM_ALARM_REGS],
    },
}

impl core::fmt::Display for Failure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Failure::Bus(e) => write!(f, "{:?}", e),
            Failure::Unexpected(result) => write!(f, "unexpected result {:?}", result),
            Failure::DataMismatch { read, expected } => {
                write!(f, "read {:02x?}, expected {:02x?}", read, expected)
            }
        }
    }
}

#[derive(Default)]
struct Report {
    passed: usize,
    total: usize,
}

impl Report {
    fn check(&mut self, name: &str, result: Result<(), Failure>) {
        self.total += 1;
        match result {
            Ok(()) => {
                self.passed += 1;
                info!("[ok] {}", name);
            }
            Err(failure) => error!("[failed] {}: {}", name, failure),
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
    error!("Panic: {info:?}");
    loop {}
}
