#![no_std]

use core::{cell::RefCell, panic::PanicInfo};

use embedded_io::Write;
use aarch32_cpu::asm::nop;
use critical_section::Mutex;
use zynq7000_hal::{time::Hertz, uart};

/// Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
pub const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_333);

/// One-shot cross-core hand-off cell.
///
/// CPU0 constructs a value and stores it with [Self::put] before releasing CPU1 via
/// `zynq7000_rt::smp::start_core1`. CPU1 retrieves it exactly once with [Self::take].
pub struct Handoff<T>(Mutex<RefCell<Option<T>>>);

impl<T> Handoff<T> {
    pub const fn new() -> Self {
        Self(Mutex::new(RefCell::new(None)))
    }

    pub fn put(&self, value: T) {
        critical_section::with(|cs| {
            *self.0.borrow_ref_mut(cs) = Some(value);
        });
    }

    /// Panics if [Self::put] was not called first.
    pub fn take(&self) -> T {
        critical_section::with(|cs| {
            self.0
                .borrow_ref_mut(cs)
                .take()
                .expect("Handoff value was not set before being taken")
        })
    }
}

impl<T> Default for Handoff<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[zynq7000_rt::irq]
pub fn irq_handler() {
    // Safety: Called here once.
    let result = unsafe { zynq7000_hal::generic_interrupt_handler() };
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

/// Panic handler. Steals the UART and writes directly to it rather than going through the log
/// pipe, since the executor draining that pipe may no longer be running. The write itself runs
/// inside a critical section, since the steal bypasses the pipe's own cross-core locking and
/// both cores could otherwise panic (or panic while the other is mid-write) at the same time.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_cs| {
        let mut uart = unsafe { uart::Uart::steal(uart::UartId::Uart1) };
        writeln!(uart, "panic: {info}\r").ok();
    });
    loop {}
}
