//! # Simple logging providers

use core::sync::atomic::{AtomicBool, AtomicU8};

static LOGGER_INIT_DONE: AtomicBool = AtomicBool::new(false);

const LOG_SEL_LOCKED: u8 = 1;
const LOG_SEL_UNSAFE_SINGLE_CORE: u8 = 2;

static LOG_SEL: AtomicU8 = AtomicU8::new(0);

/// Blocking UART loggers.
pub mod uart_blocking {
    use super::*;
    use core::cell::{RefCell, UnsafeCell};
    use embedded_io::Write as _;

    use critical_section::Mutex;
    use log::{LevelFilter, Log, set_logger, set_max_level};

    use crate::uart::Uart;

    pub struct UartLoggerBlocking(Mutex<RefCell<Option<Uart>>>);

    unsafe impl Send for UartLoggerBlocking {}
    unsafe impl Sync for UartLoggerBlocking {}

    static UART_LOGGER_BLOCKING: UartLoggerBlocking =
        UartLoggerBlocking(Mutex::new(RefCell::new(None)));

    /// Initialize the logger with a blocking UART instance.
    ///
    /// This is a blocking logger which performs a write inside a critical section. This logger is
    /// thread-safe, but interrupts will be disabled while the logger is writing to the UART.
    ///
    /// For async applications, it is strongly recommended to use the asynchronous ring buffer
    /// logger instead.
    pub fn init_with_locks(uart: Uart, level: LevelFilter) {
        if LOGGER_INIT_DONE.swap(true, core::sync::atomic::Ordering::Relaxed) {
            return;
        }
        LOG_SEL.swap(LOG_SEL_LOCKED, core::sync::atomic::Ordering::Relaxed);
        critical_section::with(|cs| {
            let inner = UART_LOGGER_BLOCKING.0.borrow(cs);
            inner.replace(Some(uart));
        });
        set_logger(&UART_LOGGER_BLOCKING).unwrap();
        // Adjust as needed
        set_max_level(level);
    }

    impl log::Log for UartLoggerBlocking {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            critical_section::with(|cs| {
                let mut opt_logger = self.0.borrow(cs).borrow_mut();
                if opt_logger.is_none() {
                    return;
                }
                let logger = opt_logger.as_mut().unwrap();
                writeln!(logger, "{} - {}\r", record.level(), record.args()).unwrap();
            })
        }

        fn flush(&self) {
            critical_section::with(|cs| {
                let mut opt_logger = self.0.borrow(cs).borrow_mut();
                if opt_logger.is_none() {
                    return;
                }
                let logger = opt_logger.as_mut().unwrap();
                logger.flush().unwrap();
            });
        }
    }

    pub struct UartLoggerWithBusyFlag {
        busy: AtomicBool,
        skip_in_isr: AtomicBool,
        uart: UnsafeCell<Option<Uart>>,
    }

    unsafe impl Send for UartLoggerWithBusyFlag {}
    unsafe impl Sync for UartLoggerWithBusyFlag {}

    static UART_LOGGER_UNSAFE_SINGLE_THREAD: UartLoggerWithBusyFlag = UartLoggerWithBusyFlag {
        busy: AtomicBool::new(false),
        skip_in_isr: AtomicBool::new(false),
        uart: UnsafeCell::new(None),
    };

    struct UartGuard<'lock>(&'lock AtomicBool);

    impl<'lock> UartGuard<'lock> {
        pub fn new(flag: &'lock AtomicBool) -> Option<Self> {
            let proc_mode = aarch32_cpu::register::Cpsr::read().mode().ok()?;

            let is_irq = proc_mode == aarch32_cpu::register::cpsr::ProcessorMode::Fiq
                || proc_mode == aarch32_cpu::register::cpsr::ProcessorMode::Irq;

            // For IRQs, only try once.
            if is_irq {
                if UART_LOGGER_UNSAFE_SINGLE_THREAD
                    .skip_in_isr
                    .load(core::sync::atomic::Ordering::Relaxed)
                {
                    return None;
                }
                if flag.swap(true, core::sync::atomic::Ordering::AcqRel) {
                    return None;
                }
                return Some(Self(flag));
            }

            // For threaded code, spinning is allowed.
            while flag.swap(true, core::sync::atomic::Ordering::AcqRel) {}
            Some(Self(flag))
        }
    }

    impl Drop for UartGuard<'_> {
        fn drop(&mut self) {
            self.0.store(false, core::sync::atomic::Ordering::Release);
        }
    }

    /// Initialize the logger with a blocking UART instance which spins on a busy flag in threaded
    /// mode, and does not log in interrupt contexts if the main task was busy with logging.
    ///
    /// It should be noted that this is still a blocking logger, and using it in an ISR might
    /// invalidate application logic and introduce problematic delays in the system.
    ///
    /// Therefore, the initialization also allows skipping logging in ISRs completely.
    pub fn init_with_busy_flag(uart: Uart, level: LevelFilter, skip_in_isr: bool) {
        if LOGGER_INIT_DONE.swap(true, core::sync::atomic::Ordering::Relaxed) {
            return;
        }
        LOG_SEL.swap(
            LOG_SEL_UNSAFE_SINGLE_CORE,
            core::sync::atomic::Ordering::Relaxed,
        );
        let opt_uart = unsafe { &mut *UART_LOGGER_UNSAFE_SINGLE_THREAD.uart.get() };
        opt_uart.replace(uart);

        UART_LOGGER_UNSAFE_SINGLE_THREAD
            .skip_in_isr
            .store(skip_in_isr, core::sync::atomic::Ordering::Relaxed);

        set_logger(&UART_LOGGER_UNSAFE_SINGLE_THREAD).unwrap();
        set_max_level(level); // Adjust as needed
    }

    impl log::Log for UartLoggerWithBusyFlag {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            let guard = UartGuard::new(&self.busy);
            if guard.is_none() {
                return;
            }

            let uart_mut = unsafe { &mut *self.uart.get() }.as_mut();
            if uart_mut.is_none() {
                return;
            }

            writeln!(
                uart_mut.unwrap(),
                "{} - {}\r",
                record.level(),
                record.args()
            )
            .unwrap();
        }

        fn flush(&self) {
            let guard = UartGuard::new(&self.busy);
            if guard.is_none() {
                return;
            }

            let uart_mut = unsafe { &mut *self.uart.get() }.as_mut();
            if uart_mut.is_none() {
                return;
            }
            uart_mut.unwrap().flush().unwrap();
        }
    }

    // Flush the selected logger instance.
    pub fn flush() {
        match LOG_SEL.load(core::sync::atomic::Ordering::Relaxed) {
            val if val == LOG_SEL_LOCKED => UART_LOGGER_BLOCKING.flush(),
            val if val == LOG_SEL_UNSAFE_SINGLE_CORE => UART_LOGGER_UNSAFE_SINGLE_THREAD.flush(),
            _ => (),
        }
    }
}

/// Logger module which logs into a pipe to allow asynchronous logging handling.
pub mod asynch {
    use core::fmt::Write as _;

    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use log::{LevelFilter, set_logger, set_max_level};

    use crate::uart::TxAsync;

    /// Logger implementation which logs frames via a ring buffer and sends the frame sizes
    /// as messages.
    ///
    /// The logger does not require allocation and reserves a generous amount of 4096 bytes for
    /// log data. This should be sufficient for most logging needs.
    pub struct Logger {
        pipe: core::cell::RefCell<
            Option<embassy_sync::pipe::Writer<'static, CriticalSectionRawMutex, 4096>>,
        >,
        buf: critical_section::Mutex<core::cell::RefCell<heapless::String<4096>>>,
    }

    unsafe impl Send for Logger {}
    unsafe impl Sync for Logger {}

    static LOGGER: Logger = Logger {
        pipe: core::cell::RefCell::new(None),
        buf: critical_section::Mutex::new(core::cell::RefCell::new(heapless::String::new())),
    };

    static PIPE: static_cell::ConstStaticCell<
        embassy_sync::pipe::Pipe<CriticalSectionRawMutex, 4096>,
    > = static_cell::ConstStaticCell::new(embassy_sync::pipe::Pipe::new());

    impl log::Log for Logger {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            if self.pipe.borrow().is_none() {
                return;
            }
            critical_section::with(|cs| {
                let mut buf = self.buf.borrow(cs).borrow_mut();
                buf.clear();
                let _ = writeln!(buf, "{} - {}\r", record.level(), record.args());

                let mut written = 0;

                let pipe_writer_ref = self.pipe.borrow();
                let pipe_writer = pipe_writer_ref.as_ref().unwrap();
                while let Ok(written_in_this_call) =
                    pipe_writer.try_write(&buf.as_bytes()[written..])
                {
                    written += written_in_this_call;
                    if written >= buf.len() {
                        break;
                    }
                }
            });
        }

        fn flush(&self) {}
    }

    pub fn init_generic(
        level: LevelFilter,
    ) -> Option<embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, 4096>> {
        if super::LOGGER_INIT_DONE.swap(true, core::sync::atomic::Ordering::Relaxed) {
            return None;
        }
        let (reader, writer) = PIPE.take().split();
        LOGGER.pipe.borrow_mut().replace(writer);
        set_logger(&LOGGER).unwrap();
        set_max_level(level); // Adjust as needed
        Some(reader)
    }

    pub fn init_with_uart_tx(level: LevelFilter, tx: TxAsync) -> Option<UartLoggerRunner> {
        init_generic(level).map(|reader| UartLoggerRunner { reader, tx })
    }

    pub struct UartLoggerRunner {
        reader: embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, 4096>,
        tx: TxAsync,
    }

    impl UartLoggerRunner {
        pub async fn run(&mut self) -> ! {
            let mut log_buf = [0u8; 1024];

            loop {
                let read_bytes = self.reader.read(&mut log_buf).await;
                if read_bytes > 0 {
                    self.tx.write(&log_buf[..read_bytes]).unwrap().await;
                }
            }
        }
    }
}
