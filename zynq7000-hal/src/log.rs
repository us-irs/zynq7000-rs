//! # Simple logging providers.

/// Blocking UART loggers.
pub mod uart_blocking {
    use core::cell::{Cell, RefCell, UnsafeCell};
    use embedded_io::Write as _;

    use cortex_ar::register::Cpsr;
    use critical_section::Mutex;
    use log::{LevelFilter, set_logger, set_max_level};

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
        // TODO: Impl debug for Uart
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

        fn flush(&self) {}
    }

    pub struct UartLoggerUnsafeSingleThread {
        skip_in_isr: Cell<bool>,
        uart: UnsafeCell<Option<Uart>>,
    }

    unsafe impl Send for UartLoggerUnsafeSingleThread {}
    unsafe impl Sync for UartLoggerUnsafeSingleThread {}

    static UART_LOGGER_UNSAFE_SINGLE_THREAD: UartLoggerUnsafeSingleThread =
        UartLoggerUnsafeSingleThread {
            skip_in_isr: Cell::new(false),
            uart: UnsafeCell::new(None),
        };

    /// Initialize the logger with a blocking UART instance.
    ///
    /// For async applications, it is strongly recommended to use the asynchronous ring buffer
    /// logger instead.
    ///
    /// # Safety
    ///
    /// This is a blocking logger which performs a write WITHOUT a critical section. This logger is
    /// NOT thread-safe. Users must ensure that this logger is not used inside a pre-emptive
    /// multi-threading context and interrupt handlers.
    pub unsafe fn create_unsafe_single_thread_logger(uart: Uart) -> UartLoggerUnsafeSingleThread {
        UartLoggerUnsafeSingleThread {
            skip_in_isr: Cell::new(false),
            uart: UnsafeCell::new(Some(uart)),
        }
    }

    /// Initialize the logger with a blocking UART instance which does not use locks.
    ///
    /// # Safety
    ///
    /// This is a blocking logger which performs a write WITHOUT a critical section. This logger is
    /// NOT thread-safe, which might lead to garbled output. Log output in ISRs can optionally be
    /// surpressed.
    pub unsafe fn init_unsafe_single_core(uart: Uart, level: LevelFilter, skip_in_isr: bool) {
        let opt_uart = unsafe { &mut *UART_LOGGER_UNSAFE_SINGLE_THREAD.uart.get() };
        opt_uart.replace(uart);
        UART_LOGGER_UNSAFE_SINGLE_THREAD
            .skip_in_isr
            .set(skip_in_isr);

        set_logger(&UART_LOGGER_UNSAFE_SINGLE_THREAD).unwrap();
        set_max_level(level); // Adjust as needed
    }

    impl log::Log for UartLoggerUnsafeSingleThread {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            if self.skip_in_isr.get() {
                match Cpsr::read().mode().unwrap() {
                    cortex_ar::register::cpsr::ProcessorMode::Fiq
                    | cortex_ar::register::cpsr::ProcessorMode::Irq => {
                        return;
                    }
                    _ => {}
                }
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

        fn flush(&self) {}
    }
}

/// Logger module which logs into a ring buffer to allow asynchronous logging handling.
pub mod rb {
    use core::cell::RefCell;
    use core::fmt::Write as _;

    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use log::{LevelFilter, set_logger, set_max_level};
    use ringbuf::{
        StaticRb,
        traits::{Consumer, Producer},
    };

    /// Logger implementation which logs frames via a ring buffer and sends the frame sizes
    /// as messages.
    ///
    /// The logger does not require allocation and reserved a generous amount of 4096 bytes for
    /// both data buffer and ring buffer. This should be sufficient for most logging needs.
    pub struct Logger {
        frame_queue: embassy_sync::channel::Channel<CriticalSectionRawMutex, usize, 32>,
        data_buf: critical_section::Mutex<RefCell<heapless::String<4096>>>,
        ring_buf: critical_section::Mutex<RefCell<Option<StaticRb<u8, 4096>>>>,
    }

    unsafe impl Send for Logger {}
    unsafe impl Sync for Logger {}

    static LOGGER_RB: Logger = Logger {
        frame_queue: embassy_sync::channel::Channel::new(),
        data_buf: critical_section::Mutex::new(RefCell::new(heapless::String::new())),
        ring_buf: critical_section::Mutex::new(RefCell::new(None)),
    };

    impl log::Log for Logger {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            critical_section::with(|cs| {
                let ref_buf = self.data_buf.borrow(cs);
                let mut buf = ref_buf.borrow_mut();
                buf.clear();
                let _ = writeln!(buf, "{} - {}\r", record.level(), record.args());
                let rb_ref = self.ring_buf.borrow(cs);
                let mut rb_opt = rb_ref.borrow_mut();
                if rb_opt.is_none() {
                    panic!("log call on uninitialized logger");
                }
                rb_opt.as_mut().unwrap().push_slice(buf.as_bytes());
                let _ = self.frame_queue.try_send(buf.len());
            });
        }

        fn flush(&self) {
            while !self.frame_queue().is_empty() {}
        }
    }

    impl Logger {
        pub fn frame_queue(
            &self,
        ) -> &embassy_sync::channel::Channel<CriticalSectionRawMutex, usize, 32> {
            &self.frame_queue
        }
    }

    pub fn init(level: LevelFilter) {
        critical_section::with(|cs| {
            let rb = StaticRb::<u8, 4096>::default();
            let rb_ref = LOGGER_RB.ring_buf.borrow(cs);
            rb_ref.borrow_mut().replace(rb);
        });
        set_logger(&LOGGER_RB).unwrap();
        set_max_level(level); // Adjust as needed
    }

    pub fn read_next_frame(frame_len: usize, buf: &mut [u8]) {
        let read_len = core::cmp::min(frame_len, buf.len());
        critical_section::with(|cs| {
            let rb_ref = LOGGER_RB.ring_buf.borrow(cs);
            let mut rb = rb_ref.borrow_mut();
            rb.as_mut().unwrap().pop_slice(&mut buf[0..read_len]);
        })
    }

    pub fn get_frame_queue()
    -> &'static embassy_sync::channel::Channel<CriticalSectionRawMutex, usize, 32> {
        LOGGER_RB.frame_queue()
    }
}
