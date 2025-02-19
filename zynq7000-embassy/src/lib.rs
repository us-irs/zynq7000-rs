#![no_std]
use core::cell::{Cell, RefCell};

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::{Driver, TICK_HZ, time_driver_impl};
use embassy_time_queue_utils::Queue;
use once_cell::sync::OnceCell;

use zynq7000_hal::{clocks::ArmClocks, gtc::Gtc, time::Hertz};

static SCALE: OnceCell<u64> = OnceCell::new();
static CPU_3X2X_CLK: OnceCell<Hertz> = OnceCell::new();

struct AlarmState {
    timestamp: Cell<u64>,
}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

unsafe impl Send for AlarmState {}

/// This is the initialization method for the embassy time driver.
///
/// It should be called ONCE at system initialization.
pub fn init(arm_clocks: &ArmClocks, gtc: Gtc) {
    if SCALE.get().is_some() || CPU_3X2X_CLK.get().is_some() {
        return;
    }
    unsafe { GTC_TIME_DRIVER.init(arm_clocks, gtc) };
}

/// This interrupt handler should be called ONCE in the interrupt handler on global timer
/// interrupts.
///
/// # Safety
///
/// Needs to be called once in the global timer interrupt.
pub unsafe fn on_interrupt() {
    unsafe { GTC_TIME_DRIVER.on_interrupt() };
}

pub struct GtcTimerDriver {
    gtc: Mutex<RefCell<Gtc>>,
    // Timestamp at which to fire alarm. u64::MAX if no alarm is scheduled.
    alarms: Mutex<AlarmState>,
    queue: Mutex<RefCell<Queue>>,
}

impl GtcTimerDriver {
    /// This is the initialization method for the embassy time driver.
    ///
    /// # Safety
    ///
    /// This has to be called ONCE at system initialization.
    pub unsafe fn init(&'static self, arm_clock: &ArmClocks, mut gtc: Gtc) {
        CPU_3X2X_CLK.set(arm_clock.cpu_3x2x_clk()).unwrap();
        SCALE
            .set(arm_clock.cpu_3x2x_clk().raw() as u64 / TICK_HZ)
            .unwrap();
        gtc.set_cpu_3x2x_clock(arm_clock.cpu_3x2x_clk());
        gtc.set_prescaler(0);
        gtc.enable();
    }

    /// Should be called inside the IRQ handler if the IRQ ID is equal to
    /// [crate::hal::gic::PpiInterrupt::GlobalTimer].
    ///
    /// # Safety
    ///
    /// This function has to be called once for interrupt ID
    /// [crate::hal::gic::PpiInterrupt::GlobalTimer].
    pub unsafe fn on_interrupt(&self) {
        critical_section::with(|cs| {
            self.trigger_alarm(cs);
        })
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        if SCALE.get().is_none() {
            return false;
        }
        let mut gtc = self.gtc.borrow(cs).borrow_mut();
        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);

        let t = self.now();
        if timestamp <= t {
            gtc.disable_interrupt();
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        // If it hasn't triggered yet, setup the relevant reset value, regardless of whether
        // the interrupts are enabled or not. When they are enabled at a later point, the
        // right value is already set.

        // If the timestamp is in the next few ticks, add a bit of buffer to be sure the alarm
        // is not missed.
        //
        // This means that an alarm can be delayed for up to 2 ticks (from t+1 to t+3), but this is allowed
        // by the Alarm trait contract. What's not allowed is triggering alarms *before* their scheduled time,
        // and we don't do that here.
        let safe_timestamp = timestamp.max(t + 3);
        let opt_comparator = safe_timestamp.checked_mul(*SCALE.get().unwrap());
        if opt_comparator.is_none() {
            return true;
        }
        gtc.set_comparator(opt_comparator.unwrap());
        gtc.enable_interrupt();
        true
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut gtc = self.gtc.borrow(cs).borrow_mut();
        gtc.disable_interrupt();
        drop(gtc);

        let alarm = &self.alarms.borrow(cs);
        // Setting the maximum value disables the alarm.
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.
        let mut next = self
            .queue
            .borrow(cs)
            .borrow_mut()
            .next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self
                .queue
                .borrow(cs)
                .borrow_mut()
                .next_expiration(self.now());
        }
    }
}
impl Driver for GtcTimerDriver {
    #[inline]
    fn now(&self) -> u64 {
        if SCALE.get().is_none() {
            return 0;
        }
        // This is okay, we only read the GTC and do not re-configure it, avoids the
        // need for a lock.
        let gtc = unsafe { Gtc::steal_fixed(Some(*CPU_3X2X_CLK.get().unwrap())) };

        gtc.read_timer() / SCALE.get().unwrap()
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

time_driver_impl!(
    // We assume ownership of the GTC, so it is okay to steal here.
    static GTC_TIME_DRIVER: GtcTimerDriver = GtcTimerDriver {
        gtc: Mutex::new(RefCell::new(unsafe { Gtc::steal_fixed(None)})),
        alarms: Mutex::new(AlarmState::new()),
        queue: Mutex::new(RefCell::new(Queue::new())),
});
