//! # CPU private timer module
//!
//! ## Examples
//!
//! - Private timer as delay provider in [blinky](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq/examples/simple/src/bin/blinky.rs)
use core::{marker::PhantomData, sync::atomic::AtomicBool};

use zynq7000::priv_tim::InterruptStatus;

use crate::{clocks::ArmClocks, time::Hertz};

static CORE_0_TIM_TAKEN: AtomicBool = AtomicBool::new(false);
static CORE_1_TIM_TAKEN: AtomicBool = AtomicBool::new(false);

/// High-level CPU private timer driver.
pub struct CpuPrivateTimer {
    regs: zynq7000::priv_tim::MmioRegisters<'static>,
    cpu_3x2x_clock: Hertz,
    // Add this marker to explicitely opt-out of Send and Sync.
    //
    // This is a CPU private timer and thus should not be sent to other threads.
    _not_send: PhantomData<*const ()>,
}

impl CpuPrivateTimer {
    /// Take the CPU private timer for a given core.
    ///
    /// This function can only be called once for each given core.
    pub fn take(clocks: &ArmClocks) -> Option<Self> {
        let mpidr = aarch32_cpu::register::mpidr::Mpidr::read();
        let core = mpidr.0 & 0xff;
        if core != 0 && core != 1 {
            return None;
        }
        if (core == 0 && CORE_0_TIM_TAKEN.swap(true, core::sync::atomic::Ordering::Relaxed))
            || (core == 1 && CORE_1_TIM_TAKEN.swap(true, core::sync::atomic::Ordering::Relaxed))
        {
            return None;
        }

        Some(Self::steal(clocks))
    }

    /// Create a new CPU private timer driver.
    ///
    /// # Safety
    ///
    /// This function allows to potentially create an arbitrary amount of timers for both cores.
    /// It also does not check the current core ID.
    pub fn steal(clocks: &ArmClocks) -> Self {
        Self {
            regs: unsafe { zynq7000::priv_tim::Registers::new_mmio_fixed() },
            cpu_3x2x_clock: clocks.cpu_3x2x_clk(),
            _not_send: PhantomData,
        }
    }

    /// Write reload value which is set by the hardware when the timer reaches zero and
    /// auto reload is enabled.
    #[inline]
    pub fn write_reload(&mut self, value: u32) {
        self.regs.write_reload(value);
    }

    #[inline]
    pub fn write_counter(&mut self, value: u32) {
        self.regs.write_counter(value);
    }

    #[inline]
    pub fn counter(&self) -> u32 {
        self.regs.read_counter()
    }
}

impl embedded_hal::delay::DelayNs for CpuPrivateTimer {
    fn delay_ns(&mut self, ns: u32) {
        // Even for a value of 1000 MHz for CPU 3x2x and u32::MAX for nanoseconds, this will
        // never overflow.
        let ticks = (ns as u64 * self.cpu_3x2x_clock.raw() as u64) / 1_000_000_000;

        // Split the total delay into manageable chunks (u32::MAX ticks max).
        let mut remaining = ticks;

        self.regs.modify_control(|mut val| {
            val.set_enable(false);
            // The event flag is still set, which is all we care about.
            val.set_interrupt_enable(false);
            val.set_auto_reload(false);
            val
        });
        while remaining > 0 {
            let chunk = (remaining as u32).min(u32::MAX - 1);

            // Clear the timer flag by writing 1 to it.
            self.regs
                .write_interrupt_status(InterruptStatus::builder().with_event_flag(true).build());

            // Load the timer with the chunk value and start it.
            self.write_reload(chunk);
            self.write_counter(chunk);
            self.regs.modify_control(|mut val| {
                val.set_enable(true);
                val
            });

            // Wait for the timer to count down to zero.
            while !self.regs.read_interrupt_status().event_flag() {}

            remaining -= chunk as u64;
        }
    }
}
