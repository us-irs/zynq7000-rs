//! Global timer counter driver module.
//!
//! # Examples
//!
//! - [GTC ticks example](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/examples/simple/src/bin/gtc-ticks.rs)
//! - [Embassy Timer Driver](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq7000-embassy/src/lib.rs)
use zynq7000::gtc::MmioGlobalTimerCounter;

use crate::{clocks::ArmClocks, time::Hertz};

/// High level GTC driver.
///
/// This structure also allows an optional clock member, which is required for the
/// [frequency_to_ticks] function and the [embedded_hal::delay::DelayNs] implementation
/// to work.
pub struct GlobalTimerCounter {
    regs: MmioGlobalTimerCounter<'static>,
    cpu_3x2x_clock: Option<Hertz>,
}

unsafe impl Send for GlobalTimerCounter {}

pub const fn frequency_to_ticks(clock: Hertz, frequency: Hertz) -> u32 {
    clock.raw().div_ceil(frequency.raw())
}

impl GlobalTimerCounter {
    /// Create a peripheral driver from a MMIO GTC block.
    #[inline]
    pub const fn new(_regs: MmioGlobalTimerCounter<'static>, clocks: &ArmClocks) -> Self {
        unsafe { Self::steal_fixed(Some(clocks.cpu_3x2x_clk())) }
    }

    /// Steal the GTC from the PAC.
    ///
    /// This function still expect the GTC clock, which is the CPU 3x2x clock frequency.
    ///
    /// # Safety
    ///
    /// This function allows creating an arbitrary amount of memory-mapped peripheral drivers.
    /// See the [zynq7000::gtc::GlobalTimerCounter::new_mmio] docs for more safety information.
    #[inline]
    pub const unsafe fn steal_fixed(cpu_3x2x_clk: Option<Hertz>) -> Self {
        Self {
            regs: unsafe { zynq7000::gtc::GlobalTimerCounter::new_mmio_fixed() },
            cpu_3x2x_clock: cpu_3x2x_clk,
        }
    }

    #[inline]
    pub fn set_cpu_3x2x_clock(&mut self, clock: Hertz) {
        self.cpu_3x2x_clock = Some(clock);
    }

    // TODO: Change this API once pure-reads work.
    /// Read the 64-bit timer.
    #[inline]
    pub fn read_timer(&self) -> u64 {
        // Safety: We require interior mutability here because even reads are unsafe.
        // But we want to avoid a RefCell which would incur a run-time cost solely to make this
        // function non-mut, so we steal the GTC here. Ownership is guaranteed or mandated
        // by constructor.
        let upper = self.regs.read_count_upper();
        loop {
            let lower = self.regs.read_count_lower();
            if self.regs.read_count_upper() == upper {
                return ((upper as u64) << 32) | (lower as u64);
            }
            // Overflow, read upper again.
        }
    }

    /// Set the comparator which can be used to trigger an interrupt in the future.
    #[inline]
    pub fn set_comparator(&mut self, comparator: u64) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_comparator_enable(false);
            ctrl
        });
        self.regs.write_comparator_upper((comparator >> 32) as u32);
        self.regs.write_comparator_lower(comparator as u32);
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_comparator_enable(true);
            ctrl
        });
    }

    pub fn frequency_to_ticks(&self, frequency: Hertz) -> u32 {
        if self.cpu_3x2x_clock.is_none() {
            return 0;
        }
        frequency_to_ticks(self.cpu_3x2x_clock.unwrap(), frequency)
    }

    /// Set the auto-increment value which will be used by the hardware to automatically
    /// increment the comparator value on a comparator interrupt, if the auto-increment is enabled.
    #[inline]
    pub fn set_auto_increment_value(&mut self, value: u32) {
        self.regs.write_auto_increment(value);
    }

    #[inline]
    pub fn set_auto_increment_value_for_frequency(&mut self, frequency: Hertz) {
        self.regs
            .write_auto_increment(self.frequency_to_ticks(frequency));
    }

    #[inline]
    pub fn enable(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_enable(true);
            ctrl
        });
    }

    #[inline]
    pub fn enable_auto_increment(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_auto_increment(true);
            ctrl
        });
    }

    #[inline]
    pub fn set_prescaler(&mut self, prescaler: u8) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_prescaler(prescaler);
            ctrl
        });
    }

    #[inline]
    pub fn disable(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_enable(false);
            ctrl
        });
    }

    /// Enable the comparator interrupt.
    #[inline]
    pub fn enable_interrupt(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_irq_enable(true);
            ctrl
        });
    }

    /// Disable the comparator interrupt.
    #[inline]
    pub fn disable_interrupt(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_irq_enable(false);
            ctrl
        });
    }
}

/// GTC can be used for blocking delays.
impl embedded_hal::delay::DelayNs for GlobalTimerCounter {
    fn delay_ns(&mut self, ns: u32) {
        if self.cpu_3x2x_clock.is_none() {
            return;
        }
        let end_of_delay = self.read_timer()
            + (((ns as u64) * self.cpu_3x2x_clock.unwrap().raw() as u64) / 1_000_000_000);
        while self.read_timer() < end_of_delay {}
    }
}
