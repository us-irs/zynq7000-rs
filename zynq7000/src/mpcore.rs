//! Application Processing Unit Registers (mpcore)
//!
//! Based on p.1483 of the Zynq-7000 TRM.
use static_assertions::const_assert_eq;

use crate::{
    gic::{GicCpuInterface, GicDistributor, MmioGicCpuInterface, MmioGicDistributor},
    gtc::{GlobalTimerCounter, MmioGlobalTimerCounter},
};

pub const MPCORE_BASE_ADDR: usize = 0xF8F0_0000;
pub const SCU_BASE_ADDR: usize = MPCORE_BASE_ADDR;
pub const GICC_BASE_ADDR: usize = MPCORE_BASE_ADDR + 0x100;
pub const GICD_BASE_ADDR: usize = MPCORE_BASE_ADDR + 0x1000;

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Scu {
    ctrl: u32,
    config: u32,
    cpu_power_status: u32,
    invalidate_all_regs_in_secure_state: u32,
    _reserved_0: [u32; 0xC],
    filtering_start_addr: u32,
    filtering_end_addr: u32,
    _reserved_1: [u32; 0x2],
    access_ctrl: u32,
    non_secure_access_ctrl: u32,
}

impl Scu {
    /// Create a new Snoop Control Unit interface at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioScu<'static> {
        unsafe { Self::new_mmio_at(SCU_BASE_ADDR) }
    }
}

const_assert_eq!(core::mem::size_of::<Scu>(), 0x58);

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Mpcore {
    #[mmio(Inner)]
    scu: Scu,

    _reserved_0: [u32; 0x2A],

    #[mmio(Inner)]
    gicc: GicCpuInterface,

    #[mmio(Inner)]
    gt: GlobalTimerCounter,

    _reserved_1: [u32; 0xF9],

    private_timer_load: u32,
    private_timer_counter: u32,
    private_timer_ctrl: u32,
    private_interrupt_status: u32,

    _reserved_2: [u32; 0x4],

    watchdog_load: u32,
    watchdog_counter: u32,
    watchdog_ctrl: u32,
    watchdog_interrupt_status: u32,
    watchdog_reset_status: u32,
    watchdog_disable: u32,

    _reserved_3: [u32; 0x272],

    #[mmio(Inner)]
    gicd: GicDistributor,
}

const_assert_eq!(core::mem::size_of::<Mpcore>(), 0x2000);

impl Mpcore {
    /// Create a MP core peripheral interface at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioMpcore<'static> {
        unsafe { Self::new_mmio_at(MPCORE_BASE_ADDR) }
    }
}
