//! Application Processing Unit Registers (mpcore)
//!
//! Based on p.1483 of the Zynq-7000 TRM.
use static_assertions::const_assert_eq;

use crate::{
    gic::{
        CpuInterfaceRegisters, DistributorRegisters, MmioCpuInterfaceRegisters,
        MmioDistributorRegisters,
    },
    gtc::{MmioRegisters, Registers},
};

/// Base address of the MPCore private memory region.
pub const MPCORE_BASE_ADDR: usize = 0xF8F0_0000;
/// Base address of the Snoop Control Unit registers.
pub const SCU_BASE_ADDR: usize = MPCORE_BASE_ADDR;
/// Base address of the GIC CPU interface registers.
pub const GICC_BASE_ADDR: usize = MPCORE_BASE_ADDR + 0x100;
/// Base address of the GIC distributor registers.
pub const GICD_BASE_ADDR: usize = MPCORE_BASE_ADDR + 0x1000;

/// Deprecated type alias.
#[deprecated]
pub type SnoopControlUnit = ScuRegisters;

/// Snoop Control Unit register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct ScuRegisters {
    /// SCU control
    control: u32,
    /// SCU configuration
    config: u32,
    /// CPU power status
    cpu_power_status: u32,
    /// Invalidate all registers in secure state
    invalidate_all_regs_in_secure_state: u32,
    _reserved_0: [u32; 0xC],
    /// Filtering start address
    filtering_start_addr: u32,
    /// Filtering end address
    filtering_end_addr: u32,
    _reserved_1: [u32; 0x2],
    /// Access control
    access_ctrl: u32,
    /// Non-secure access control
    non_secure_access_ctrl: u32,
}

/// Deprecated type alias.
#[deprecated]
pub type MpCore = MpCoreRegisters;

/// MP Core register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct MpCoreRegisters {
    /// Snoop Control Unit
    #[mmio(Inner)]
    scu: ScuRegisters,

    _reserved_0: [u32; 0x2A],

    /// GIC CPU interface
    #[mmio(Inner)]
    gicc: CpuInterfaceRegisters,

    /// Global timer
    #[mmio(Inner)]
    gt: Registers,

    _reserved_1: [u32; 0xF9],

    /// Private timer load
    private_timer_load: u32,
    /// Private timer counter
    private_timer_counter: u32,
    /// Private timer control
    private_timer_ctrl: u32,
    /// Private timer interrupt status
    private_interrupt_status: u32,

    _reserved_2: [u32; 0x4],

    /// Watchdog load
    watchdog_load: u32,
    /// Watchdog counter
    watchdog_counter: u32,
    /// Watchdog control
    watchdog_ctrl: u32,
    /// Watchdog interrupt status
    watchdog_interrupt_status: u32,
    /// Watchdog reset status
    watchdog_reset_status: u32,
    /// Watchdog disable
    watchdog_disable: u32,

    _reserved_3: [u32; 0x272],

    /// GIC distributor
    #[mmio(Inner)]
    gicd: DistributorRegisters,
}

const_assert_eq!(core::mem::size_of::<ScuRegisters>(), 0x58);

impl ScuRegisters {
    /// Create a new Snoop Control Unit interface at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioScuRegisters<'static> {
        unsafe { Self::new_mmio_at(SCU_BASE_ADDR) }
    }
}

const_assert_eq!(core::mem::size_of::<MpCoreRegisters>(), 0x2000);

impl MpCoreRegisters {
    /// Create a MP core peripheral interface at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioMpCoreRegisters<'static> {
        unsafe { Self::new_mmio_at(MPCORE_BASE_ADDR) }
    }
}
