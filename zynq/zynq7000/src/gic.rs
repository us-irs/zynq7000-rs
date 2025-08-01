//! # GIC (Generic Interrupt Controller) register module.
pub use crate::mpcore::{GICC_BASE_ADDR, GICD_BASE_ADDR};
use arbitrary_int::{u3, u5, u10};
use static_assertions::const_assert_eq;

/// Distributor Control Register
#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct DistributorControlRegister {
    #[bit(1, rw)]
    enable_non_secure: bool,
    #[bit(0, rw)]
    enable_secure: bool,
}

/// Read only bit. This register only returns fixed constants.
#[bitbybit::bitfield(u32, debug)]
pub struct TypeRegister {
    #[bits(11..=15, r)]
    lspi: u5,
    #[bit(10, r)]
    security_extension: bool,
    #[bits(5..=7, r)]
    cpu_number: u3,
    #[bits(0..=4, r)]
    it_lines_number: u5,
}

impl TypeRegister {
    pub const SECURITY_EXTNS_BIT: bool = true;
    /// 31 LSPIs.
    pub const NUM_LSPI: usize = 0x1f;
    /// Encoding: 0b001 means that the Cortex-A9 MPCore has 2 processors.
    pub const CPU_NUMBER_BITS: u8 = 0b001;
    /// The distributor provides 96 interrupts.
    pub const IT_LINES_NUMBER: u8 = 0x2;

    pub const NUM_OF_CPUS: usize = 2;
    pub const NUM_OF_INTERRUPTS: usize = 96;
}

pub type Typer = TypeRegister;

/// GIC Distributor registers.
#[derive(derive_mmio::Mmio)]
#[repr(C, align(8))]
pub struct GicDistributor {
    /// Distributor Control Register
    pub dcr: DistributorControlRegister,
    /// Interrupt Controller Type Register
    #[mmio(PureRead)]
    pub ictr: Typer,
    /// Distributor Implementer Identification Register
    #[mmio(PureRead)]
    pub iidr: u32,
    _reserved_0: [u32; 0x1D],
    /// Interrupt security registers
    pub isr: [u32; 3],
    _reserved_1: [u32; 0x1D],
    /// Interrupt Set-Enable Registers
    pub iser: [u32; 0x3],
    _reserved_3: [u32; 0x1D],
    /// Interrupt Clear-Enable Registers
    pub icer: [u32; 0x3],
    _reserved_4: [u32; 0x1D],
    /// Interrupt Set-Pending Registers
    pub ispr: [u32; 0x3],
    _reserved_5: [u32; 0x1D],
    /// Interrupt Clear-Pending Registers
    pub icpr: [u32; 0x3],
    _reserved_6: [u32; 0x1D],
    /// Active Bit Registers
    pub abr: [u32; 0x3],
    _reserved_10: [u32; 0x3D],
    /// Interrupt Priority Registers
    pub ipr: [u32; 0x18],
    _reserved_11: [u32; 0xE8],
    /// Interrupt Processor Targes Registers
    pub iptr_sgi: [u32; 0x4],
    // TODO: Mark those read-only as soon as that works for arrays.
    pub iptr_ppi: [u32; 0x4],
    pub iptr_spi: [u32; 0x10],
    // Those are split in the ARM documentation for some reason..
    _reserved_12: [u32; 0xE8],
    /// Interrupt Configuration Registers
    /// Interupt sensitivity register for software generated interrupts (SGI)
    pub icfr_0_sgi: u32,
    /// Interupt sensitivity register for private peripheral interrupts (PPI)
    pub icfr_1_ppi: u32,
    pub icfr_2_spi: u32,
    pub icfr_3_spi: u32,
    pub icfr_4_spi: u32,
    pub icfr_5_spi: u32,
    _reserved_13: [u32; 0x3A],
    pub ppi_status: u32,
    pub spi_status_0: u32,
    pub spi_status_1: u32,
    _reserved_14: [u32; 0x7D],
    /// Software Generated Interrupt Register.
    pub sgir: u32,
    _reserved_15: [u32; 0x33],
    pub pidr_4: u32,
    pub pidr_5: u32,
    pub pidr_6: u32,
    pub pidr_7: u32,
    pub pidr_0: u32,
    pub pidr_1: u32,
    pub pidr_2: u32,
    pub pidr_3: u32,
    pub cidr: [u32; 4],
}

const_assert_eq!(core::mem::size_of::<GicDistributor>(), 0x1000);

impl GicDistributor {
    /// Create a new Global Interrupt Controller Distributor MMIO instance at the fixed address of
    /// the processing system.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioGicDistributor<'static> {
        unsafe { Self::new_mmio_at(GICD_BASE_ADDR) }
    }
}

/// CPU interface control register.
#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct InterfaceControl {
    #[bit(4, rw)]
    sbpr: bool,
    #[bit(3, rw)]
    fiq_en: bool,
    #[bit(2, rw)]
    ack_ctrl: bool,
    #[bit(1, rw)]
    enable_non_secure: bool,
    #[bit(0, rw)]
    enable_secure: bool,
}

/// Priority Mask Register
#[bitbybit::bitfield(u32, debug)]
pub struct PriorityRegister {
    #[bits(0..=7, rw)]
    priority: u8,
}

/// Interrupt acknowledge register.
#[bitbybit::bitfield(u32, debug)]
pub struct InterruptSignalRegister {
    #[bits(10..=12, rw)]
    cpu_id: u3,
    #[bits(0..=9, rw)]
    ack_int_id: u10,
}

/// GIC CPU interface registers.
#[derive(derive_mmio::Mmio)]
#[repr(C, align(8))]
pub struct GicCpuInterface {
    /// CPU Interface Control Register (ICR).
    pub icr: InterfaceControl,
    /// Interrupt Priority Mask Register.
    pub pmr: PriorityRegister,
    /// Binary Point Register.
    pub bpr: u32,
    /// Interrupt Acknowledge Register.
    pub iar: InterruptSignalRegister,
    /// End of Interrupt Register.
    pub eoir: InterruptSignalRegister,
    /// Running Priority Register.
    pub rpr: PriorityRegister,
    /// Highest Pending Interrupt Register.
    pub hpir: InterruptSignalRegister,
    /// Aliased Binary Point Register
    pub abpr: u32,
    _reserved_0: [u32; 0x37],
    /// CPU Interface Identification Register.
    #[mmio(PureRead)]
    pub iidr: u32,
}

const_assert_eq!(core::mem::size_of::<GicCpuInterface>(), 0x100);

impl GicCpuInterface {
    /// Create a new Global Interrupt Controller CPU MMIO instance at the fixed address of the
    /// processing system.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioGicCpuInterface<'static> {
        unsafe { Self::new_mmio_at(GICC_BASE_ADDR) }
    }
}
