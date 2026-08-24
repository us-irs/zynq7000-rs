//! # GIC (Generic Interrupt Controller) register module.
pub use crate::mpcore::{GICC_BASE_ADDR, GICD_BASE_ADDR};
use static_assertions::const_assert_eq;

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u2, u3, u4, u5, u10, u11};

    /// Distributor Control Register
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DistributorControlRegister {
        /// Enables forwarding of Non-secure interrupts from the distributor to the CPU interfaces.
        #[bit(1, rw)]
        enable_non_secure: bool,
        /// Enables forwarding of Secure interrupts from the distributor to the CPU interfaces.
        #[bit(0, rw)]
        enable_secure: bool,
    }

    /// Read only bit. This register only returns fixed constants.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct TypeRegister {
        /// Number of lockable shared peripheral interrupts.
        #[bits(11..=15, r)]
        lspi: u5,
        /// The GIC implements the security extension.
        #[bit(10, r)]
        security_extension: bool,
        /// Number of implemented CPU interfaces, encoded.
        #[bits(5..=7, r)]
        cpu_number: u3,
        /// Number of implemented interrupt lines, encoded in blocks of 32.
        #[bits(0..=4, r)]
        it_lines_number: u5,
    }

    impl TypeRegister {
        /// Fixed value of the security extension bit.
        pub const SECURITY_EXTNS_BIT: bool = true;
        /// 31 LSPIs.
        pub const NUM_LSPI: usize = 0x1f;
        /// Encoding: 0b001 means that the Cortex-A9 MPCore has 2 processors.
        pub const CPU_NUMBER_BITS: u8 = 0b001;
        /// The distributor provides 96 interrupts.
        pub const IT_LINES_NUMBER: u8 = 0x2;

        /// Number of CPUs implemented in the Cortex-A9 MPCore.
        pub const NUM_OF_CPUS: usize = 2;
        /// Number of interrupts implemented by the distributor.
        pub const NUM_OF_INTERRUPTS: usize = 96;
    }

    /// Interrupt Controller Type Register.
    pub type Typer = TypeRegister;

    /// Interrupt Processor Targets Register, selecting which CPUs an interrupt is forwarded to.
    #[bitbybit::bitfield(
        u32,
        debug,
        default = 0x0,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    #[derive(PartialEq, Eq)]
    pub struct InterruptProcessorTargetRegister {
        /// Target array. Every register holds the information for 4 interrupts.
        #[bits(0..=1, rw, stride = 8)]
        targets: [u2; 4],
    }

    /// CPU interface control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterfaceControl {
        /// Controls whether the CPU interface uses the secure or non-secure binary point register.
        #[bit(4, rw)]
        sbpr: bool,
        /// Signal Secure interrupts as FIQ instead of IRQ.
        #[bit(3, rw)]
        fiq_en: bool,
        /// Controls whether a Non-secure read of the Interrupt Acknowledge Register can acknowledge
        /// a Secure interrupt.
        #[bit(2, rw)]
        ack_ctrl: bool,
        /// Enables signaling of Non-secure interrupts by the CPU interface.
        #[bit(1, rw)]
        enable_non_secure: bool,
        /// Enables signaling of Secure interrupts by the CPU interface.
        #[bit(0, rw)]
        enable_secure: bool,
    }

    /// Priority Mask Register
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct PriorityRegister {
        /// Interrupt priority value, smaller values indicate higher priority.
        #[bits(0..=7, rw)]
        priority: u8,
    }

    /// Interrupt acknowledge register.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct InterruptSignalRegister {
        /// ID of the CPU that requested the interrupt, valid for software generated interrupts.
        #[bits(10..=12, rw)]
        cpu_id: u3,
        /// Interrupt ID of the acknowledged or completed interrupt.
        #[bits(0..=9, rw)]
        ack_int_id: u10,
    }

    /// Determines when a software generated interrupt is treated as a Group 1 interrupt.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug, PartialEq, Eq)]
    pub enum SecurityCondition {
        /// Treated as Group 1 only if the distributor is configured as Secure.
        IfConfiguredAsSecure = 0,
        /// Treated as Group 1 only if the distributor is configured as Non-secure.
        IfConfiguredAsNonSecure = 1,
    }

    /// Selects the target CPUs of a software generated interrupt.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug, PartialEq, Eq)]
    pub enum TargetListFilter {
        /// Forward the interrupt to the CPUs in the target list.
        SendToCpusInTargetList = 0b00,
        /// Forward the interrupt to all CPUs except the requesting one.
        SendToAllOtherCpus = 0b01,
        /// Forward the interrupt to the requesting CPU only.
        SendToSelf = 0b10,
        /// Reserved value.
        Reserved = 0b11,
    }

    /// Software Generated Interrupt Register, used to request an interrupt on other CPUs.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct SoftwareGeneratedInterruptRegister {
        /// Selects which CPUs the interrupt is forwarded to.
        #[bits(24..=25, rw)]
        target_list_filter: TargetListFilter,
        /// Bitmask of target CPUs, used when the target list filter selects the target list.
        #[bits(16..=23, rw)]
        cpu_target_list: u8,
        /// SATT field.
        #[bit(15, rw)]
        security_condition: SecurityCondition,
        /// Should be zero.
        #[bits(4..=14, rw)]
        sbz: u11,
        /// Interrupt ID of the software generated interrupt, in the range 0 to 15.
        #[bits(0..=3, rw)]
        interrupt_id: u4,
    }
}

/// GIC Distributor registers.
#[derive(derive_mmio::Mmio)]
#[repr(C, align(8))]
pub struct DistributorRegisters {
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
    /// Interrupt Processor Targets Registers
    pub iptr_sgi: [InterruptProcessorTargetRegister; 0x4],
    /// These are read-only because they always target their private CPU.
    #[mmio(PureRead)]
    pub iptr_ppi: [InterruptProcessorTargetRegister; 0x4],
    /// Interrupt Processor Targets Registers for shared peripheral interrupts.
    pub iptr_spi: [InterruptProcessorTargetRegister; 0x10],
    // Those are split in the ARM documentation for some reason..
    _reserved_12: [u32; 0xE8],
    /// Interrupt Configuration Registers
    /// Interupt sensitivity register for software generated interrupts (SGI)
    #[mmio(PureRead)]
    pub icfr_0_sgi: u32,
    /// Interupt sensitivity register for private peripheral interrupts (PPI)
    pub icfr_1_ppi: u32,
    /// Interrupt sensitivity register for shared peripheral interrupts, bank 0.
    pub icfr_2_spi: u32,
    /// Interrupt sensitivity register for shared peripheral interrupts, bank 1.
    pub icfr_3_spi: u32,
    /// Interrupt sensitivity register for shared peripheral interrupts, bank 2.
    pub icfr_4_spi: u32,
    /// Interrupt sensitivity register for shared peripheral interrupts, bank 3.
    pub icfr_5_spi: u32,
    _reserved_13: [u32; 0x3A],
    /// Implementation defined status register for private peripheral interrupts.
    pub ppi_status: u32,
    /// Implementation defined status register for shared peripheral interrupts, bank 0.
    pub spi_status_0: u32,
    /// Implementation defined status register for shared peripheral interrupts, bank 1.
    pub spi_status_1: u32,
    _reserved_14: [u32; 0x7D],
    /// Software Generated Interrupt Register.
    pub sgir: SoftwareGeneratedInterruptRegister,
    _reserved_15: [u32; 0x33],
    /// Peripheral ID register 4.
    pub pidr_4: u32,
    /// Peripheral ID register 5.
    pub pidr_5: u32,
    /// Peripheral ID register 6.
    pub pidr_6: u32,
    /// Peripheral ID register 7.
    pub pidr_7: u32,
    /// Peripheral ID register 0.
    pub pidr_0: u32,
    /// Peripheral ID register 1.
    pub pidr_1: u32,
    /// Peripheral ID register 2.
    pub pidr_2: u32,
    /// Peripheral ID register 3.
    pub pidr_3: u32,
    /// Component ID registers, part of the CoreSight identification scheme.
    pub cidr: [u32; 4],
}

const_assert_eq!(core::mem::size_of::<DistributorRegisters>(), 0x1000);

impl DistributorRegisters {
    /// Create a new Global Interrupt Controller Distributor MMIO instance at the fixed address of
    /// the processing system.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioDistributorRegisters<'static> {
        unsafe { Self::new_mmio_at(GICD_BASE_ADDR) }
    }
}

/// GIC CPU interface registers.
#[derive(derive_mmio::Mmio)]
#[repr(C, align(8))]
pub struct CpuInterfaceRegisters {
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

const_assert_eq!(core::mem::size_of::<CpuInterfaceRegisters>(), 0x100);

impl CpuInterfaceRegisters {
    /// Create a new Global Interrupt Controller CPU MMIO instance at the fixed address of the
    /// processing system.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioCpuInterfaceRegisters<'static> {
        unsafe { Self::new_mmio_at(GICC_BASE_ADDR) }
    }
}
