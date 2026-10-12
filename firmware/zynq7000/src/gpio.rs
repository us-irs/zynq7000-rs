//! # GPIO register module.

pub use types::*;

/// Register helper types.
pub mod types {

    /// Interrupt type.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum InterruptType {
        /// Level sensitivity.
        Level = 0,
        /// Edge sensitivity.
        Edge = 1,
    }

    /// Interrupt type register.
    #[bitbybit::bitfield(
        u32,
        debug,
        default = 0x0,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptTypeRegister {
        /// Individual bits.
        #[bit(0, rw)]
        pub bit: [InterruptType; 32],
    }

    /// Interrupt polarity bit.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Polarity {
        /// Active low / falling edge.
        ActiveLowOrFallingEdge = 0,
        /// Active high / rising edge.
        ActiveHighOrRisingEdge = 1,
    }

    /// Interrupt polarity register.
    #[bitbybit::bitfield(
        u32,
        debug,
        default = 0x0,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PolarityRegister {
        /// Individual bits.
        #[bit(0, rw)]
        pub bit: [Polarity; 32],
    }

    /// Maskable output data, only bits with the corresponding mask bit cleared are updated.
    #[bitbybit::bitfield(
        u32,
        debug,
        default = 0x0,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct MaskedOutput {
        /// Write mask, a set bit prevents the corresponding output bit from being updated.
        #[bits(16..=31, w)]
        mask: u16,
        /// Output data.
        #[bits(0..=15, rw)]
        output: u16,
    }

    /// Per-bank GPIO direction and interrupt control registers.
    #[derive(derive_mmio::Mmio)]
    #[repr(C)]
    pub struct BankControlRegisters {
        /// Direction mode
        dirm: u32,
        /// Output enable
        out_en: u32,
        /// Interrupt mask status
        ///
        /// TRM name: `INT_MASK`.
        #[mmio(PureRead)]
        interrupt_mask: u32,
        /// Interrupt enable/unmask
        ///
        /// TRM name: `INT_EN`.
        #[mmio(Write)]
        enable_interrupt: u32,
        /// Interrupt disable/mask
        ///
        /// TRM name: `INT_DIS`.
        #[mmio(Write)]
        disable_interrupt: u32,
        /// Interrupt status
        ///
        /// On reading: 1 means an interrupt has occurred, 0 is that no interrupt occurred.
        /// On write: 1 clears the interrupt status bit, 0 is no action.
        ///
        /// TRM name: `INT_STAT`.
        #[mmio(PureRead, Write)]
        interrupt_status: u32,
        /// Interrupt type
        ///
        /// TRM name: `INT_TYPE`.
        interrupt_type: InterruptTypeRegister,
        /// Interrupt polarity
        ///
        /// TRM name: `INT_POLARITY`.
        interrupt_polarity: PolarityRegister,
        /// Interrupt any edge sensitivity
        ///
        /// TRM name: `INT_ANY`.
        interrupt_any_sensitivity: u32,
    }
}

/// GPIO register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Maskable output data (GPIO bank 0, MIO, lower 16 bits)
    masked_out_0_lsw: MaskedOutput,
    /// Maskable output data (GPIO bank 0, MIO, upper 16 bits)
    masked_out_0_msw: MaskedOutput,
    /// Maskable output data (GPIO bank 1, MIO, lower 16 bits)
    masked_out_1_lsw: MaskedOutput,
    /// Maskable output data (GPIO bank 1, MIO, upper 16 bits)
    masked_out_1_msw: MaskedOutput,
    /// Maskable output data (GPIO bank 2, EMIO, lower 16 bits)
    masked_out_2_lsw: MaskedOutput,
    /// Maskable output data (GPIO bank 2, EMIO, upper 16 bits)
    masked_out_2_msw: MaskedOutput,
    /// Maskable output data (GPIO bank 3, EMIO, lower 16 bits)
    masked_out_3_lsw: MaskedOutput,
    /// Maskable output data (GPIO bank 3, EMIO, upper 16 bits)
    masked_out_3_msw: MaskedOutput,

    _reserved_0: [u32; 8],

    /// Output data (GPIO bank 0, MIO)
    out_0: u32,
    /// Output data (GPIO bank 1, MIO)
    out_1: u32,
    /// Output data (GPIO bank 2, EMIO)
    out_2: u32,
    /// Output data (GPIO bank 3, EMIO)
    out_3: u32,

    _reserved_1: [u32; 4],

    /// Input data (GPIO bank 0, MIO)
    #[mmio(PureRead)]
    in_0: u32,
    /// Input data (GPIO bank 1, MIO)
    #[mmio(PureRead)]
    in_1: u32,
    /// Input data (GPIO bank 2, EMIO)
    #[mmio(PureRead)]
    in_2: u32,
    /// Input data (GPIO bank 3, EMIO)
    #[mmio(PureRead)]
    in_3: u32,

    _reserved_2: [u32; 101],

    /// Bank 0 (MIO) direction and interrupt control
    #[mmio(Inner)]
    bank_0: BankControlRegisters,

    _reserved_3: [u32; 7],

    /// Bank 1 (MIO) direction and interrupt control
    #[mmio(Inner)]
    bank_1: BankControlRegisters,

    _reserved_4: [u32; 7],

    /// Bank 2 (EMIO) direction and interrupt control
    #[mmio(Inner)]
    bank_2: BankControlRegisters,

    _reserved_5: [u32; 7],

    /// Bank 3 (EMIO) direction and interrupt control
    #[mmio(Inner)]
    bank_3: BankControlRegisters,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x2E8);

impl Registers {
    /// Create a new XGPIOPS GPIO MMIO instance.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        MmioRegisters {
            ptr: 0xE000A000 as *mut Registers,
            phantom: core::marker::PhantomData,
        }
    }
}
