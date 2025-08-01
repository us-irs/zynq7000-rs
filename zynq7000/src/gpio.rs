//! # GPIO register module.
#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct MaskedOutput {
    #[bits(16..=31, w)]
    pub mask: u16,
    #[bits(0..=15, rw)]
    pub output: u16,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct BankCtrl {
    /// Direction mode
    dirm: u32,
    /// Output enable
    out_en: u32,
    /// Interrupt mask status
    #[mmio(PureRead)]
    int_mask: u32,
    /// Interrupt enable/unmask
    #[mmio(Write)]
    int_en: u32,
    /// Interrupt disable/mask
    #[mmio(Write)]
    int_dis: u32,
    /// Interrupt status
    #[mmio(PureRead, Write)]
    int_sts: u32,
    /// Interrupt type
    int_type: u32,
    /// Interrupt polarity
    int_pol: u32,
    /// Interrupt any edge sensitivity
    int_any: u32,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Gpio {
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

    #[mmio(Inner)]
    bank_0: BankCtrl,

    _reserved_3: [u32; 7],

    #[mmio(Inner)]
    bank_1: BankCtrl,

    _reserved_4: [u32; 7],

    #[mmio(Inner)]
    bank_2: BankCtrl,

    _reserved_5: [u32; 7],

    #[mmio(Inner)]
    bank_3: BankCtrl,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Gpio>(), 0x2E8);

impl Gpio {
    /// Create a new XGPIOPS GPIO MMIO instance.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioGpio<'static> {
        MmioGpio {
            ptr: 0xE000A000 as *mut Gpio,
            phantom: core::marker::PhantomData,
        }
    }
}
