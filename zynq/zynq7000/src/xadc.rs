pub const XADC_BASE_ADDR: usize = 0xF8007100;

/// XADC register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct XAdc {
    config: u32,
    interrupt_status: u32,
    interrupt_mask: u32,
    misc_status: u32,
    command_fifo: u32,
    data_fifo: u32,
    misc_control: u32,
}

impl XAdc {
    /// Create a new XADC MMIO instance for for device configuration peripheral at address
    /// [XADC_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub unsafe fn new_mmio_fixed() -> MmioXAdc<'static> {
        unsafe { XAdc::new_mmio_at(XADC_BASE_ADDR) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<XAdc>(), 0x1C);
