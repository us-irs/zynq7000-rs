use super::{RESET_BLOCK_OFFSET, SLCR_BASE_ADDR};

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct DualClockReset {
    /// Peripheral 1 AMBA software reset.
    #[bit(1, rw)]
    periph1_cpu1x_rst: bool,
    /// Peripheral 0 AMBA software reset.
    #[bit(0, rw)]
    periph0_cpu1x_rst: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct DualRefAndClockReset {
    /// Periperal 1 Reference software reset.
    #[bit(3, rw)]
    periph1_ref_rst: bool,
    /// Peripheral 0 Reference software reset.
    #[bit(2, rw)]
    periph0_ref_rst: bool,
    /// Peripheral 1 AMBA software reset.
    #[bit(1, rw)]
    periph1_cpu1x_rst: bool,
    /// Peripheral 0 AMBA software reset.
    #[bit(0, rw)]
    periph0_cpu1x_rst: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct GpioClockReset {
    #[bit(0, rw)]
    gpio_cpu1x_rst: bool,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct ResetControl {
    /// PS Software reset control
    pss: u32,
    ddr: u32,
    /// Central interconnect reset control
    topsw: u32,
    dmac: u32,
    usb: u32,
    gem: u32,
    sdio: DualRefAndClockReset,
    spi: DualRefAndClockReset,
    can: DualClockReset,
    i2c: DualClockReset,
    uart: DualRefAndClockReset,
    gpio: GpioClockReset,
    lqspi: u32,
    smc: u32,
    ocm: u32,
    _gap0: u32,
    fpga: u32,
    a9_cpu: u32,
    _gap1: u32,
    rs_awdt: u32,
}

impl ResetControl {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioResetControl<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR + RESET_BLOCK_OFFSET) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<ResetControl>(), 0x50);
