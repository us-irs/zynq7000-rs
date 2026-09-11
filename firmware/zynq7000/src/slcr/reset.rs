use arbitrary_int::u17;

use super::{RESET_BLOCK_OFFSET, SLCR_BASE_ADDR};

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct DualClockReset {
    /// Peripheral 1 AMBA software reset.
    #[bit(1, rw)]
    periph1_cpu1x_rst: bool,
    /// Peripheral 0 AMBA software reset.
    #[bit(0, rw)]
    periph0_cpu1x_rst: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct DualRefAndClockResetSpiUart {
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

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct DualRefAndClockResetSdio {
    /// Periperal 1 Reference software reset.
    #[bit(5, rw)]
    periph1_ref_rst: bool,
    /// Peripheral 0 Reference software reset.
    #[bit(4, rw)]
    periph0_ref_rst: bool,
    /// Peripheral 1 AMBA software reset.
    #[bit(1, rw)]
    periph1_cpu1x_rst: bool,
    /// Peripheral 0 AMBA software reset.
    #[bit(0, rw)]
    periph0_cpu1x_rst: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct GpioClockReset {
    #[bit(0, rw)]
    gpio_cpu1x_rst: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct EthernetReset {
    #[bit(5, rw)]
    gem1_ref_rst: bool,
    #[bit(4, rw)]
    gem0_ref_rst: bool,
    #[bit(3, rw)]
    gem1_rx_rst: bool,
    #[bit(2, rw)]
    gem0_rx_rst: bool,
    #[bit(1, rw)]
    gem1_cpu1x_rst: bool,
    #[bit(0, rw)]
    gem0_cpu1x_rst: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct ResetControlQspiSmc {
    #[bit(1, rw)]
    ref_reset: bool,
    #[bit(0, rw)]
    cpu_1x_reset: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct FpgaResetControl {
    /// This block always needs to be written with 0. I think it contains some other hidden
    /// reset lines. This field makes this explicit.
    #[bits(8..=24, rw)]
    zero_block_0: u17,
    #[bit(3, rw)]
    fpga_3: bool,
    #[bit(2, rw)]
    fpga_2: bool,
    #[bit(1, rw)]
    fpga_1: bool,
    #[bit(0, rw)]
    fpga_0: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct CpuResetControl {
    #[bit(8, rw)]
    peripheral_reset: bool,
    #[bit(5, rw)]
    cpu1_clockstop: bool,
    #[bit(4, rw)]
    cpu0_clockstop: bool,
    #[bit(1, rw)]
    cpu1_reset: bool,
    #[bit(0, rw)]
    cpu0_reset: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct PsResetControl {
    #[bit(0, rw)]
    soft_reset: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct ResetControlSingleBit {
    #[bit(0, rw)]
    reset: bool,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct ResetControlInterconnect {
    /// Care must be taken to ensure that the AXI
    /// interconnect does not have outstanding
    /// transactions and the bus is idle.
    #[bit(0, rw)]
    reset: bool,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ApuWatchdogTarget {
    /// Same system level as PS_SRST_B.
    PsSrstB = 1,
    CpuAssociatedWithWdt = 0,
}

#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct WatchTimerResetControl {
    #[bit(1, rw)]
    apu_wdt_1_reset_target: ApuWatchdogTarget,
    #[bit(0, rw)]
    apu_wdt_0_reset_target: ApuWatchdogTarget,
}

/// Reset control block.
///
/// All reset signal bits are active high, writing a 1 asserts the reset.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct ResetControl {
    /// Processing System Software reset control
    pss: PsResetControl,
    ddr: ResetControlSingleBit,
    /// Central interconnect reset control
    topsw: ResetControlInterconnect,
    dmac: ResetControlSingleBit,
    usb: DualClockReset,
    eth: EthernetReset,
    sdio: DualRefAndClockResetSdio,
    spi: DualRefAndClockResetSpiUart,
    can: DualClockReset,
    i2c: DualClockReset,
    uart: DualRefAndClockResetSpiUart,
    gpio: GpioClockReset,
    lqspi: ResetControlQspiSmc,
    smc: ResetControlQspiSmc,
    ocm: ResetControlSingleBit,
    _gap0: u32,
    fpga: FpgaResetControl,
    a9_cpu: CpuResetControl,
    _gap1: u32,
    rs_awdt: WatchTimerResetControl,
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
