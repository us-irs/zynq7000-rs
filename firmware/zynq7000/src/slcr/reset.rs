use super::{RESET_BLOCK_OFFSET, SLCR_BASE_ADDR};

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::u17;

    /// AMBA (CPU_1x) software reset control for a peripheral with two instances.
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

    /// AMBA and reference clock software reset control for SPI and UART, each with two instances.
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

    /// AMBA and reference clock software reset control for the two SDIO instances.
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

    /// GPIO AMBA (CPU_1x) software reset control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct GpioClockReset {
        /// GPIO AMBA software reset.
        #[bit(0, rw)]
        gpio_cpu1x_rst: bool,
    }

    /// Ethernet (GEM) software reset control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct EthernetReset {
        /// GEM 1 reference clock reset.
        #[bit(5, rw)]
        gem1_ref_rst: bool,
        /// GEM 0 reference clock reset.
        #[bit(4, rw)]
        gem0_ref_rst: bool,
        /// GEM 1 receiver logic reset.
        #[bit(3, rw)]
        gem1_rx_rst: bool,
        /// GEM 0 receiver logic reset.
        #[bit(2, rw)]
        gem0_rx_rst: bool,
        /// GEM 1 AMBA software reset.
        #[bit(1, rw)]
        gem1_cpu1x_rst: bool,
        /// GEM 0 AMBA software reset.
        #[bit(0, rw)]
        gem0_cpu1x_rst: bool,
    }

    /// Software reset control shared by the QSPI and SMC memory controllers.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ResetControlQspiSmc {
        /// Reference clock domain reset.
        #[bit(1, rw)]
        ref_reset: bool,
        /// AMBA (CPU_1x) domain reset.
        #[bit(0, rw)]
        cpu_1x_reset: bool,
    }

    /// PL (FPGA) software reset control for the four PL reset lines.
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
        /// PL reset line 3.
        #[bit(3, rw)]
        fpga_3: bool,
        /// PL reset line 2.
        #[bit(2, rw)]
        fpga_2: bool,
        /// PL reset line 1.
        #[bit(1, rw)]
        fpga_1: bool,
        /// PL reset line 0.
        #[bit(0, rw)]
        fpga_0: bool,
    }

    /// APU (Cortex-A9) software reset control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CpuResetControl {
        /// Peripheral reset, for example the SCU, GIC, timers and watchdog.
        #[bit(8, rw)]
        peripheral_reset: bool,
        /// Stop the CPU 1 clock.
        #[bit(5, rw)]
        cpu1_clockstop: bool,
        /// Stop the CPU 0 clock.
        #[bit(4, rw)]
        cpu0_clockstop: bool,
        /// CPU 1 core, debug and watchdog software reset.
        #[bit(1, rw)]
        cpu1_reset: bool,
        /// CPU 0 core, debug and watchdog software reset.
        #[bit(0, rw)]
        cpu0_reset: bool,
    }

    /// Processing system software reset control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PsResetControl {
        /// Resets the whole processing system, equivalent to a power-on reset.
        #[bit(0, rw)]
        soft_reset: bool,
    }

    /// Generic single-bit software reset control, used for DDR, DMA and OCM.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ResetControlSingleBit {
        /// Software reset.
        #[bit(0, rw)]
        reset: bool,
    }

    /// Central interconnect software reset control.
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

    /// Selects which reset the APU watchdog timeout triggers.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ApuWatchdogTarget {
        /// Same system level as PS_SRST_B.
        PsSrstB = 1,
        /// Resets only the CPU associated with the watchdog.
        CpuAssociatedWithWdt = 0,
    }

    /// APU watchdog timer reset target selection.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct WatchTimerResetControl {
        /// Reset target for CPU 1's watchdog.
        #[bit(1, rw)]
        apu_wdt_1_reset_target: ApuWatchdogTarget,
        /// Reset target for CPU 0's watchdog.
        #[bit(0, rw)]
        apu_wdt_0_reset_target: ApuWatchdogTarget,
    }
}

/// Reset control block.
///
/// All reset signal bits are active high, writing a 1 asserts the reset.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct ResetControl {
    /// Processing System Software reset control
    pss: PsResetControl,
    /// DDR reset control
    ddr: ResetControlSingleBit,
    /// Central interconnect reset control
    topsw: ResetControlInterconnect,
    /// DMA controller reset control
    dmac: ResetControlSingleBit,
    /// USB reset control
    usb: DualClockReset,
    /// Ethernet reset control
    eth: EthernetReset,
    /// SDIO reset control
    sdio: DualRefAndClockResetSdio,
    /// SPI reset control
    spi: DualRefAndClockResetSpiUart,
    /// CAN reset control
    can: DualClockReset,
    /// I2C reset control
    i2c: DualClockReset,
    /// UART reset control
    uart: DualRefAndClockResetSpiUart,
    /// GPIO reset control
    gpio: GpioClockReset,
    /// Linear QSPI reset control
    lqspi: ResetControlQspiSmc,
    /// Static memory controller reset control
    smc: ResetControlQspiSmc,
    /// On-chip memory reset control
    ocm: ResetControlSingleBit,
    _gap0: u32,
    /// PL (FPGA) reset control
    fpga: FpgaResetControl,
    /// APU (Cortex-A9) reset control
    a9_cpu: CpuResetControl,
    _gap1: u32,
    /// APU watchdog reset target control
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
