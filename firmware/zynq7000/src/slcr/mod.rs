//! System Level Control Registers (slcr)
//!
//! Writing any of these registers required unlocking the SLCR first.
use arbitrary_int::u4;
pub use clocks::{ClockControlRegisters, MmioClockControlRegisters};
pub use reset::{MmioResetControl, ResetControl};

const SLCR_BASE_ADDR: usize = 0xF8000000;
const CLOCK_CONTROL_OFFSET: usize = 0x100;
const RESET_BLOCK_OFFSET: usize = 0x200;
const GPIOB_OFFSET: usize = 0xB00;
const DDRIOB_OFFSET: usize = 0xB40;

/// Clock control registers.
pub mod clocks;
/// DDR I/O buffer configuration registers.
pub mod ddriob;
/// MIO pin configuration registers.
pub mod mio;
/// Peripheral reset control registers.
pub mod reset;

/// Reference voltage selection for GPIOB.
#[bitbybit::bitenum(u3, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VrefSel {
    /// Reference voltage disabled.
    Disabled = 0b000,
    /// 0.9V reference voltage.
    Vref0_9V = 0b001,
}

/// GPIOB bank reference voltage control.
#[bitbybit::bitfield(
    u32,
    default = 0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct GpiobControl {
    /// Enables the reference voltage switch.
    #[bit(11, rw)]
    vref_sw_en: bool,
    /// Reference voltage selection.
    #[bits(4..=6, rw)]
    vref_sel: Option<VrefSel>,
    /// Enables the reference voltage.
    #[bit(0, rw)]
    vref_en: bool,
}

/// GPIOB bank I/O buffer configuration registers.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct GpiobRegisters {
    /// Reference voltage control
    ctrl: GpiobControl,
    /// LVCMOS18 drive strength and slew rate configuration
    cfg_cmos18: u32,
    /// LVCMOS25 drive strength and slew rate configuration
    cfg_cmos25: u32,
    /// LVCMOS33 drive strength and slew rate configuration
    cfg_cmos33: u32,
    _gap17: u32,
    /// HSTL drive strength and slew rate configuration
    cfg_hstl: u32,
    /// Driver bias control
    drvr_bias_ctrl: u32,
}

impl GpiobRegisters {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioGpiobRegisters<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR + GPIOB_OFFSET) }
    }
}

/// Boot PLL bypass configuration, sampled from the boot mode pins.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BootPllConfig {
    /// PLL enabled.
    Enabled = 0,
    /// Disabled and bypassed.
    Bypassed = 1,
}

/// Boot mode strapping pin status, read-only.
#[bitbybit::bitfield(
    u32,
    default = 0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct BootModeRegister {
    /// Boot PLL bypass configuration.
    #[bit(4, r)]
    pll_config: BootPllConfig,
    /// Boot device selection.
    #[bits(0..=3, r)]
    boot_mode: u4,
}

/// PS-to-PL and PL-to-PS level shifter enable configuration.
#[bitbybit::bitenum(u4)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LevelShifterConfig {
    /// Disable all level shifters.
    DisableAll = 0x00,
    /// Enable only the PS-to-PL level shifters.
    EnablePsToPl = 0xA,
    /// Enable all level shifters.
    EnableAll = 0xF,
}

/// PS-PL level shifter enable control.
#[bitbybit::bitfield(
    u32,
    default = 0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct LevelShifterRegister {
    /// Level shifter enable configuration.
    #[bits(0..=3, rw)]
    user_lvl_shftr_en: Option<LevelShifterConfig>,
}

/// MIO peripheral loopback configuration.
#[bitbybit::bitfield(
    u32,
    default = 0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct MioLoopback {
    /// Loop I2C0 to I2C1.
    #[bit(3, rw)]
    i2c0_loop_i2c1: bool,
    /// Loop CAN0 to CAN1.
    #[bit(2, rw)]
    can0_loop_can1: bool,
    /// Loop UART0 to UART1.
    #[bit(1, rw)]
    ua0_loop_ua1: bool,
    /// Loop SPI0 to SPI1.
    #[bit(0, rw)]
    spi0_loop_spi1: bool,
}

/// WDT input clock select.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WdtClockSelect {
    /// Internal clock: CPU1x.
    InternalCpu1x = 0b0,
    /// External clock, either through EMIO or through MIO.
    ExternalEmioOrMio = 0b1,
}

/// WDT input clock select register.
#[bitbybit::bitfield(
    u32,
    default = 0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct WdtClockSelectRegister {
    /// Select bit.
    #[bit(0, rw)]
    sel: WdtClockSelect,
}

/// System Level Control Registers access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Secure configuration lock.
    scl: u32,
    /// SLCR write protection lock
    lock: u32,
    /// SLCR write protection unlock
    unlock: u32,
    /// SLCR write protection status
    lock_status: u32,

    _gap0: [u32; 0x3C],

    #[mmio(Inner)]
    clk_ctrl: ClockControlRegisters,

    _gap1: [u32; 0x0E],

    #[mmio(Inner)]
    reset_ctrl: ResetControl,

    _gap2: [u32; 0x02],

    /// Reboot status, persists across a soft reset
    reboot_status: u32,
    /// Boot mode strapping pins status
    boot_mode: BootModeRegister,

    _gap3: [u32; 0x28],

    /// APU control
    apu_ctrl: u32,
    /// SWDT clock select
    wdt_clk_set: WdtClockSelectRegister,

    _gap4: [u32; 0x4E],

    /// DMA non-secure access control
    tz_dma_ns: u32,
    /// DMA IRQ non-secure access control
    tz_dma_irq_ns: u32,
    /// DMA peripheral non-secure access control
    tz_dma_periph_ns: u32,

    _gap5: [u32; 0x39],

    /// PS IDCODE
    pss_idcode: u32,

    _gap6: [u32; 0x33],

    /// DDR urgent read/write control
    ddr_urgent: u32,
    _gap7: [u32; 0x02],
    /// DDR calibration start trigger
    ddr_cal_start: u32,
    _gap8: u32,
    /// DDR refresh start trigger
    ddr_ref_start: u32,
    /// DDR command status
    ddr_cmd_status: u32,
    /// DDR urgent select
    ddr_urgent_sel: u32,
    /// DDR DFI status
    ddr_dfi_status: u32,

    _gap9: [u32; 0x37],

    /// Per-pin MIO configuration
    mio_pins: [mio::Config; 0x36],

    _gap10: [u32; 0x0B],

    /// MIO peripheral loopback configuration
    mio_loopback: MioLoopback,
    _gap11: u32,
    /// MIO pin tri-state control, lower 32 pins
    mio_mst_tri_0: u32,
    /// MIO pin tri-state control, upper pins
    mio_mst_tri_1: u32,
    _gap12: [u32; 7],
    /// SDIO 0 write-protect/card-detect MIO select
    sd_0_wp_cd_sel: u32,
    /// SDIO 1 write-protect/card-detect MIO select
    sd_1_wp_cd_sel: u32,

    _gap13: [u32; 0x32],

    /// PS-PL level shifter enable control
    lvl_shftr_en: LevelShifterRegister,

    _gap14: [u32; 0x03],

    /// OCM address mapping configuration
    ocm_cfg: u32,

    _gap15: [u32; 0x42],

    /// Xilinx marks this as reserved but writes to it in their low-level L2 cache configuration.
    magic_l2c_register: u32,

    _gap16: [u32; 0x38],

    _gap18: [u32; 0x09],

    #[mmio(Inner)]
    gpiob: GpiobRegisters,

    #[mmio(Inner)]
    ddriob: ddriob::DdrIobRegisters,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0xB78);

impl Registers {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR) }
    }
}
