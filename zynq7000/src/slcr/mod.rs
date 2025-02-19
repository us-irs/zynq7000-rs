//! System Level Control Registers (slcr)
//!
//! Writing any of these registers required unlocking the SLCR first.
use arbitrary_int::u4;
pub use clocks::{ClockControl, MmioClockControl};
pub use reset::{MmioResetControl, ResetControl};

const SLCR_BASE_ADDR: usize = 0xF8000000;
const CLOCK_CONTROL_OFFSET: usize = 0x100;
const RESET_BLOCK_OFFSET: usize = 0x200;
const GPIOB_OFFSET: usize = 0xB00;
const DDRIOB_OFFSET: usize = 0xB40;

pub mod clocks;
pub mod mio;
pub mod reset;

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct DdrIoB {
    ddriob_addr0: u32,
    ddriob_addr1: u32,
    ddriob_data0: u32,
    ddriob_data1: u32,
    ddriob_diff0: u32,
    ddriob_diff1: u32,
    ddriob_clock: u32,
    ddriob_drive_slew_addr: u32,
    ddriob_drive_slew_data: u32,
    ddriob_drive_slew_diff: u32,
    ddriob_drive_slew_clock: u32,
    ddriob_ddr_ctrl: u32,
    ddriob_dci_ctrl: u32,
    ddriob_dci_status: u32,
}

impl DdrIoB {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioDdrIoB<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR + DDRIOB_OFFSET) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<DdrIoB>(), 0x38);

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct GpiobCtrl {
    ctrl: u32,
    cfg_cmos18: u32,
    cfg_cmos25: u32,
    cfg_cmos33: u32,
    _gap17: u32,
    cfg_hstl: u32,
    drvr_bias_ctrl: u32,
}

impl GpiobCtrl {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioGpiobCtrl<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR + GPIOB_OFFSET) }
    }
}

#[bitbybit::bitfield(u32)]
pub struct BootModeRegister {
    #[bit(4, r)]
    pll_bypass: bool,
    #[bits(0..=3, r)]
    boot_mode: u4,
}

#[bitbybit::bitenum(u4)]
#[derive(Debug, PartialEq, Eq)]
pub enum LevelShifterConfig {
    DisableAll = 0x00,
    EnablePsToPl = 0xA,
    EnableAll = 0xF,
}

#[bitbybit::bitfield(u32)]
pub struct LevelShifterReg {
    #[bits(0..=3, rw)]
    user_lvl_shftr_en: Option<LevelShifterConfig>,
}

/// System Level Control Registers
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Slcr {
    /// Secure configuration lock.
    scl: u32,
    /// SLCR write protection lock
    lock: u32,
    /// SLCR write protection unlock
    unlock: u32,
    /// SLCR write protection status
    lock_status: u32,

    _gap0: [u32; 0x3C],

    #[mmio(inner)]
    clk_ctrl: ClockControl,

    _gap1: [u32; 0x0E],

    #[mmio(inner)]
    reset_ctrl: ResetControl,

    _gap2: [u32; 0x02],

    reboot_status: u32,
    boot_mode: BootModeRegister,

    _gap3: [u32; 0x28],

    apu_ctrl: u32,
    wdt_clk_set: u32,

    _gap4: [u32; 0x4E],

    tz_dma_ns: u32,
    tz_dma_irq_ns: u32,
    tz_dma_periph_ns: u32,

    _gap5: [u32; 0x39],

    pss_idcode: u32,

    _gap6: [u32; 0x33],

    ddr_urgent: u32,
    _gap7: [u32; 0x02],
    ddr_cal_start: u32,
    _gap8: u32,
    ddr_ref_start: u32,
    ddr_cmd_status: u32,
    ddr_urgent_sel: u32,
    ddr_dfi_status: u32,

    _gap9: [u32; 0x37],

    mio_pins: [mio::Config; 0x36],

    _gap10: [u32; 0x0B],

    mio_loopback: u32,
    _gap11: u32,
    mio_mst_tri_0: u32,
    mio_mst_tri_1: u32,
    _gap12: [u32; 7],
    sd_0_wp_cd_sel: u32,
    sd_1_wp_cd_sel: u32,

    _gap13: [u32; 0x32],

    lvl_shftr_en: LevelShifterReg,

    _gap14: [u32; 0x03],

    ocm_cfg: u32,

    _gap15: [u32; 0x42],

    reserved: u32,

    _gap16: [u32; 0x38],

    _gap18: [u32; 0x09],

    #[mmio(inner)]
    gpiob: GpiobCtrl,

    #[mmio(inner)]
    ddriob: DdrIoB,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Slcr>(), 0xB78);

impl Slcr {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioSlcr<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR) }
    }
}
