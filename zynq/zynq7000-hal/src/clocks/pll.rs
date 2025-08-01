use core::sync::atomic::AtomicBool;

use arbitrary_int::{u4, u7, u10};

use crate::{BootMode, time::Hertz};

/// Minimal value based on Zynq-7000 TRM Table 25-6, p.744
pub const PLL_MUL_MIN: u32 = 13;
/// Maximum value based on Zynq-7000 TRM Table 25-6, p.744
pub const PLL_MUL_MAX: u32 = 66;

static ARM_PLL_INIT: AtomicBool = AtomicBool::new(false);
static IO_PLL_INIT: AtomicBool = AtomicBool::new(false);
static DDR_PLL_INIT: AtomicBool = AtomicBool::new(false);

#[derive(thiserror::Error, Debug, Clone, Copy, PartialEq, Eq)]
#[error("pll muliplier value {0} is out of range ({PLL_MUL_MIN}..={PLL_MUL_MAX})")]
pub struct MulOutOfRangeError(pub u32);

#[derive(thiserror::Error, Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllConfigCtorError {
    #[error("invalid input")]
    InvalidInput,
    #[error("pll multiplier out of range: {0}")]
    MulOutOfRange(#[from] MulOutOfRangeError),
}

pub struct PllConfig {
    fdiv: u7,
    charge_pump: u4,
    loop_resistor: u4,
    lock_count: u10,
}

impl PllConfig {
    pub fn new_from_target_clock(
        ps_clk: Hertz,
        target_clk: Hertz,
    ) -> Result<Self, PllConfigCtorError> {
        if ps_clk.raw() == 0 {
            return Err(PllConfigCtorError::InvalidInput);
        }
        let mul = target_clk / ps_clk;
        Self::new(mul).map_err(PllConfigCtorError::MulOutOfRange)
    }
    /// Create a new PLL configuration based on the multiplier value.
    ///
    /// These configuration values are based on the Zynq-7000 TRM Table 25-6, p.744.
    pub fn new(pll_mul: u32) -> Result<Self, MulOutOfRangeError> {
        if !(PLL_MUL_MIN..=PLL_MUL_MAX).contains(&pll_mul) {
            return Err(MulOutOfRangeError(pll_mul));
        }

        Ok(match pll_mul {
            13 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(6),
                u10::new(750),
            ),
            14 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(6),
                u10::new(700),
            ),
            15 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(6),
                u10::new(650),
            ),
            16 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(10),
                u10::new(625),
            ),
            17 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(10),
                u10::new(575),
            ),
            18 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(10),
                u10::new(550),
            ),
            19 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(10),
                u10::new(525),
            ),
            20 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(500),
            ),
            21 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(475),
            ),
            22 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(450),
            ),
            23 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(425),
            ),
            24..=25 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(400),
            ),
            26 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(375),
            ),
            27..=28 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(350),
            ),

            29..=30 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(12),
                u10::new(325),
            ),
            31..=33 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(2),
                u10::new(300),
            ),
            34..=36 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(2),
                u10::new(275),
            ),
            37..=40 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(2),
                u10::new(250),
            ),
            41..=47 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(3),
                u4::new(12),
                u10::new(250),
            ),
            48..=66 => Self::new_raw(
                u7::new(pll_mul as u8),
                u4::new(2),
                u4::new(4),
                u10::new(250),
            ),
            _ => {
                unreachable!()
            }
        })
    }

    /// Create a new PLL configuration with raw values.
    ///
    /// It is recommended to use [Self::new] instead, which creates a configuration
    /// based on a look-up table provided in the Zynq-7000 TRM.
    pub fn new_raw(fdiv: u7, charge_pump: u4, loop_resistor: u4, lock_count: u10) -> Self {
        Self {
            fdiv,
            charge_pump,
            loop_resistor,
            lock_count,
        }
    }
}

/// This function configures the ARM PLL based on the provided [PllConfig].
pub fn configure_arm_pll(boot_mode: BootMode, pll_config: PllConfig) {
    if ARM_PLL_INIT.swap(true, core::sync::atomic::Ordering::SeqCst) {
        return;
    }
    // Safety: This will only run at most once because of the atomic boolean check.
    unsafe { configure_arm_pll_unchecked(boot_mode, pll_config) };
}

/// This function configures the IO PLL based on the provided [PllConfig].
pub fn configure_io_pll(boot_mode: BootMode, pll_config: PllConfig) {
    if IO_PLL_INIT.swap(true, core::sync::atomic::Ordering::SeqCst) {
        return;
    }
    // Safety: This will only run at most once because of the atomic boolean check.
    unsafe { configure_arm_pll_unchecked(boot_mode, pll_config) };
}

/// This function configures the DDR PLL based on the provided [PllConfig].
pub fn configure_ddr_pll(boot_mode: BootMode, pll_config: PllConfig) {
    if DDR_PLL_INIT.swap(true, core::sync::atomic::Ordering::SeqCst) {
        return;
    }
    // Safety: This will only run at most once because of the atomic boolean check.
    unsafe { configure_arm_pll_unchecked(boot_mode, pll_config) };
}

/// This function configures the ARM PLL based on the provided [PllConfig].
///
/// # Safety
///
/// This function should only be called once during system initialization, for example in the
/// first-stage bootloader (FSBL).
pub unsafe fn configure_arm_pll_unchecked(boot_mode: BootMode, pll_config: PllConfig) {
    unsafe {
        crate::slcr::Slcr::with(|slcr| {
            let pll_ctrl_reg = slcr.clk_ctrl().pointer_to_arm_pll_ctrl();
            let pll_cfg_reg = slcr.clk_ctrl().pointer_to_arm_pll_cfg();
            configure_pll_unchecked(
                boot_mode,
                pll_config,
                PllType::Arm,
                slcr,
                pll_ctrl_reg,
                pll_cfg_reg,
            );
        });
    }
}

/// This function configures the IO PLL based on the provided [PllConfig].
///
/// # Safety
///
/// This function should only be called once during system initialization, for example in the
/// first-stage bootloader (FSBL).
pub unsafe fn configure_io_pll_unchecked(boot_mode: BootMode, pll_config: PllConfig) {
    unsafe {
        crate::slcr::Slcr::with(|slcr| {
            let pll_ctrl_reg = slcr.clk_ctrl().pointer_to_io_pll_ctrl();
            let pll_cfg_reg = slcr.clk_ctrl().pointer_to_io_pll_cfg();
            configure_pll_unchecked(
                boot_mode,
                pll_config,
                PllType::Io,
                slcr,
                pll_ctrl_reg,
                pll_cfg_reg,
            );
        });
    }
}

/// This function configures the DDR PLL based on the provided [PllConfig].
///
/// # Safety
///
/// This function should only be called once during system initialization, for example in the
/// first-stage bootloader (FSBL).
pub unsafe fn configure_ddr_pll_unchecked(boot_mode: BootMode, pll_config: PllConfig) {
    unsafe {
        crate::slcr::Slcr::with(|slcr| {
            let pll_ctrl_reg = slcr.clk_ctrl().pointer_to_ddr_pll_ctrl();
            let pll_cfg_reg = slcr.clk_ctrl().pointer_to_ddr_pll_cfg();
            configure_pll_unchecked(
                boot_mode,
                pll_config,
                PllType::Ddr,
                slcr,
                pll_ctrl_reg,
                pll_cfg_reg,
            );
        });
    }
}

enum PllType {
    Io,
    Ddr,
    Arm,
}

impl PllType {
    pub const fn bit_offset_pll_locked(&self) -> usize {
        match self {
            PllType::Io => 2,
            PllType::Ddr => 1,
            PllType::Arm => 0,
        }
    }
}

unsafe fn configure_pll_unchecked(
    boot_mode: BootMode,
    cfg: PllConfig,
    pll_type: PllType,
    slcr: &mut zynq7000::slcr::MmioSlcr<'static>,
    pll_ctrl_reg: *mut zynq7000::slcr::clocks::PllControl,
    pll_cfg_reg: *mut zynq7000::slcr::clocks::PllConfig,
) {
    // Step 1: Program the multiplier and other PLL configuration parameters.
    // New values will only be consumed once the PLL is reset.
    let mut pll_ctrl = unsafe { core::ptr::read_volatile(pll_ctrl_reg) };
    pll_ctrl.set_fdiv(cfg.fdiv);
    unsafe { core::ptr::write_volatile(pll_ctrl_reg, pll_ctrl) };

    let mut pll_cfg = unsafe { core::ptr::read_volatile(pll_cfg_reg) };
    pll_cfg.set_charge_pump(cfg.charge_pump);
    pll_cfg.set_loop_resistor(cfg.loop_resistor);
    pll_cfg.set_lock_count(cfg.lock_count);
    unsafe { core::ptr::write_volatile(pll_cfg_reg, pll_cfg) };

    // Step 2: Force the PLL into bypass mode. If the PLL bypass mode pin is tied high,
    // the PLLs need to be enabled. According to the TRM, this is done by setting the
    // PLL_BYPASS_QUAL bit to 0, which de-asserts the reset to the Arm PLL.
    pll_ctrl = unsafe { core::ptr::read_volatile(pll_ctrl_reg) };
    if boot_mode.pll_config() == zynq7000::slcr::BootPllConfig::Bypassed {
        pll_ctrl.set_bypass_qual(false);
    }
    pll_ctrl.set_bypass_force(true);
    pll_ctrl.set_pwrdwn(false);
    unsafe { core::ptr::write_volatile(pll_ctrl_reg, pll_ctrl) };

    // Step 3: Reset the PLL. This also loads the new configuration.
    pll_ctrl = unsafe { core::ptr::read_volatile(pll_ctrl_reg) };
    pll_ctrl.set_reset(true);
    unsafe { core::ptr::write_volatile(pll_ctrl_reg, pll_ctrl) };
    pll_ctrl.set_reset(false);
    unsafe { core::ptr::write_volatile(pll_ctrl_reg, pll_ctrl) };

    while ((slcr.clk_ctrl().read_pll_status().raw_value() >> pll_type.bit_offset_pll_locked())
        & 0b1)
        != 1
    {
        cortex_ar::asm::nop();
    }

    pll_ctrl = unsafe { core::ptr::read_volatile(pll_ctrl_reg) };
    pll_ctrl.set_bypass_force(false);
    unsafe { core::ptr::write_volatile(pll_ctrl_reg, pll_ctrl) };
}
