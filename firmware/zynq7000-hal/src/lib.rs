//! # HAL for the AMD Zynq 7000 SoC family
//!
//! This repository contains the **H**ardware **A**bstraction **L**ayer (HAL), which is an additional
//! hardware abstraction on top of the [peripheral access API](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq7000).
//!
//! It is the result of reading the datasheet for the device and encoding a type-safe layer over the
//! raw PAC. This crate also implements traits specified by the
//! [embedded-hal](https://github.com/rust-embedded/embedded-hal) project, making it compatible with
//! various drivers in the embedded rust ecosystem.
//!
//! ## Features
//!
//! * `defmt` - Activates `defmt` support for various data structures
//! * `7z010-7z007s-clg225`  - Chip variant which has a lower pin count
//! * `time-driver-gtc` - Access to the `embassy-time` driver API which uses the global timer
//!   counter (GTC).
//! * `alloc` - Enables `extern crate alloc` and APIs which need it
//! * `first-segment-ddr-attr` - Maps the first 1 MB MMU segment with the DDR memory attribute
//!   instead of the OCM one, to allow accessing DDR through addresses 0x8000 to 0x10_0000. See
//!   [`mmu::section_attrs::FIRST_SEGMENT`].
//!
//! ## Examples
//!
//! All examples can be found inside the [examples folder](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/examples)
//! and [firmware folder](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware) of the project
#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

#[cfg(feature = "alloc")]
extern crate alloc;

pub use slcr::Slcr;
use zynq7000::{
    SpiClockPhase, SpiClockPolarity,
    slcr::{BootModeRegister, BootPllConfig, LevelShifterRegister},
};

pub mod cache;
pub mod clocks;
pub mod ddr;
pub mod eth;
pub mod gic;
pub mod gpio;
pub mod gtc;
pub mod i2c;
pub mod interrupt;
pub mod l2_cache;
pub mod log;
pub mod mmu;
pub mod mmu_table;
pub mod pl;
pub mod prelude;
pub mod priv_tim;
pub mod qspi;
pub mod sd;
pub mod slcr;
pub mod spi;
pub mod time;
#[cfg(feature = "time-driver-gtc")]
pub mod time_driver_gtc;
pub mod ttc;
pub mod uart;

pub use gic::{Interrupt, PpiInterrupt, SpiInterrupt};
pub use interrupt::{generic_interrupt_handler, register_interrupt};
pub use zynq7000 as pac;
pub use zynq7000::slcr::LevelShifterConfig;

/// Identifies one of the two physical Cortex-A9 cores.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CoreId {
    Cpu0 = 0,
    Cpu1 = 1,
}

/// Reads the ID of the core this is called on, from MPIDR's affinity level 0 field.
#[inline]
pub fn core_id() -> CoreId {
    if aarch32_cpu::register::mpidr::Mpidr::read().0 & 0x3 == 0 {
        CoreId::Cpu0
    } else {
        CoreId::Cpu1
    }
}

#[derive(Debug, thiserror::Error)]
pub enum InitError {
    #[error("peripheral singleton was already taken")]
    PeripheralsAlreadyTaken,
}

#[derive(Debug, thiserror::Error)]
pub enum SecondaryCoreInitError {
    #[error("secondary core init was already performed")]
    AlreadyInitialized,
    #[error("secondary core init must not be called from the primary core")]
    CalledFromPrimaryCore,
}

#[derive(Debug)]
pub enum InteruptConfig {
    /// GIC is configured to route all interrupts to CPU0. Suitable if the software handles all
    /// the interrupts and only runs on CPU0.
    AllInterruptsToCpu0,
    /// Minimal configuration which does not route any interrupts but still enables them.
    Minimal,
}

#[derive(Debug)]
#[non_exhaustive]
pub struct Config {
    /// Basic initialization of the L2 cache.
    pub init_l2_cache: bool,
    /// If this has some value, it will configure the level shifter between PS and PL.
    pub level_shifter_config: Option<LevelShifterConfig>,
    /// If this has some value, it configures the GIC to pre-defined settings.
    pub interrupt_config: Option<InteruptConfig>,
    /// The PL starts in reset state after power-up. A first-stage bootloader is expected to clear
    /// this, but not every boot flow goes through one that does (e.g. JTAG-loaded applications), so
    /// every application relying on this generic init routine gets it unconditionally. Calling this
    /// when the PL is already out of reset is harmless.
    pub deassert_pl_reset: bool,
    /// If true, calls [`mmu::init`] to set up and enable the MMU and cache. The run-time
    /// startup code no longer does this itself, so this must be enabled (or `mmu::init` called
    /// manually) for the MMU to be active at all.
    pub init_mmu: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            init_l2_cache: true,
            level_shifter_config: Some(LevelShifterConfig::EnableAll),
            interrupt_config: Some(InteruptConfig::AllInterruptsToCpu0),
            deassert_pl_reset: true,
            init_mmu: true,
        }
    }
}

/// Utility function to perform common initialization steps.
pub fn init(config: Config) -> Result<zynq7000::Peripherals, InitError> {
    if config.init_mmu {
        // Safety: Initialization function is only called once.
        unsafe {
            crate::mmu::init();
        }
    }
    let mut periphs = zynq7000::Peripherals::take().ok_or(InitError::PeripheralsAlreadyTaken)?;
    if config.init_l2_cache {
        l2_cache::init_with_defaults(&mut periphs.l2c);
    }
    if let Some(config) = config.level_shifter_config {
        configure_level_shifter(config);
    }
    if config.deassert_pl_reset {
        // The PL is in reset state after power-up. A first-stage bootloader is expected to clear
        // this, but not every boot flow goes through one that does (e.g. JTAG-loaded applications),
        // so every application relying on this generic init routine gets it unconditionally. Calling
        // this when the PL is already out of reset is harmless.
        pl::deassert_reset();
    }
    if let Some(interrupt_config) = config.interrupt_config {
        let mut gic = gic::Configurator::new_with_init(periphs.gicc, periphs.gicd);
        match interrupt_config {
            InteruptConfig::AllInterruptsToCpu0 => {
                gic.enable_all_interrupts();
                gic.set_all_spi_interrupt_targets_cpu0();
            }
            InteruptConfig::Minimal => (),
        }
        gic.enable();
        unsafe {
            gic.enable_interrupts();
        }
    }

    Ok(unsafe { zynq7000::Peripherals::steal() })
}

/// Configuration for [`init_secondary_core`].
#[derive(Debug, Clone, Copy)]
pub struct SecondaryCoreConfig {
    /// Initializes and enables the MMU and cache for this core. This state is genuinely
    /// per-core, so every core needs to do this itself, even though [`init`] already did it for
    /// the primary core.
    pub init_mmu: bool,
    /// Configures this core's own banked GIC CPU interface state (PPI enable bits, priority
    /// mask, CPU interface enable) and unmasks IRQs on this core. Requires the primary core to
    /// have already configured the GIC's shared distributor state via [`init`].
    pub init_gic: bool,
}

impl Default for SecondaryCoreConfig {
    fn default() -> Self {
        Self {
            init_mmu: true,
            init_gic: true,
        }
    }
}

static SECONDARY_CORE_INIT_DONE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Utility function to perform common initialization steps on a secondary core.
///
/// This is [`init`]'s counterpart for any core other than the primary one. It only touches
/// core specific state: the MMU/cache and the GIC's banked CPU interface state
/// (PPI enable bits, priority mask, CPU interface enable).
///
/// Like [`init`] with the peripheral singleton, this can only succeed once: a second call from
/// any core returns [`SecondaryCoreInitError::AlreadyInitialized`]. It also checks that it is
/// not called from the primary core, returning
/// [`SecondaryCoreInitError::CalledFromPrimaryCore`] otherwise. The primary core must already
/// have called [`init`] and released this core, e.g. via `zynq7000_rt::smp::start_core1`,
/// before this is called.
pub fn init_secondary_core(config: SecondaryCoreConfig) -> Result<(), SecondaryCoreInitError> {
    if core_id() == CoreId::Cpu0 {
        return Err(SecondaryCoreInitError::CalledFromPrimaryCore);
    }
    if SECONDARY_CORE_INIT_DONE.swap(true, core::sync::atomic::Ordering::Relaxed) {
        return Err(SecondaryCoreInitError::AlreadyInitialized);
    }

    if config.init_mmu {
        // Safety: Guarded above to run at most once, only from a secondary core.
        unsafe {
            crate::mmu::init();
        }
    }
    if config.init_gic {
        // Safety: We only touch our own banked GIC state here. The primary core already
        // configured the shared distributor state before releasing this core.
        let mut gic = unsafe { gic::Configurator::steal() };
        gic.enable_all_ppi_interrupts();
        // This must be done per core.
        gic.set_priority_mask(0xff);
        gic.enable();
        unsafe {
            gic.enable_interrupts();
        }
    }
    Ok(())
}

/// This enumeration encodes the various boot sources.
#[derive(Debug, Copy, Clone)]
pub enum BootDevice {
    JtagCascaded,
    JtagIndependent,
    Nor,
    Nand,
    Qspi,
    SdCard,
}

#[derive(Debug, Copy, Clone)]
pub struct BootMode {
    boot_mode: Option<BootDevice>,
    pll_config: BootPllConfig,
}

impl BootMode {
    /// Create a new boot mode information structure by reading the boot mode register from the
    /// fixed SLCR block.
    pub fn new_from_regs() -> Self {
        // Safety: Only read a read-only register here.
        Self::new_with_reg(unsafe { zynq7000::slcr::Registers::new_mmio_fixed() }.read_boot_mode())
    }

    fn new_with_reg(boot_mode_reg: BootModeRegister) -> Self {
        let boot_dev = boot_mode_reg.boot_mode();
        let msb_three_bits = (boot_dev.value() >> 1) & 0b111;

        let boot_mode = match msb_three_bits {
            0b000 => {
                if boot_dev.value() & 0b1 == 0 {
                    Some(BootDevice::JtagCascaded)
                } else {
                    Some(BootDevice::JtagIndependent)
                }
            }
            0b001 => Some(BootDevice::Nor),
            0b010 => Some(BootDevice::Nand),
            0b100 => Some(BootDevice::Qspi),
            0b110 => Some(BootDevice::SdCard),
            _ => None,
        };
        Self {
            boot_mode,
            pll_config: boot_mode_reg.pll_config(),
        }
    }

    pub const fn boot_device(&self) -> Option<BootDevice> {
        self.boot_mode
    }

    pub const fn pll_config(&self) -> BootPllConfig {
        self.pll_config
    }
}

/// This configures the level shifters between the programmable logic (PL) and the processing
/// system (PS).
///
/// The Zynq-7000 TRM p.32 specifies more information about this register and how to use it.
pub fn configure_level_shifter(config: LevelShifterConfig) {
    // Safety: We only manipulate the level shift registers.
    unsafe {
        Slcr::with(|slcr_unlocked| {
            slcr_unlocked
                .write_lvl_shftr_en(LevelShifterRegister::new_with_raw_value(config as u32));
        });
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PeriphSelect {
    Smc = 24,
    Lqspi = 23,
    Gpio = 22,
    Uart1 = 21,
    Uart0 = 20,
    I2c1 = 19,
    I2c0 = 18,
    Can1 = 17,
    Can0 = 16,
    Spi1 = 15,
    Spi0 = 14,
    Sdio1 = 11,
    Sdio0 = 10,
    Gem1 = 7,
    Gem0 = 6,
    Usb1 = 3,
    Usb0 = 2,
    Dma = 0,
}

/// Enable the AMBA peripheral clock, which is required to read the registers of a peripheral
/// block.
#[inline]
pub fn enable_amba_peripheral_clock(select: PeriphSelect) {
    unsafe {
        Slcr::with(|regs| {
            regs.clk_ctrl().modify_aper_clk_ctrl(|mut val| {
                match select {
                    PeriphSelect::Smc => val.set_smc_1x_clk_act(true),
                    PeriphSelect::Lqspi => val.set_lqspi_1x_clk_act(true),
                    PeriphSelect::Gpio => val.set_gpio_1x_clk_act(true),
                    PeriphSelect::Uart1 => val.set_uart_1_1x_clk_act(true),
                    PeriphSelect::Uart0 => val.set_uart_0_1x_clk_act(true),
                    PeriphSelect::I2c1 => val.set_i2c_1_1x_clk_act(true),
                    PeriphSelect::I2c0 => val.set_i2c_0_1x_clk_act(true),
                    PeriphSelect::Can1 => val.set_can_1_1x_clk_act(true),
                    PeriphSelect::Can0 => val.set_can_0_1x_clk_act(true),
                    PeriphSelect::Spi1 => val.set_spi_1_1x_clk_act(true),
                    PeriphSelect::Spi0 => val.set_spi_0_1x_clk_act(true),
                    PeriphSelect::Sdio1 => val.set_sdio_1_1x_clk_act(true),
                    PeriphSelect::Sdio0 => val.set_sdio_0_1x_clk_act(true),
                    PeriphSelect::Gem1 => val.set_gem_1_1x_clk_act(true),
                    PeriphSelect::Gem0 => val.set_gem_0_1x_clk_act(true),
                    PeriphSelect::Usb1 => val.set_usb_1_cpu_1x_clk_act(true),
                    PeriphSelect::Usb0 => val.set_usb_0_cpu_1x_clk_act(true),
                    PeriphSelect::Dma => val.set_dma_cpu_2x_clk_act(true),
                }
                val
            })
        });
    }
}

/// Disable the AMBA peripheral clock, which is required to read the registers of a peripheral
/// block.
#[inline]
pub fn disable_amba_periph_clk(select: PeriphSelect) {
    unsafe {
        Slcr::with(|regs| {
            regs.clk_ctrl().modify_aper_clk_ctrl(|mut val| {
                match select {
                    PeriphSelect::Smc => val.set_smc_1x_clk_act(false),
                    PeriphSelect::Lqspi => val.set_lqspi_1x_clk_act(false),
                    PeriphSelect::Gpio => val.set_gpio_1x_clk_act(false),
                    PeriphSelect::Uart1 => val.set_uart_1_1x_clk_act(false),
                    PeriphSelect::Uart0 => val.set_uart_0_1x_clk_act(false),
                    PeriphSelect::I2c1 => val.set_i2c_1_1x_clk_act(false),
                    PeriphSelect::I2c0 => val.set_i2c_0_1x_clk_act(false),
                    PeriphSelect::Can1 => val.set_can_1_1x_clk_act(false),
                    PeriphSelect::Can0 => val.set_can_0_1x_clk_act(false),
                    PeriphSelect::Spi1 => val.set_spi_1_1x_clk_act(false),
                    PeriphSelect::Spi0 => val.set_spi_0_1x_clk_act(false),
                    PeriphSelect::Sdio1 => val.set_sdio_1_1x_clk_act(false),
                    PeriphSelect::Sdio0 => val.set_sdio_0_1x_clk_act(false),
                    PeriphSelect::Gem1 => val.set_gem_1_1x_clk_act(false),
                    PeriphSelect::Gem0 => val.set_gem_0_1x_clk_act(false),
                    PeriphSelect::Usb1 => val.set_usb_1_cpu_1x_clk_act(false),
                    PeriphSelect::Usb0 => val.set_usb_0_cpu_1x_clk_act(false),
                    PeriphSelect::Dma => val.set_dma_cpu_2x_clk_act(false),
                }
                val
            })
        });
    }
}

#[inline]
const fn spi_mode_const_to_cpol_cpha(
    mode: embedded_hal::spi::Mode,
) -> (SpiClockPolarity, SpiClockPhase) {
    match mode {
        embedded_hal::spi::MODE_0 => (
            SpiClockPolarity::QuiescentLow,
            SpiClockPhase::ActiveOutsideOfWord,
        ),
        embedded_hal::spi::MODE_1 => (
            SpiClockPolarity::QuiescentLow,
            SpiClockPhase::InactiveOutsideOfWord,
        ),
        embedded_hal::spi::MODE_2 => (
            SpiClockPolarity::QuiescentHigh,
            SpiClockPhase::ActiveOutsideOfWord,
        ),
        embedded_hal::spi::MODE_3 => (
            SpiClockPolarity::QuiescentHigh,
            SpiClockPhase::InactiveOutsideOfWord,
        ),
    }
}

pub(crate) mod sealed {
    pub trait Sealed {}
}
