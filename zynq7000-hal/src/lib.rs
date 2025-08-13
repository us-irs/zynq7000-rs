//! # HAL for the AMD Zynq 7000 SoC family
//!
//! This repository contains the **H**ardware **A**bstraction **L**ayer (HAL), which is an additional
//! hardware abstraction on top of the [peripheral access API](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq7000).
//!
//! It is the result of reading the datasheet for the device and encoding a type-safe layer over the
//! raw PAC. This crate also implements traits specified by the
//! [embedded-hal](https://github.com/rust-embedded/embedded-hal) project, making it compatible with
//! various drivers in the embedded rust ecosystem.
#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;

use slcr::Slcr;
use zynq7000::slcr::LevelShifterRegister;

pub mod cache;
pub mod clocks;
pub mod eth;
pub mod gic;
pub mod gpio;
pub mod gtc;
pub mod i2c;
pub mod l2_cache;
pub mod log;
pub mod prelude;
pub mod slcr;
pub mod spi;
pub mod time;
pub mod ttc;
pub mod uart;

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
pub enum BootPllConfig {
    Enabled,
    Bypassed,
}

#[derive(Debug)]
pub struct BootMode {
    boot_mode: Option<BootDevice>,
    pll_config: BootPllConfig,
}

impl BootMode {
    #[allow(clippy::new_without_default)]
    /// Create a new boot mode information structure by reading the boot mode register from the
    /// fixed SLCR block.
    pub fn new() -> Self {
        // Safety: Only read a read-only register here.
        Self::new_with_raw_reg(
            unsafe { zynq7000::slcr::Slcr::new_mmio_fixed() }
                .read_boot_mode()
                .raw_value(),
        )
    }

    fn new_with_raw_reg(raw_register: u32) -> Self {
        let msb_three_bits = (raw_register >> 1) & 0b111;

        let boot_mode = match msb_three_bits {
            0b000 => {
                if raw_register & 0b1 == 0 {
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
        let pll_config = if (raw_register >> 4) & 0b1 == 0 {
            BootPllConfig::Enabled
        } else {
            BootPllConfig::Bypassed
        };
        Self {
            boot_mode,
            pll_config,
        }
    }
    pub fn boot_device(&self) -> Option<BootDevice> {
        self.boot_mode
    }

    pub const fn pll_enable(&self) -> BootPllConfig {
        self.pll_config
    }
}

/// This configures the level shifters between the programmable logic (PL) and the processing
/// system (PS).
///
/// The Zynq-7000 TRM p.32 specifies more information about this register and how to use it.
pub fn configure_level_shifter(config: zynq7000::slcr::LevelShifterConfig) {
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
pub fn enable_amba_periph_clk(select: PeriphSelect) {
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
                    PeriphSelect::Spi0 => val.set_spi_1_1x_clk_act(true),
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
                    PeriphSelect::Spi0 => val.set_spi_1_1x_clk_act(false),
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

#[allow(dead_code)]
pub(crate) mod sealed {
    pub trait Sealed {}
}
