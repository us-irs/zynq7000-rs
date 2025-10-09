//! # DDR module
//!
//! ## Examples
//!
//! - [Zedboard FSBL](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq/zedboard-fsbl)
use arbitrary_int::u6;
use zynq7000::ddrc::MmioDdrController;

use crate::{
    BootMode,
    clocks::pll::{PllConfig, configure_ddr_pll},
    time::Hertz,
};

pub mod ll;

pub use ll::{DdrcConfigSet, DdriobConfigSet};

#[derive(Debug, Clone, Copy)]
pub struct DdrClockSetupConfig {
    pub ps_clk: Hertz,
    pub ddr_clk: Hertz,
    pub ddr_3x_div: u6,
    pub ddr_2x_div: u6,
}

impl DdrClockSetupConfig {
    pub const fn new(ps_clk: Hertz, ddr_clk: Hertz, ddr_3x_div: u6, ddr_2x_div: u6) -> Self {
        Self {
            ps_clk,
            ddr_clk,
            ddr_3x_div,
            ddr_2x_div,
        }
    }
}

/// This completely sets up the DDR module for DDR3 operation.
///
/// It performs the following steps accoridng to the functional programming model of the
/// DDR memory controller, TRM p.323.
///
///  1. Configures the DDR PLL to the target clock frequency.
///  2. Configures the DDR clocks based on the user-provided configuration.
///  3. Configures and sets up the DDR I/O buffers (DDR IOB) module.
///  4. Configures and sets up the DDR controller (DDRC) module.
///
/// This function consumes the DDRC register block once and thus provides a safe interface for DDR
/// initialization.
pub fn configure_ddr_for_ddr3(
    mut ddrc_regs: MmioDdrController<'static>,
    boot_mode: BootMode,
    clk_setup_cfg: DdrClockSetupConfig,
    ddriob_cfg: &DdriobConfigSet,
    ddr_cfg: &DdrcConfigSet,
) {
    // Set the DDR PLL output frequency to an even multiple of the operating frequency,
    // as recommended by the DDR documentation.
    configure_ddr_pll(
        boot_mode,
        PllConfig::new_from_target_clock(clk_setup_cfg.ps_clk, clk_setup_cfg.ddr_clk).unwrap(),
    );
    // Safety: Only done once here during start-up.
    let ddr_clks = unsafe {
        crate::clocks::DdrClocks::new_with_2x_3x_init(
            clk_setup_cfg.ddr_clk,
            clk_setup_cfg.ddr_3x_div,
            clk_setup_cfg.ddr_2x_div,
        )
    };
    let dci_clk_cfg = ll::calculate_dci_divisors(&ddr_clks);

    ddrc_regs.modify_ddrc_ctrl(|mut val| {
        val.set_soft_reset(zynq7000::ddrc::regs::SoftReset::Reset);
        val
    });

    // Safety: This is only called once during DDR initialization.
    unsafe {
        ll::configure_iob(ddriob_cfg);
        // Do not wait for completion, it takes a bit of time. We can set all the DDR config registers
        // before polling for completion.
        ll::calibrate_iob_impedance_for_ddr3(dci_clk_cfg, false);
    }
    ll::configure_ddr_config(&mut ddrc_regs, ddr_cfg);
    // Safety: This is only called once during DDR initialization, and we only modify DDR related
    // registers.
    let slcr = unsafe { crate::slcr::Slcr::steal() };
    let ddriob_shared = slcr.regs().ddriob_shared();
    // Wait for DDR IOB impedance calibration to complete first.
    while !ddriob_shared.read_dci_status().done() {
        cortex_ar::asm::nop();
    }
    log::debug!("DDR IOB impedance calib done");

    // Now take the DDR out of reset.
    ddrc_regs.modify_ddrc_ctrl(|mut val| {
        val.set_soft_reset(zynq7000::ddrc::regs::SoftReset::Active);
        val
    });
    // Wait until the DDR setup has completed.
    while ddrc_regs.read_mode_status().operating_mode()
        != zynq7000::ddrc::regs::OperatingMode::NormalOperation
    {
        // Wait for the soft reset to complete.
        cortex_ar::asm::nop();
    }
}

pub mod memtest {
    #[derive(Debug, thiserror::Error)]
    pub enum MemTestError {
        #[error("memory address is not aligned to 4 bytes")]
        AddrNotAligned,
        #[error("memory test error")]
        Memory {
            addr: usize,
            expected: u32,
            found: u32,
        },
    }

    /// # Safety
    ///
    /// This tests writes and reads on a memory block starting at the base address
    /// with the size `words` times 4.
    pub unsafe fn walking_zero_test(base_addr: usize, words: usize) -> Result<(), MemTestError> {
        unsafe { walking_value_test(true, base_addr, words) }
    }

    /// # Safety
    ///
    /// This tests writes and reads on a memory block starting at the base address
    /// with the size `words` times 4.
    pub unsafe fn walking_one_test(base_addr: usize, words: usize) -> Result<(), MemTestError> {
        unsafe { walking_value_test(true, base_addr, words) }
    }

    /// # Safety
    ///
    /// This tests writes and reads on a memory block starting at the base address
    /// with the size `words` times 4.
    pub unsafe fn walking_value_test(
        walking_zero: bool,
        base_addr: usize,
        words: usize,
    ) -> Result<(), MemTestError> {
        if words == 0 {
            return Ok(());
        }
        if !base_addr.is_multiple_of(4) {
            return Err(MemTestError::AddrNotAligned);
        }
        let base_ptr = base_addr as *mut u32;

        // For each bit position 0..31 generate pattern = 1 << bit
        for bit in 0..32 {
            let pattern = if walking_zero {
                !(1u32 << bit)
            } else {
                1u32 << bit
            };

            // write pass
            for i in 0..words {
                unsafe {
                    let p = base_ptr.add(i);
                    core::ptr::write_volatile(p, pattern);
                }
            }

            // read/verify pass
            for i in 0..words {
                let val;
                unsafe {
                    let p = base_ptr.add(i) as *const u32;
                    val = core::ptr::read_volatile(p);
                }
                if val != pattern {
                    return Err(MemTestError::Memory {
                        addr: base_addr + i * 4,
                        expected: pattern,
                        found: val,
                    });
                }
            }
        }
        Ok(())
    }

    /// # Safety
    ///
    /// This tests writes and reads on a memory block starting at the base address
    /// with the size `words` times 4.
    pub unsafe fn checkerboard_test(base_addr: usize, words: usize) -> Result<(), MemTestError> {
        if words == 0 {
            return Ok(());
        }
        if !base_addr.is_multiple_of(4) {
            return Err(MemTestError::AddrNotAligned);
        }

        let base_ptr = base_addr as *mut u32;
        let patterns = [0xAAAAAAAAu32, 0x55555555u32];

        for &pattern in &patterns {
            // Write pass
            for i in 0..words {
                let value = if i % 2 == 0 { pattern } else { !pattern };
                unsafe {
                    core::ptr::write_volatile(base_ptr.add(i), value);
                }
            }

            // Read/verify pass
            for i in 0..words {
                let expected = if i % 2 == 0 { pattern } else { !pattern };
                let val = unsafe { core::ptr::read_volatile(base_ptr.add(i)) };

                if val != expected {
                    return Err(MemTestError::Memory {
                        addr: base_addr + i * 4,
                        expected,
                        found: val,
                    });
                }
            }
        }

        Ok(())
    }
}
