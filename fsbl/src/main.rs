//! Simple FSBL.
#![no_std]
#![no_main]

use arbitrary_int::u6;
use core::panic::PanicInfo;
use cortex_ar::asm::nop;
use log::error;
use zynq7000_hal::{
    clocks::pll::{configure_arm_pll, configure_ddr_pll, configure_io_pll, PllConfig}, ddr::{calculate_dci_divisors, calibrate_iob_impedance_for_ddr3, configure_iob}, time::Hertz, BootMode
};
use zynq7000_rt as _;

pub mod ddr_cfg;

// PS clock input frequency.
const PS_CLK: Hertz = Hertz::from_raw(33_333_333);

/// 1600 MHz.
const ARM_CLK: Hertz = Hertz::from_raw(1_600_000_000);
/// 1000 MHz.
const IO_CLK: Hertz = Hertz::from_raw(1_000_000_000);

/// DDR frequency for the MT41K128M16JT-125 device.
const DDR_FREQUENCY: Hertz = Hertz::from_raw(533_333_333);

/// 1067 MHz.
const DDR_CLK: Hertz = Hertz::from_raw(2 * DDR_FREQUENCY.raw());

/// Entry point (not called like a normal main function)
#[unsafe(no_mangle)]
pub extern "C" fn boot_core(cpu_id: u32) -> ! {
    if cpu_id != 0 {
        panic!("unexpected CPU ID {}", cpu_id);
    }
    main();
}

#[unsafe(export_name = "main")]
pub fn main() -> ! {
    let boot_mode = BootMode::new_from_regs();
    // The unwraps are okay here, the provided clock frequencies are standard values also used
    // by other Xilinx tools.
    configure_arm_pll(
        boot_mode,
        PllConfig::new_from_target_clock(PS_CLK, ARM_CLK).unwrap(),
    );
    configure_io_pll(
        boot_mode,
        PllConfig::new_from_target_clock(PS_CLK, IO_CLK).unwrap(),
    );

    // Set the DDR PLL output frequency to an even multiple of the operating frequency,
    // as recommended by the DDR documentation.
    configure_ddr_pll(
        boot_mode,
        PllConfig::new_from_target_clock(PS_CLK, DDR_CLK).unwrap(),
    );
    // Safety: Only done once here during start-up.
    let ddr_clks = unsafe {
        zynq7000_hal::clocks::DdrClocks::new_with_2x_3x_init(DDR_CLK, u6::new(2), u6::new(3))
    };
    let dci_clk_cfg = calculate_dci_divisors(&ddr_clks);
    calibrate_iob_impedance_for_ddr3(dci_clk_cfg, false);
    configure_iob(&ddr_cfg::DDRIOB_CFG_SET_ZEDBOARD);

    loop {
        cortex_ar::asm::nop();
    }
}

#[zynq7000_rt::exception(DataAbort)]
fn data_abort_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(Undefined)]
fn undefined_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(PrefetchAbort)]
fn prefetch_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

/// Panic handler
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {info:?}");
    loop {}
}
