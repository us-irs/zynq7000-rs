use zynq7000::{
    sdio::SdClockDivisor,
    slcr::{clocks::SrcSelIo, reset::DualRefAndClockReset},
};

use crate::{clocks::Clocks, slcr::Slcr, time::Hertz};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SdioId {
    Sdio0,
    Sdio1,
}

pub struct SdioDivisors {
    /// Divisor which will be used during the initialization phase when ACMD41 is issued.
    ///
    /// The SD card specification mentions that the clock needs to be between 100 and 400 kHz for
    /// that phase.
    pub divisor_init_phase: SdClockDivisor,
    /// Divisor for the regular data transfer phase. Common target speeds are 25 MHz or 50 MHz.
    pub divisor_normal: SdClockDivisor,
}

impl SdioDivisors {
    pub fn calculate(src_sel: SrcSelIo, clocks: &Clocks, target_speed: Hertz) {

    }
}
/// Reset the UART peripheral using the SLCR reset register for UART.
///
/// Please note that this function will interfere with an already configured
/// UART instance.
#[inline]
pub fn reset(id: SdioId, cycles: u32) {
    let assert_reset = match id {
        SdioId::Sdio0 => DualRefAndClockReset::builder()
            .with_periph1_ref_rst(false)
            .with_periph0_ref_rst(true)
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        SdioId::Sdio1 => DualRefAndClockReset::builder()
            .with_periph1_ref_rst(true)
            .with_periph0_ref_rst(false)
            .with_periph1_cpu1x_rst(true)
            .with_periph0_cpu1x_rst(false)
            .build(),
    };
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl().write_sdio(assert_reset);
            // Keep it in reset for a few cycle.. not sure if this is necessary.
            for _ in 0..cycles {
                cortex_ar::asm::nop();
            }
            regs.reset_ctrl().write_sdio(DualRefAndClockReset::DEFAULT);
        });
    }
}
