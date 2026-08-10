use arbitrary_int::traits::Integer as _;
use arbitrary_int::u6;
use zynq7000::slcr::ClockControlRegisters;
pub use zynq7000::slcr::clocks::FpgaClockControl;
pub use zynq7000::slcr::clocks::SrcSelIo;

pub use crate::clocks::DivisorZeroError;
pub use crate::time::Hertz;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FpgaClockConfig(pub [FpgaClockControl; 4]);

impl FpgaClockConfig {
    /// The common default target frequency used by [Self::calculate_default].
    pub const DEFAULT_FPGA_CLK: Hertz = Hertz::from_raw(100_000_000);

    /// Configures all 4 FPGA output clocks (FCLK0-3) to the common default of 100 MHz
    /// each, sourced from the IO PLL.
    pub fn calculate_default(ps_clk_freq: Hertz) -> Result<Self, DivisorZeroError> {
        Self::calculate(ps_clk_freq, [Self::DEFAULT_FPGA_CLK; 4])
    }

    pub fn calculate(ps_clk_freq: Hertz, clks: [Hertz; 4]) -> Result<Self, DivisorZeroError> {
        Self::calculate_generic(ps_clk_freq, SrcSelIo::IoPll, clks)
    }

    pub fn calculate_generic(
        ps_clk_freq: Hertz,
        src_sel: SrcSelIo,
        clks: [Hertz; 4],
    ) -> Result<Self, DivisorZeroError> {
        let clk_regs = unsafe { ClockControlRegisters::new_mmio_fixed() };

        let arm_pll_cfg = clk_regs.read_arm_pll_ctrl();
        let io_pll_cfg = clk_regs.read_io_pll_ctrl();
        let ddr_pll_cfg = clk_regs.read_ddr_pll_ctrl();

        if arm_pll_cfg.fdiv().value() == 0
            || io_pll_cfg.fdiv().value() == 0
            || ddr_pll_cfg.fdiv().value() == 0
        {
            return Err(DivisorZeroError(super::ClockModuleId::Pll));
        }
        let arm_pll_out = ps_clk_freq * arm_pll_cfg.fdiv().into();
        let io_pll_out = ps_clk_freq * io_pll_cfg.fdiv().into();
        let ddr_pll_out = ps_clk_freq * ddr_pll_cfg.fdiv().into();
        let ref_in = match src_sel {
            SrcSelIo::IoPll | SrcSelIo::IoPllAlt => io_pll_out,
            SrcSelIo::ArmPll => arm_pll_out,
            SrcSelIo::DdrPll => ddr_pll_out,
        };
        let mut clock_config: [FpgaClockControl; 4] =
            [FpgaClockControl::default().with_srcsel(src_sel); 4];
        let mut divisors = [z7_clock_calc::fpga::DivisorPair { div0: 1, div1: 1 }; 4];
        for (index, clk) in clks.iter().enumerate() {
            // All 4 outputs currently share the same `ref_in` (single `src_sel` for the
            // whole config), so an identical target frequency always yields an identical
            // divisor pair - reuse an already-computed result instead of redoing the search.
            let pair = match clks[..index].iter().position(|prev| prev == clk) {
                Some(prev_index) => divisors[prev_index],
                None => z7_clock_calc::fpga::find_two_stage_divisors(
                    ref_in,
                    *clk,
                    u6::MAX.value() as u32,
                ),
            };
            divisors[index] = pair;
            clock_config[index].set_divisor_0(u6::new(pair.div0 as u8));
            clock_config[index].set_divisor_1(u6::new(pair.div1 as u8));
        }
        Ok(FpgaClockConfig(clock_config))
    }
}

/// Configures the FPGA clocks.
pub fn configure(clock_config: FpgaClockConfig) {
    let mut clk_regs = unsafe { ClockControlRegisters::new_mmio_fixed() };

    clk_regs.fpga_0_clk_ctrl().write_ctrl(clock_config.0[0]);
    clk_regs.fpga_1_clk_ctrl().write_ctrl(clock_config.0[1]);
    clk_regs.fpga_2_clk_ctrl().write_ctrl(clock_config.0[2]);
    clk_regs.fpga_3_clk_ctrl().write_ctrl(clock_config.0[3]);
}

// The divisor search itself is tested in the `z7-clock-calc` crate, which - unlike this
// one - can build and run its test suite on the host.
