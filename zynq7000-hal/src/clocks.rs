//! Clock module.
use arbitrary_int::Number;

use zynq7000::slcr::{
    ClockControl,
    clocks::{
        ClockkRatioSelect, DualCommonPeriphIoClockControl, FpgaClockControl, GigEthClockControl,
        SingleCommonPeriphIoClockControl,
    },
};

use super::time::Hertz;

#[derive(Debug)]
pub struct ArmClocks {
    ref_clk: Hertz,
    cpu_1x_clk: Hertz,
    cpu_2x_clk: Hertz,
    cpu_3x2x_clk: Hertz,
    cpu_6x4x_clk: Hertz,
}

impl ArmClocks {
    /// Reference clock provided by ARM PLL which is used to calculate all other clock frequencies.
    pub const fn ref_clk(&self) -> Hertz {
        self.ref_clk
    }

    pub const fn cpu_1x_clk(&self) -> Hertz {
        self.cpu_1x_clk
    }

    pub const fn cpu_2x_clk(&self) -> Hertz {
        self.cpu_2x_clk
    }

    pub const fn cpu_3x2x_clk(&self) -> Hertz {
        self.cpu_3x2x_clk
    }

    pub const fn cpu_6x4x_clk(&self) -> Hertz {
        self.cpu_6x4x_clk
    }
}

#[derive(Debug)]
pub struct DdrClocks {
    ref_clk: Hertz,
    ddr_3x_clk: Hertz,
    ddr_2x_clk: Hertz,
}

impl DdrClocks {
    /// Reference clock provided by DDR PLL which is used to calculate all other clock frequencies.
    pub const fn ref_clk(&self) -> Hertz {
        self.ref_clk
    }

    pub fn ddr_3x_clk(&self) -> Hertz {
        self.ddr_3x_clk
    }

    pub fn ddr_2x_clk(&self) -> Hertz {
        self.ddr_2x_clk
    }
}

#[derive(Debug)]
pub struct IoClocks {
    /// Reference clock provided by IO PLL which is used to calculate all other clock frequencies.
    ref_clk: Hertz,
    smc_clk: Hertz,
    qspi_clk: Hertz,
    sdio_clk: Hertz,
    uart_clk: Hertz,
    spi_clk: Hertz,
    can_clk: Hertz,
    pcap_2x_clk: Hertz,
    trace_clk: Option<Hertz>,
}

impl IoClocks {
    pub const fn ref_clk(&self) -> Hertz {
        self.ref_clk
    }

    pub const fn smc_clk(&self) -> Hertz {
        self.smc_clk
    }

    pub fn update_smc_clk(&mut self, clk: Hertz) {
        self.smc_clk = clk
    }

    pub const fn qspi_clk(&self) -> Hertz {
        self.qspi_clk
    }

    pub fn update_qspi_clk(&mut self, clk: Hertz) {
        self.qspi_clk = clk
    }

    pub const fn sdio_clk(&self) -> Hertz {
        self.sdio_clk
    }

    pub fn update_sdio_clk(&mut self, clk: Hertz) {
        self.sdio_clk = clk
    }

    pub const fn uart_clk(&self) -> Hertz {
        self.uart_clk
    }

    pub fn update_uart_clk(&mut self, clk: Hertz) {
        self.uart_clk = clk
    }

    pub const fn spi_clk(&self) -> Hertz {
        self.spi_clk
    }

    pub fn update_spi_clk(&mut self, clk: Hertz) {
        self.spi_clk = clk
    }

    pub fn can_clk(&self) -> Hertz {
        self.can_clk
    }

    pub fn update_can_clk(&mut self, clk: Hertz) {
        self.can_clk = clk
    }

    pub fn pcap_2x_clk(&self) -> Hertz {
        self.pcap_2x_clk
    }

    pub fn update_pcap_2x_clk(&mut self, clk: Hertz) {
        self.pcap_2x_clk = clk
    }

    /// Returns [None] if the trace clock is configured to use the EMIO trace clock.
    pub fn trace_clk(&self) -> Option<Hertz> {
        self.trace_clk
    }
}

#[derive(Debug)]
pub struct Clocks {
    ps_clk: Hertz,
    arm_pll_out: Hertz,
    io_pll_out: Hertz,
    ddr_pll_out: Hertz,
    arm: ArmClocks,
    ddr: DdrClocks,
    io: IoClocks,
    pl: [Hertz; 4],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ClockModuleId {
    Ddr,
    Arm,
    Smc,
    Qspi,
    Sdio,
    Uart,
    Spi,
    Pcap,
    Can,
    Fpga,
    Trace,
    Gem0,
    Gem1,
}

#[derive(Debug)]
pub struct DivisorZero(pub ClockModuleId);

#[derive(Debug)]
pub enum ClockReadError {
    /// The feedback value for the PLL clock output calculation is zero.
    PllFeedbackZero,
    /// Detected a divisor of zero.
    DivisorZero(DivisorZero),
    /// Detected a divisor that is not even.
    DivisorNotEven,
}

impl Clocks {
    /// Processing system clock, which is generally dependent on the board and the used crystal.
    pub fn ps_clk(&self) -> Hertz {
        self.ps_clk
    }

    /// This generates the clock configuration by reading the SLCR clock registers.
    ///
    /// It assumes that the clock already has been configured, for example by a first-stage
    /// bootloader, or the PS7 initialization script.
    pub fn new_from_regs(ps_clk_freq: Hertz) -> Result<Self, ClockReadError> {
        let mut clk_regs = unsafe { ClockControl::new_mmio_fixed() };

        let arm_pll_cfg = clk_regs.read_arm_pll();
        let io_pll_cfg = clk_regs.read_io_pll();
        let ddr_pll_cfg = clk_regs.read_ddr_pll();

        if arm_pll_cfg.fdiv().as_u32() == 0
            || io_pll_cfg.fdiv().as_u32() == 0
            || ddr_pll_cfg.fdiv().as_u32() == 0
        {
            return Err(ClockReadError::PllFeedbackZero);
        }
        let arm_pll_out = ps_clk_freq * arm_pll_cfg.fdiv().into();
        let io_pll_out = ps_clk_freq * io_pll_cfg.fdiv().into();
        let ddr_pll_out = ps_clk_freq * ddr_pll_cfg.fdiv().into();

        let arm_clk_ctrl = clk_regs.read_arm_clk_ctrl();
        let arm_base_clk = match arm_clk_ctrl.srcsel() {
            zynq7000::slcr::clocks::SrcSelArm::ArmPll
            | zynq7000::slcr::clocks::SrcSelArm::ArmPllAlt => arm_pll_out,
            zynq7000::slcr::clocks::SrcSelArm::DdrPll => ddr_pll_out,
            zynq7000::slcr::clocks::SrcSelArm::IoPll => io_pll_out,
        };
        let clk_sel = clk_regs.read_clk_621_true();
        if arm_clk_ctrl.divisor().as_u32() == 0 {
            return Err(ClockReadError::DivisorZero(DivisorZero(ClockModuleId::Arm)));
        }
        let arm_clk_divided = arm_base_clk / arm_clk_ctrl.divisor().as_u32();
        let arm_clks = match clk_sel.sel() {
            ClockkRatioSelect::FourToTwoToOne => ArmClocks {
                ref_clk: arm_pll_out,
                cpu_1x_clk: arm_clk_divided / 4,
                cpu_2x_clk: arm_clk_divided / 2,
                cpu_3x2x_clk: arm_clk_divided / 2,
                cpu_6x4x_clk: arm_clk_divided,
            },
            ClockkRatioSelect::SixToTwoToOne => ArmClocks {
                ref_clk: arm_pll_out,
                cpu_1x_clk: arm_clk_divided / 6,
                cpu_2x_clk: arm_clk_divided / 3,
                cpu_3x2x_clk: arm_clk_divided / 2,
                cpu_6x4x_clk: arm_clk_divided,
            },
        };

        let ddr_clk_ctrl = clk_regs.read_ddr_clk_ctrl();
        if ddr_clk_ctrl.div_3x_clk().as_u32() == 0 || ddr_clk_ctrl.div_2x_clk().as_u32() == 0 {
            return Err(ClockReadError::DivisorZero(DivisorZero(ClockModuleId::Ddr)));
        }
        let ddr_clks = DdrClocks {
            ref_clk: ddr_pll_out,
            ddr_3x_clk: ddr_pll_out / ddr_clk_ctrl.div_3x_clk().as_u32(),
            ddr_2x_clk: ddr_pll_out / ddr_clk_ctrl.div_2x_clk().as_u32(),
        };

        let handle_common_single_clock_config = |single_block: SingleCommonPeriphIoClockControl,
                                                 id: ClockModuleId|
         -> Result<Hertz, ClockReadError> {
            if single_block.divisor().as_u32() == 0 {
                return Err(ClockReadError::DivisorZero(DivisorZero(id)));
            }
            Ok(match single_block.srcsel() {
                zynq7000::slcr::clocks::SrcSelIo::IoPll
                | zynq7000::slcr::clocks::SrcSelIo::IoPllAlt => {
                    io_pll_out / single_block.divisor().as_u32()
                }
                zynq7000::slcr::clocks::SrcSelIo::ArmPll => {
                    arm_pll_out / single_block.divisor().as_u32()
                }
                zynq7000::slcr::clocks::SrcSelIo::DdrPll => {
                    ddr_pll_out / single_block.divisor().as_u32()
                }
            })
        };
        let handle_common_dual_clock_config = |dual_block: DualCommonPeriphIoClockControl,
                                               id: ClockModuleId|
         -> Result<Hertz, ClockReadError> {
            if dual_block.divisor().as_u32() == 0 {
                return Err(ClockReadError::DivisorZero(DivisorZero(id)));
            }
            Ok(match dual_block.srcsel() {
                zynq7000::slcr::clocks::SrcSelIo::IoPll
                | zynq7000::slcr::clocks::SrcSelIo::IoPllAlt => {
                    io_pll_out / dual_block.divisor().as_u32()
                }
                zynq7000::slcr::clocks::SrcSelIo::ArmPll => {
                    arm_pll_out / dual_block.divisor().as_u32()
                }
                zynq7000::slcr::clocks::SrcSelIo::DdrPll => {
                    ddr_pll_out / dual_block.divisor().as_u32()
                }
            })
        };

        let smc_clk =
            handle_common_single_clock_config(clk_regs.read_smc_clk_ctrl(), ClockModuleId::Smc)?;
        let qspi_clk =
            handle_common_single_clock_config(clk_regs.read_lqspi_clk_ctrl(), ClockModuleId::Qspi)?;
        let sdio_clk =
            handle_common_dual_clock_config(clk_regs.read_sdio_clk_ctrl(), ClockModuleId::Sdio)?;
        let uart_clk =
            handle_common_dual_clock_config(clk_regs.read_uart_clk_ctrl(), ClockModuleId::Uart)?;
        let spi_clk =
            handle_common_dual_clock_config(clk_regs.read_spi_clk_ctrl(), ClockModuleId::Spi)?;
        let pcap_2x_clk =
            handle_common_single_clock_config(clk_regs.read_pcap_clk_ctrl(), ClockModuleId::Pcap)?;
        let can_clk_ctrl = clk_regs.read_can_clk_ctrl();
        let can_clk_ref_clk = match can_clk_ctrl.srcsel() {
            zynq7000::slcr::clocks::SrcSelIo::IoPll
            | zynq7000::slcr::clocks::SrcSelIo::IoPllAlt => io_pll_out,
            zynq7000::slcr::clocks::SrcSelIo::ArmPll => arm_pll_out,
            zynq7000::slcr::clocks::SrcSelIo::DdrPll => ddr_pll_out,
        };
        if can_clk_ctrl.divisor_0().as_u32() == 0 || can_clk_ctrl.divisor_1().as_u32() == 0 {
            return Err(ClockReadError::DivisorZero(DivisorZero(ClockModuleId::Can)));
        }
        let can_clk =
            can_clk_ref_clk / can_clk_ctrl.divisor_0().as_u32() / can_clk_ctrl.divisor_1().as_u32();

        let trace_clk_ctrl = clk_regs.read_dbg_clk_ctrl();
        if trace_clk_ctrl.divisor().as_u32() == 0 {
            return Err(ClockReadError::DivisorZero(DivisorZero(
                ClockModuleId::Trace,
            )));
        }
        let trace_clk = match trace_clk_ctrl.srcsel() {
            zynq7000::slcr::clocks::SrcSelTpiu::IoPll
            | zynq7000::slcr::clocks::SrcSelTpiu::IoPllAlt => {
                Some(io_pll_out / trace_clk_ctrl.divisor().as_u32())
            }
            zynq7000::slcr::clocks::SrcSelTpiu::ArmPll => {
                Some(arm_pll_out / trace_clk_ctrl.divisor().as_u32())
            }
            zynq7000::slcr::clocks::SrcSelTpiu::DdrPll => {
                Some(ddr_pll_out / trace_clk_ctrl.divisor().as_u32())
            }
            zynq7000::slcr::clocks::SrcSelTpiu::EmioTraceClk
            | zynq7000::slcr::clocks::SrcSelTpiu::EmioTraceClkAlt0
            | zynq7000::slcr::clocks::SrcSelTpiu::EmioTraceClkAlt1
            | zynq7000::slcr::clocks::SrcSelTpiu::EmioTraceClkAlt2 => None,
        };
        let calculate_fpga_clk =
            |fpga_clk_ctrl: FpgaClockControl| -> Result<Hertz, ClockReadError> {
                if fpga_clk_ctrl.divisor_0().as_u32() == 0
                    || fpga_clk_ctrl.divisor_1().as_u32() == 0
                {
                    return Err(ClockReadError::DivisorZero(DivisorZero(
                        ClockModuleId::Fpga,
                    )));
                }
                Ok(match fpga_clk_ctrl.srcsel() {
                    zynq7000::slcr::clocks::SrcSelIo::IoPll
                    | zynq7000::slcr::clocks::SrcSelIo::IoPllAlt => {
                        io_pll_out
                            / fpga_clk_ctrl.divisor_0().as_u32()
                            / fpga_clk_ctrl.divisor_1().as_u32()
                    }
                    zynq7000::slcr::clocks::SrcSelIo::ArmPll => {
                        arm_pll_out
                            / fpga_clk_ctrl.divisor_0().as_u32()
                            / fpga_clk_ctrl.divisor_1().as_u32()
                    }
                    zynq7000::slcr::clocks::SrcSelIo::DdrPll => {
                        ddr_pll_out
                            / fpga_clk_ctrl.divisor_0().as_u32()
                            / fpga_clk_ctrl.divisor_1().as_u32()
                    }
                })
            };

        Ok(Self {
            ps_clk: ps_clk_freq,
            io_pll_out,
            ddr_pll_out,
            arm_pll_out,
            arm: arm_clks,
            ddr: ddr_clks,
            io: IoClocks {
                ref_clk: io_pll_out,
                smc_clk,
                qspi_clk,
                sdio_clk,
                uart_clk,
                spi_clk,
                can_clk,
                pcap_2x_clk,
                trace_clk,
            },
            // TODO: There should be a mut and a non-mut getter for an inner block. We only do pure
            // reads with the inner block here.
            pl: [
                calculate_fpga_clk(clk_regs.fpga_0_clk_ctrl().read_ctrl())?,
                calculate_fpga_clk(clk_regs.fpga_1_clk_ctrl().read_ctrl())?,
                calculate_fpga_clk(clk_regs.fpga_2_clk_ctrl().read_ctrl())?,
                calculate_fpga_clk(clk_regs.fpga_3_clk_ctrl().read_ctrl())?,
            ],
        })
    }

    pub fn arm_clocks(&self) -> &ArmClocks {
        &self.arm
    }

    pub fn ddr_clocks(&self) -> &DdrClocks {
        &self.ddr
    }

    pub fn io_clocks(&self) -> &IoClocks {
        &self.io
    }

    pub fn io_clocks_mut(&mut self) -> &mut IoClocks {
        &mut self.io
    }

    /// Programmable Logic (PL) FCLK clocks.
    pub fn pl_clocks(&self) -> &[Hertz; 4] {
        &self.pl
    }

    fn calculate_gem_ref_clock(
        &self,
        reg: GigEthClockControl,
        module: ClockModuleId,
    ) -> Result<Hertz, DivisorZero> {
        let source_clk = match reg.srcsel() {
            zynq7000::slcr::clocks::SrcSelIo::IoPll
            | zynq7000::slcr::clocks::SrcSelIo::IoPllAlt => self.io_pll_out,
            zynq7000::slcr::clocks::SrcSelIo::ArmPll => self.arm_pll_out,
            zynq7000::slcr::clocks::SrcSelIo::DdrPll => self.ddr_pll_out,
        };
        let div0 = reg.divisor_0().as_u32();
        if div0 == 0 {
            return Err(DivisorZero(module));
        }
        let div1 = reg.divisor_1().as_u32();
        if div1 == 0 {
            return Err(DivisorZero(module));
        }
        Ok(source_clk / reg.divisor_0().as_u32() / reg.divisor_1().as_u32())
    }

    /// Calculate the reference clock for GEM0.
    ///
    /// The divisor 1 of the GEM is 0 on reset. You have to properly initialize the clock
    /// configuration before calling this function.
    ///
    /// It should be noted that the GEM has a separate TX and RX clock.
    /// The reference clock will only be the RX clock in loopback mode. For the TX block,
    /// the reference clock is used if the EMIO enable bit `GEM{0,1}_CLK_CTRL[6]` is set to 0.
    pub fn calculate_gem_0_ref_clock(&self) -> Result<Hertz, DivisorZero> {
        let clk_regs = unsafe { ClockControl::new_mmio_fixed() };
        self.calculate_gem_ref_clock(clk_regs.read_gem_0_clk_ctrl(), ClockModuleId::Gem0)
    }

    /// Calculate the reference clock for GEM1.
    ///
    /// The divisor 1 of the GEM is 0 on reset. You have to properly initialize the clock
    /// configuration before calling this function.
    ///
    /// It should be noted that the GEM has a separate TX and RX clock.
    /// The reference clock will only be the RX clock in loopback mode. For the TX block,
    /// the reference clock is used if the EMIO enable bit `GEM{0,1}_CLK_CTRL[6]` is set to 0.
    pub fn calculate_gem_1_ref_clock(&self) -> Result<Hertz, DivisorZero> {
        let clk_regs = unsafe { ClockControl::new_mmio_fixed() };
        self.calculate_gem_ref_clock(clk_regs.read_gem_0_clk_ctrl(), ClockModuleId::Gem1)
    }
}
