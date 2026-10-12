//! Low-level DDR configuration module.
use arbitrary_int::{prelude::*, u2, u3, u6};
use zynq7000::ddrc::{MmioRegisters, types::*};
use zynq7000::slcr::{clocks::DciClockControl, ddriob::DdriobConfig};

use crate::{clocks::DdrClocks, time::Hertz};

const DCI_MAX_FREQ: Hertz = Hertz::from_raw(10_000_000);

// These values were extracted from the ps7_init files and are not documented in the TMR.
// zynq-rs uses the same values.. I assume they are constant.

/// Drive and slew configuration of the address IOBs.
pub const DRIVE_SLEW_ADDR_CFG: u32 = 0x0018_c61c;
/// Drive and slew configuration of the data IOBs.
pub const DRIVE_SLEW_DATA_CFG: u32 = 0x00f9_861c;
/// Drive and slew configuration of the differential IOBs.
pub const DRIVE_SLEW_DIFF_CFG: u32 = 0x00f9_861c;
/// Drive and slew configuration of the clock IOBs.
pub const DRIVE_SLEW_CLOCK_CFG: u32 = 0x00f9_861c;

/// DCI clock divisors.
#[derive(Debug, Clone, Copy)]
pub struct DciClkConfig {
    div0: u6,
    div1: u6,
}

/// Calculate the required DCI divisors for the given DDR clock.
pub fn calculate_dci_divisors(ddr_clks: &DdrClocks) -> DciClkConfig {
    calculate_dci_divisors_with_ddr_clk(ddr_clks.ref_clk())
}

/// Calculate the required DCI divisors for the given DDR clock frequency.
pub fn calculate_dci_divisors_with_ddr_clk(ddr_clk: Hertz) -> DciClkConfig {
    let target_div = ddr_clk.to_raw().div_ceil(DCI_MAX_FREQ.to_raw());
    let mut config = DciClkConfig {
        div0: u6::new(u6::MAX.value()),
        div1: u6::new(u6::MAX.value()),
    };

    let mut best_error = u32::MAX;
    for divisor0 in 1..=63 {
        for divisor1 in 1..=63 {
            let current_div = (divisor0 as u32) * (divisor1 as u32);
            let error = current_div.abs_diff(target_div);
            if error < best_error {
                config.div0 = u6::new(divisor0 as u8);
                config.div1 = u6::new(divisor1 as u8);
                best_error = error;
            }
        }
    }
    config
}

/// Configure the DCI module by configure its clock divisors and enabling it.
///
/// # Safety
///
/// This function writes to DCI related registers. It should only be called once during
/// DDR initialization.
pub unsafe fn configure_dci(ddr_clk: &DdrClocks) {
    let cfg = calculate_dci_divisors(ddr_clk);
    // Safety: Only writes to DCI clock related registers.
    unsafe {
        crate::Slcr::with(|slcr| {
            slcr.clk_ctrl().write_dci_clk_ctrl(
                DciClockControl::builder()
                    .with_divisor_1(cfg.div1)
                    .with_divisor_0(cfg.div0)
                    .with_clk_act(true)
                    .build(),
            );
        });
    }
}

/// Calibrates the IOB impedance for DDR3 memory according to to TRM p.325, DDR IOB Impedance
/// calibration.
///
/// This function will also enable the DCI clock with the provided clock configuration.
/// You can use [calculate_dci_divisors] to calculate the divisor values for the given DDR clock,
/// or you can hardcode the values if they are fixed.
///
/// Polling for completion can be disabled. The calibration takes 1-2 ms according to the TRM, so
/// the user can set other configuration values which are not reliant on DDR operation before
/// polling for completion.
///
/// # Safety
///
/// This function writes to the DDR IOB related registers. It should only be called once during
/// DDR initialization.
pub unsafe fn calibrate_iob_impedance_for_ddr3(
    ddr_control: zynq7000::slcr::ddriob::DdrControl,
    dci_clk_cfg: DciClkConfig,
    poll_for_done: bool,
) {
    // SAFETY: Upheld by the caller.
    unsafe {
        calibrate_iob_impedance(
            ddr_control,
            dci_clk_cfg,
            CalibrationParams::new_ddr3(),
            poll_for_done,
        );
    }
}

/// DDR IOB impedance calibration parameters. The fields have the names of the `DDRIOB_DCI_CTRL`
/// register fields.
pub struct CalibrationParams {
    /// `PREF_OPT2` field.
    pub pref_opt2: u3,
    /// `PREF_OPT1` field.
    pub pref_opt1: u2,
    /// `NREF_OPT4` field.
    pub nref_opt4: u3,
    /// `NREF_OPT2` field.
    pub nref_opt2: u3,
    /// `NREF_OPT1` field.
    pub nref_opt1: u2,
}

impl CalibrationParams {
    /// Calibration parameters for DDR3.
    pub const fn new_ddr3() -> Self {
        Self {
            pref_opt2: u3::new(0),
            pref_opt1: u2::new(0),
            nref_opt4: u3::new(0b001),
            nref_opt2: u3::new(0),
            nref_opt1: u2::new(0),
        }
    }
}

/// Calibrates the IOB impedance according to to TRM p.325, DDR IOB Impedance calibration.
///
/// This function will also enable the DCI clock with the provided clock configuration.
/// You can use [calculate_dci_divisors] to calculate the divisor values for the given DDR clock,
/// or you can hardcode the values if they are fixed.
///
/// Polling for completion can be disabled. The calibration takes 1-2 ms according to the TRM, so
/// the user can set other configuration values which are not reliant on DDR operation before
/// polling for completion.
///
/// # Safety
///
/// This function writes to the DDR IOB related registers. It should only be called once during
/// DDR initialization.
pub unsafe fn calibrate_iob_impedance(
    ddr_control: zynq7000::slcr::ddriob::DdrControl,
    dci_clk_cfg: DciClkConfig,
    calibration_params: CalibrationParams,
    poll_for_done: bool,
) {
    // Safety: Only writes to DDR IOB related registers.
    let mut slcr = unsafe { crate::slcr::Slcr::steal() };
    slcr.modify(|slcr| {
        slcr.clk_ctrl().write_dci_clk_ctrl(
            DciClockControl::builder()
                .with_divisor_1(dci_clk_cfg.div1)
                .with_divisor_0(dci_clk_cfg.div0)
                .with_clk_act(true)
                .build(),
        );
        let mut ddriob = slcr.ddriob();
        ddriob.write_ddr_control(ddr_control);
        ddriob.modify_dci_control(|val| val.with_reset(true));
        ddriob.modify_dci_control(|val| val.with_reset(false));
        ddriob.modify_dci_control(|val| val.with_reset(true));
        ddriob.modify_dci_control(|mut val| {
            val.set_pref_opt2(calibration_params.pref_opt2);
            val.set_pref_opt1(calibration_params.pref_opt1);
            val.set_nref_opt4(calibration_params.nref_opt4);
            val.set_nref_opt2(calibration_params.nref_opt2);
            val.set_nref_opt1(calibration_params.nref_opt1);
            val
        });
        ddriob.modify_dci_control(|mut val| {
            val.set_update_control(false);
            val
        });
        ddriob.modify_dci_control(|mut val| {
            val.set_enable(true);
            val
        });
    });
    // Reads work while the SLCR is locked, so the poll does not block interrupts.
    if poll_for_done {
        while !slcr.regs().ddriob_shared().read_dci_status().done() {}
    }
}

/// Static configuration for DDR IOBs. Each field is the value of the register with the same name.
pub struct DdriobConfigSet {
    /// `DDRIOB_DDR_CTRL` register.
    pub ddr_control: zynq7000::slcr::ddriob::DdrControl,
    /// `DDRIOB_ADDR0` register.
    pub addr0: DdriobConfig,
    /// `DDRIOB_ADDR1` register.
    pub addr1: DdriobConfig,
    /// `DDRIOB_DATA0` register.
    pub data0: DdriobConfig,
    /// `DDRIOB_DATA1` register.
    pub data1: DdriobConfig,
    /// `DDRIOB_DIFF0` register.
    pub diff0: DdriobConfig,
    /// `DDRIOB_DIFF1` register.
    pub diff1: DdriobConfig,
    /// `DDRIOB_CLOCK` register.
    pub clock: DdriobConfig,
}

/// # Safety
///
/// This function writes to the IOB related registers. It should only be called once during
/// DDR initialization.
pub unsafe fn configure_iob(cfg_set: &DdriobConfigSet) {
    // Safety: Only configures IOB related registers.
    let mut slcr = unsafe { crate::slcr::Slcr::steal() };
    slcr.modify(|slcr| {
        let mut ddriob = slcr.ddriob();
        ddriob.write_ddriob_addr0(cfg_set.addr0);
        ddriob.write_ddriob_addr1(cfg_set.addr1);

        ddriob.write_ddriob_data0(cfg_set.data0);
        ddriob.write_ddriob_data1(cfg_set.data1);

        ddriob.write_ddriob_diff0(cfg_set.diff0);
        ddriob.write_ddriob_diff1(cfg_set.diff1);

        ddriob.write_ddriob_clock(cfg_set.clock);

        // These values were extracted from the ps7_init files and are not documented in the TRM.
        // zynq-rs uses the same values.. I assume they are constant.
        ddriob.write_ddriob_drive_slew_addr(DRIVE_SLEW_ADDR_CFG);
        ddriob.write_ddriob_drive_slew_data(DRIVE_SLEW_DATA_CFG);
        ddriob.write_ddriob_drive_slew_diff(DRIVE_SLEW_DIFF_CFG);
        ddriob.write_ddriob_drive_slew_clock(DRIVE_SLEW_CLOCK_CFG);
    });
}

/// Full static DDRC configuration set. Each field is the value of the register with the same name.
#[derive(Debug)]
pub struct DdrcConfigSet {
    /// `ddrc_ctrl` register.
    pub ctrl: DdrcControl,
    /// `Two_rank_cfg` register.
    pub two_rank: TwoRankConfig,
    /// `HPR_reg` register.
    pub hpr: LprHprQueueControl,
    /// `LPR_reg` register.
    pub lpr: LprHprQueueControl,
    /// `WR_reg` register.
    pub wr: WriteQueueControl,
    /// `DRAM_param_reg0` register.
    pub dram_param_0: DramParamReg0,
    /// `DRAM_param_reg1` register.
    pub dram_param_1: DramParamReg1,
    /// `DRAM_param_reg2` register.
    pub dram_param_2: DramParamReg2,
    /// `DRAM_param_reg3` register.
    pub dram_param_3: DramParamReg3,
    /// `DRAM_param_reg4` register.
    pub dram_param_4: DramParamReg4,
    /// `DRAM_init_param` register.
    pub dram_init_param: DramInitParam,
    /// `DRAM_EMR_reg` register.
    pub dram_emr: DramEmr,
    /// `DRAM_EMR_MR_reg` register.
    pub dram_emr_mr: DramEmrMr,
    /// `DRAM_burst8_rdwr` register.
    pub dram_burst8_rdwr: DramBurst8ReadWrite,
    /// `DRAM_disable_DQ` register.
    pub disable_dq: DisableDq,
    /// `DRAM_addr_map_bank` register.
    pub dram_addr_map_bank: DramAddrMapBank,
    /// `DRAM_addr_map_col` register.
    pub dram_addr_map_col: DramAddrMapColumn,
    /// `DRAM_addr_map_row` register.
    pub dram_addr_map_row: DramAddrMapRow,
    /// `DRAM_ODT_reg` register.
    pub dram_odt: DramOdt,
    /// `phy_cmd_timeout_rddata_cpt` register.
    pub phy_cmd_timeout_rddata_cpt: PhyCmdTimeoutRdDataCpt,
    /// `DLL_calib` register.
    pub dll_calib: DllCalib,
    /// `ODT_delay_hold` register.
    pub odt_delay_hold: OdtDelayHold,
    /// `ctrl_reg1` register.
    pub ctrl_reg1: CtrlReg1,
    /// `ctrl_reg2` register.
    pub ctrl_reg2: CtrlReg2,
    /// `ctrl_reg3` register.
    pub ctrl_reg3: CtrlReg3,
    /// `ctrl_reg4` register.
    pub ctrl_reg4: CtrlReg4,
    /// `ctrl_reg5` register.
    pub ctrl_reg5: CtrlReg5,
    /// `ctrl_reg6` register.
    pub ctrl_reg6: CtrlReg6,
    /// `CHE_T_ZQ` register.
    pub che_t_zq: CheTZq,
    /// `CHE_T_ZQ_Short_Interval_Reg` register.
    pub che_t_zq_short_interval_reg: CheTZqShortInterval,
    /// `deep_pwrdwn_reg` register.
    pub deep_powerdown: DeepPowerdown,
    /// `reg_2c` register.
    pub reg_2c: Reg2c,
    /// `reg_2d` register.
    pub reg_2d: Reg2d,
    /// `dfi_timing` register.
    pub dfi_timing: DfiTiming,
    /// `CHE_ECC_CONTROL_REG_OFFSET` register.
    pub che_ecc_ctrl: CheEccControl,
    /// `ECC_scrub` register.
    pub ecc_scrub: EccScrub,
    /// `phy_rcvr_enable` register.
    pub phy_receiver_enable: PhyReceiverEnable,
    /// `PHY_Config0` to `PHY_Config3` registers.
    pub phy_config: [PhyConfig; 4],
    /// `phy_init_ratio0` to `phy_init_ratio3` registers.
    pub phy_init_ratio: [PhyInitRatio; 4],
    /// `phy_rd_dqs_cfg0` to `phy_rd_dqs_cfg3` registers.
    pub phy_rd_dqs_config: [PhyDqsConfig; 4],
    /// `phy_wr_dqs_cfg0` to `phy_wr_dqs_cfg3` registers.
    pub phy_wr_dqs_config: [PhyDqsConfig; 4],
    /// `phy_we_cfg0` to `phy_we_cfg3` registers.
    pub phy_we_cfg: [PhyWriteEnableConfig; 4],
    /// `wr_data_slv0` to `wr_data_slv3` registers.
    pub phy_wr_data_slv: [PhyWriteDataSlaveConfig; 4],
    /// `reg_64` register.
    pub reg64: Reg64,
    /// `reg_65` register.
    pub reg65: Reg65,
    /// `page_mask` register.
    pub page_mask: u32,
    /// `axi_priority_wr_port0` to `axi_priority_wr_port3` registers.
    pub axi_priority_wr_port: [AxiPriorityWritePort; 4],
    /// `axi_priority_rd_port0` to `axi_priority_rd_port3` registers.
    pub axi_priority_rd_port: [AxiPriorityReadPort; 4],
    /// `lpddr_ctrl0` register.
    pub lpddr_ctrl_0: LpddrControl0,
    /// `lpddr_ctrl1` register.
    pub lpddr_ctrl_1: LpddrControl1,
    /// `lpddr_ctrl2` register.
    pub lpddr_ctrl_2: LpddrControl2,
    /// `lpddr_ctrl3` register.
    pub lpddr_ctrl_3: LpddrControl3,
}

/// This low-level function sets all the configuration registers.
///
/// It does NOT take care of taking the DDR controller out of reset and polling for DDR
/// configuration completion.
pub fn configure_ddr_config(ddrc: &mut MmioRegisters<'static>, cfg_set: &DdrcConfigSet) {
    ddrc.write_ddrc_ctrl(cfg_set.ctrl);
    // Write all configuration registers.
    ddrc.write_two_rank_cfg(cfg_set.two_rank);
    ddrc.write_hpr_queue_ctrl(cfg_set.hpr);
    ddrc.write_lpr_queue_ctrl(cfg_set.lpr);
    ddrc.write_wr_reg(cfg_set.wr);
    ddrc.write_dram_param_reg0(cfg_set.dram_param_0);
    ddrc.write_dram_param_reg1(cfg_set.dram_param_1);
    ddrc.write_dram_param_reg2(cfg_set.dram_param_2);
    ddrc.write_dram_param_reg3(cfg_set.dram_param_3);
    ddrc.write_dram_param_reg4(cfg_set.dram_param_4);
    ddrc.write_dram_init_param(cfg_set.dram_init_param);
    ddrc.write_dram_emr(cfg_set.dram_emr);
    ddrc.write_dram_emr_mr(cfg_set.dram_emr_mr);
    ddrc.write_dram_burst8_rdwr(cfg_set.dram_burst8_rdwr);
    ddrc.write_dram_disable_dq(cfg_set.disable_dq);
    ddrc.write_phy_cmd_timeout_rddata_cpt(cfg_set.phy_cmd_timeout_rddata_cpt);
    ddrc.write_dll_calib(cfg_set.dll_calib);
    ddrc.write_odt_delay_hold(cfg_set.odt_delay_hold);
    ddrc.write_ctrl_reg1(cfg_set.ctrl_reg1);
    ddrc.write_ctrl_reg2(cfg_set.ctrl_reg2);
    ddrc.write_ctrl_reg3(cfg_set.ctrl_reg3);
    ddrc.write_ctrl_reg4(cfg_set.ctrl_reg4);
    ddrc.write_ctrl_reg5(cfg_set.ctrl_reg5);
    ddrc.write_ctrl_reg6(cfg_set.ctrl_reg6);
    ddrc.write_che_t_zq(cfg_set.che_t_zq);
    ddrc.write_che_t_zq_short_interval_reg(cfg_set.che_t_zq_short_interval_reg);
    ddrc.write_deep_powerdown_reg(cfg_set.deep_powerdown);
    ddrc.write_reg_2c(cfg_set.reg_2c);
    ddrc.write_reg_2d(cfg_set.reg_2d);
    ddrc.write_dfi_timing(cfg_set.dfi_timing);
    ddrc.write_che_ecc_control(cfg_set.che_ecc_ctrl);
    ddrc.write_ecc_scrub(cfg_set.ecc_scrub);
    ddrc.write_phy_receiver_enable(cfg_set.phy_receiver_enable);
    for i in 0..4 {
        // Safety: Indexes are valid.
        unsafe {
            ddrc.write_phy_config_unchecked(i, cfg_set.phy_config[i]);
            ddrc.write_phy_init_ratio_unchecked(i, cfg_set.phy_init_ratio[i]);
            ddrc.write_phy_rd_dqs_cfg_unchecked(i, cfg_set.phy_rd_dqs_config[i]);
            ddrc.write_phy_wr_dqs_cfg_unchecked(i, cfg_set.phy_wr_dqs_config[i]);
            ddrc.write_phy_we_cfg_unchecked(i, cfg_set.phy_we_cfg[i]);
            ddrc.write_phy_wr_data_slave_unchecked(i, cfg_set.phy_wr_data_slv[i]);
        }
    }
    ddrc.write_reg_64(cfg_set.reg64);
    ddrc.write_reg_65(cfg_set.reg65);
    ddrc.write_page_mask(cfg_set.page_mask);
    for i in 0..4 {
        // Safety: Indexes are valid.
        unsafe {
            ddrc.write_axi_priority_wr_port_unchecked(i, cfg_set.axi_priority_wr_port[i]);
            ddrc.write_axi_priority_rd_port_unchecked(i, cfg_set.axi_priority_rd_port[i]);
        }
    }
    ddrc.write_lpddr_ctrl_0(cfg_set.lpddr_ctrl_0);
    ddrc.write_lpddr_ctrl_1(cfg_set.lpddr_ctrl_1);
    ddrc.write_lpddr_ctrl_2(cfg_set.lpddr_ctrl_2);
    ddrc.write_lpddr_ctrl_3(cfg_set.lpddr_ctrl_3);
}
