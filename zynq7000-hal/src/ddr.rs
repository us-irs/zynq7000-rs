use arbitrary_int::{Number, u2, u3, u4, u5, u6, u7, u10, u11, u12};
use zynq7000::{
    ddrc::{
        DataBusWidth, DdrcControl, DisableDq, DramBurst8ReadWrite, DramEmr, DramEmrMr,
        DramInitParam, DramParamReg0, DramParamReg1, DramParamReg2, DramParamReg3, DramParamReg4,
        LprHprQueueControl, MmioDdrController, MobileSetting, ModeRegisterType, SoftReset,
        TwoRankConfig, WriteQueueControl,
    },
    slcr::{clocks::DciClkCtrl, ddriob::DdriobConfig},
};

use crate::{clocks::DdrClocks, time::Hertz};

const DCI_MAX_FREQ: Hertz = Hertz::from_raw(10_000_000);

// These values were extracted from the ps7_init files and are not documented in the TMR.
// zynq-rs uses the same values.. I assume they are constant.

pub const DRIVE_SLEW_ADDR_CFG: u32 = 0x0018_c61c;
pub const DRIVE_SLEW_DATA_CFG: u32 = 0x00f9_861c;
pub const DRIVE_SLEW_DIFF_CFG: u32 = 0x00f9_861c;
pub const DRIVE_SLEW_CLOCK_CFG: u32 = 0x00f9_861c;

#[derive(Debug, Clone, Copy)]
pub struct DciClkConfig {
    div0: u6,
    div1: u6,
}

pub fn calculate_dci_divisors(ddr_clks: &DdrClocks) -> DciClkConfig {
    calculate_dci_divisors_with_ddr_clk(ddr_clks.ref_clk())
}

pub fn calculate_dci_divisors_with_ddr_clk(ddr_clk: Hertz) -> DciClkConfig {
    let target_div = ddr_clk.raw().div_ceil(DCI_MAX_FREQ.raw());
    let mut config = DciClkConfig {
        div0: u6::new(u6::MAX.value() as u8),
        div1: u6::new(u6::MAX.value() as u8),
    };

    let mut best_error = 0;
    for divisor0 in 1..63 {
        for divisor1 in 1..63 {
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

pub fn configure_dci(ddr_clk: &DdrClocks) {
    let cfg = calculate_dci_divisors(ddr_clk);
    // Safety: Only done once here during start-up.
    unsafe {
        crate::Slcr::with(|slcr| {
            slcr.clk_ctrl().write_dci_clk_ctrl(
                DciClkCtrl::builder()
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
pub fn calibrate_iob_impedance_for_ddr3(dci_clk_cfg: DciClkConfig, poll_for_done: bool) {
    calibrate_iob_impedance(
        dci_clk_cfg,
        u3::new(0),
        u2::new(0),
        u3::new(0b001),
        u3::new(0),
        u2::new(0),
        poll_for_done,
    );
}

/// Calibrates the IOB impedance according to to TRM p.325, DDR IOB Impedance calibration.
///
/// This function will also enable the DCI clock with the provided clock configuration.
/// You can use [calculate_dci_divisors] to calculate the divisor values for the given DDR clock,
/// or you can hardcode the values if they are fixed.
pub fn calibrate_iob_impedance(
    dci_clk_cfg: DciClkConfig,
    pref_opt2: u3,
    pref_opt1: u2,
    nref_opt4: u3,
    nref_opt2: u3,
    nref_opt1: u2,
    poll_for_done: bool,
) {
    let mut slcr = unsafe { crate::slcr::Slcr::steal() };
    slcr.modify(|slcr| {
        slcr.clk_ctrl().write_dci_clk_ctrl(
            DciClkCtrl::builder()
                .with_divisor_1(dci_clk_cfg.div1)
                .with_divisor_0(dci_clk_cfg.div0)
                .with_clk_act(true)
                .build(),
        );
        let mut ddriob = slcr.ddriob();
        ddriob.modify_dci_ctrl(|mut val| {
            val.set_reset(true);
            val
        });
        ddriob.modify_dci_ctrl(|mut val| {
            val.set_reset(false);
            val
        });
        ddriob.modify_dci_ctrl(|mut val| {
            val.set_reset(true);
            val
        });
        ddriob.modify_dci_ctrl(|mut val| {
            val.set_pref_opt2(pref_opt2);
            val.set_pref_opt1(pref_opt1);
            val.set_nref_opt4(nref_opt4);
            val.set_nref_opt2(nref_opt2);
            val.set_nref_opt1(nref_opt1);
            val
        });
        ddriob.modify_dci_ctrl(|mut val| {
            val.set_update_control(false);
            val
        });
        ddriob.modify_dci_ctrl(|mut val| {
            val.set_enable(true);
            val
        });
        if poll_for_done {
            while !slcr.ddriob().read_dci_status().done() {
                // Wait for the DDR IOB impedance calibration to complete.
                cortex_ar::asm::nop();
            }
        }
    });
}

pub struct DdriobConfigSet {
    pub addr0: DdriobConfig,
    pub addr1: DdriobConfig,
    pub data0: DdriobConfig,
    pub data1: DdriobConfig,
    pub diff0: DdriobConfig,
    pub diff1: DdriobConfig,
    pub clock: DdriobConfig,
}

pub fn configure_iob(cfg_set: &DdriobConfigSet) {
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

pub fn configure_ddr(mut ddrc: MmioDdrController<'static>) {
    // Lock the DDR.
    ddrc.write_ddrc_ctrl(
        DdrcControl::builder()
            .with_disable_auto_refresh(false)
            .with_disable_active_bypass(false)
            .with_disable_read_bypass(false)
            .with_read_write_idle_gap(u7::new(1))
            .with_burst8_refresh(u3::new(0))
            .with_data_bus_width(DataBusWidth::_32Bit)
            .with_power_down_enable(false)
            .with_soft_reset(SoftReset::Reset)
            .build(),
    );
    ddrc.write_two_rank_cfg(
        TwoRankConfig::builder()
            .with_addrmap_cs_bit0(u5::new(0))
            .with_ddrc_active_ranks(u2::new(1))
            .with_rfc_nom_x32(u12::new(0x82))
            .build(),
    );
    ddrc.write_hpr_queue_ctrl(
        LprHprQueueControl::builder()
            .with_xact_run_length(u4::new(0xf))
            .with_max_starve_x32(u11::new(0xf))
            .with_min_non_critical_x32(u11::new(0xf))
            .build(),
    );
    ddrc.write_lpr_queue_ctrl(
        LprHprQueueControl::builder()
            .with_xact_run_length(u4::new(0x8))
            .with_max_starve_x32(u11::new(0x2))
            .with_min_non_critical_x32(u11::new(0x1))
            .build(),
    );
    ddrc.write_wr_reg(
        WriteQueueControl::builder()
            .with_max_starve_x32(u11::new(0x2))
            .with_xact_run_length(u4::new(0x8))
            .with_min_non_critical_x32(u11::new(0x1))
            .build(),
    );
    ddrc.write_dram_param_reg0(
        DramParamReg0::builder()
            .with_post_selfref_gap_x32(u7::new(0x10))
            .with_t_rfc_min(0x56)
            .with_t_rc(u6::new(0x1b))
            .build(),
    );
    ddrc.write_dram_param_reg0(
        DramParamReg0::builder()
            .with_post_selfref_gap_x32(u7::new(0x10))
            .with_t_rfc_min(0x56)
            .with_t_rc(u6::new(0x1b))
            .build(),
    );
    ddrc.write_dram_param_reg1(
        DramParamReg1::builder()
            .with_t_cke(u4::new(0x4))
            .with_t_ras_min(u5::new(0x13))
            .with_t_ras_max(u6::new(0x24))
            .with_t_faw(u6::new(0x16))
            .with_powerdown_to_x32(u5::new(0x6))
            .with_wr2pre(u5::new(0x13))
            .build(),
    );
    ddrc.write_dram_param_reg2(
        DramParamReg2::builder()
            .with_t_rcd(u4::new(0x7))
            .with_rd2pre(u5::new(0x5))
            .with_pad_pd(u3::new(0x0))
            .with_t_xp(u5::new(0x5))
            .with_wr2rd(u5::new(0xf))
            .with_rd2wr(u5::new(0x7))
            .with_write_latency(u5::new(0x5))
            .build(),
    );
    ddrc.write_dram_param_reg3(
        DramParamReg3::builder()
            .with_disable_pad_pd_feature(false)
            .with_read_latency(u5::new(0x7))
            .with_enable_dfi_dram_clk_disable(false)
            .with_mobile(MobileSetting::Ddr2Ddr3)
            .with_sdram(false)
            .with_refresh_to_x32(u5::new(0x8))
            .with_t_rp(u4::new(0x7))
            .with_refresh_margin(u4::new(0x2))
            .with_t_rrd(u3::new(0x6))
            .with_t_ccd(u3::new(0x4))
            .build(),
    );
    ddrc.write_dram_param_reg4(
        DramParamReg4::builder()
            .with_mr_rdata_valid(false)
            .with_mr_type(ModeRegisterType::Write)
            .with_mr_wr_busy(false)
            .with_mr_data(u16::new(0x0))
            .with_mr_addr(u2::new(0x0))
            .with_mr_wr(false)
            .with_prefer_write(false)
            .with_enable_2t_timing_mode(false)
            .build(),
    );
    ddrc.write_dram_init_param(
        DramInitParam::builder()
            .with_t_mrd(u3::new(0x4))
            .with_pre_ocd_x32(u4::new(0x0))
            .with_final_wait_x32(u7::new(0x7))
            .build(),
    );
    ddrc.write_dram_emr(DramEmr::builder().with_emr3(0x0).with_emr2(0x8).build());
    ddrc.write_dram_emr_mr(DramEmrMr::builder().with_emr(0x4).with_mr(0xb30).build());
    ddrc.write_dram_burst8_rdwr(
        DramBurst8ReadWrite::builder()
            .with_burst_rdwr(u4::new(0x4))
            .with_pre_cke_x1024(u10::new(0x16d))
            .with_post_cke_x1024(u10::new(0x1))
            .with_burstchop(false)
            .build(),
    );
    ddrc.write_dram_disable_dq(
        DisableDq::builder()
            .with_dis_dq(false)
            .with_force_low_pri_n(false)
            .build(),
    );
}
