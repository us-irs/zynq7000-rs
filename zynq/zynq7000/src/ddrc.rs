pub const DDRC_BASE_ADDR: usize = 0xF800_6000;

pub mod regs {
    pub use crate::slcr::ddriob::DdriobConfig;
    use arbitrary_int::{u2, u3, u4, u5, u6, u7, u9, u10, u11, u12, u20};

    #[bitbybit::bitenum(u2)]
    #[derive(Debug, PartialEq, Eq)]
    pub enum DataBusWidth {
        _32Bit = 0b00,
        _16Bit = 0b01,
    }

    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    pub enum SoftReset {
        Reset = 0,
        Active = 1,
    }
    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DdrcControl {
        #[bit(16, rw)]
        disable_auto_refresh: bool,
        #[bit(15, rw)]
        disable_active_bypass: bool,
        #[bit(14, rw)]
        disable_read_bypass: bool,
        #[bits(7..=13, rw)]
        read_write_idle_gap: u7,
        #[bits(4..=6, rw)]
        burst8_refresh: u3,
        #[bits(2..=3, rw)]
        data_bus_width: Option<DataBusWidth>,
        #[bit(1, rw)]
        power_down_enable: bool,
        #[bit(0, rw)]
        soft_reset: SoftReset,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct TwoRankConfig {
        #[bits(14..=18, rw)]
        addrmap_cs_bit0: u5,
        /// Reserved register, but for some reason, Xilinx tooling writes a 1 here?
        #[bits(12..=13, rw)]
        ddrc_active_ranks: u2,
        /// tREFI - Average time between refreshes, in multiples of 32 clocks.
        #[bits(0..=11, rw)]
        rfc_nom_x32: u12,
    }

    /// Queue control for the low priority and high priority read queues.
    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct LprHprQueueControl {
        #[bits(22..=25, rw)]
        xact_run_length: u4,
        #[bits(11..=21, rw)]
        max_starve_x32: u11,
        #[bits(0..=10, rw)]
        min_non_critical_x32: u11,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct WriteQueueControl {
        #[bits(15..=25, rw)]
        max_starve_x32: u11,
        #[bits(11..=14, rw)]
        xact_run_length: u4,
        #[bits(0..=10, rw)]
        min_non_critical_x32: u11,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramParamReg0 {
        /// Minimum time to wait after coming out of self refresh before doing anything. This must be
        /// bigger than all the constraints that exist.
        #[bits(14..=20, rw)]
        post_selfref_gap_x32: u7,
        /// tRFC(min) - Minimum time from refresh to refresh or activate in clock
        /// cycles.
        #[bits(6..=13, rw)]
        t_rfc_min: u8,
        /// tRC - Min time between activates to the same bank.
        #[bits(0..=5, rw)]
        t_rc: u6,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramParamReg1 {
        #[bits(28..=31, rw)]
        t_cke: u4,
        #[bits(22..=26, rw)]
        t_ras_min: u5,
        #[bits(16..=21, rw)]
        t_ras_max: u6,
        #[bits(10..=15, rw)]
        t_faw: u6,
        #[bits(5..=9, rw)]
        powerdown_to_x32: u5,
        #[bits(0..=4, rw)]
        wr2pre: u5,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramParamReg2 {
        #[bits(28..=31, rw)]
        t_rcd: u4,
        #[bits(23..=27, rw)]
        rd2pre: u5,
        #[bits(20..=22, rw)]
        pad_pd: u3,
        #[bits(15..=19, rw)]
        t_xp: u5,
        #[bits(10..=14, rw)]
        wr2rd: u5,
        #[bits(5..=9, rw)]
        rd2wr: u5,
        #[bits(0..=4, rw)]
        write_latency: u5,
    }

    /// Weird naming.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug)]
    pub enum MobileSetting {
        Ddr2Ddr3 = 0,
        Lpddr2 = 1,
    }
    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramParamReg3 {
        #[bit(30, rw)]
        disable_pad_pd_feature: bool,
        #[bits(24..=28, rw)]
        read_latency: u5,
        #[bit(23, rw)]
        enable_dfi_dram_clk_disable: bool,
        /// 0: DDR2 or DDR3. 1: LPDDR2.
        #[bit(22, rw)]
        mobile: MobileSetting,
        /// Must be set to 0.
        #[bit(21, rw)]
        sdram: bool,
        #[bits(16..=20, rw)]
        refresh_to_x32: u5,
        #[bits(12..=15, rw)]
        t_rp: u4,
        #[bits(8..=11, rw)]
        refresh_margin: u4,
        #[bits(5..=7, rw)]
        t_rrd: u3,
        #[bits(2..=4, rw)]
        t_ccd: u3,
    }

    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug)]
    pub enum ModeRegisterType {
        Write = 0,
        Read = 1,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramParamReg4 {
        #[bit(27, rw)]
        mr_rdata_valid: bool,
        #[bit(26, rw)]
        mr_type: ModeRegisterType,
        #[bit(25, rw)]
        mr_wr_busy: bool,
        #[bits(9..=24, rw)]
        mr_data: u16,
        #[bits(7..=8, rw)]
        mr_addr: u2,
        #[bit(6, rw)]
        mr_wr: bool,
        #[bit(1, rw)]
        prefer_write: bool,
        #[bit(0, rw)]
        enable_2t_timing_mode: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramInitParam {
        #[bits(11..=13, rw)]
        t_mrd: u3,
        #[bits(7..=10, rw)]
        pre_ocd_x32: u4,
        #[bits(0..=6, rw)]
        final_wait_x32: u7,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramEmr {
        #[bits(16..=31, rw)]
        emr3: u16,
        #[bits(0..=15, rw)]
        emr2: u16,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramEmrMr {
        #[bits(16..=31, rw)]
        emr: u16,
        #[bits(0..=15, rw)]
        mr: u16,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramBurst8ReadWrite {
        #[bits(0..=3, rw)]
        burst_rdwr: u4,
        #[bits(4..=13, rw)]
        pre_cke_x1024: u10,
        #[bits(16..=25, rw)]
        post_cke_x1024: u10,
        #[bit(26, rw)]
        burstchop: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DisableDq {
        #[bit(1, rw)]
        dis_dq: bool,
        #[bit(0, rw)]
        force_low_pri_n: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramAddrMapBank {
        #[bits(16..=19, rw)]
        addrmap_bank_b6: u4,
        #[bits(12..=15, rw)]
        addrmap_bank_b5: u4,
        #[bits(8..=11, rw)]
        addrmap_bank_b2: u4,
        #[bits(4..=7, rw)]
        addrmap_bank_b1: u4,
        #[bits(0..=3, rw)]
        addrmap_bank_b0: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramAddrMapColumn {
        #[bits(28..=31, rw)]
        addrmap_col_b11: u4,
        #[bits(24..=27, rw)]
        addrmap_col_b10: u4,
        #[bits(20..=23, rw)]
        addrmap_col_b9: u4,
        #[bits(16..=19, rw)]
        addrmap_col_b8: u4,
        #[bits(12..=15, rw)]
        addrmap_col_b7: u4,
        #[bits(8..=11, rw)]
        addrmap_col_b4: u4,
        #[bits(4..=7, rw)]
        addrmap_col_b3: u4,
        #[bits(0..=3, rw)]
        addrmap_col_b2: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramAddrMapRow {
        #[bits(24..=27, rw)]
        addrmap_row_b15: u4,
        #[bits(20..=23, rw)]
        addrmap_row_b14: u4,
        #[bits(16..=19, rw)]
        addrmap_row_b13: u4,
        #[bits(12..=15, rw)]
        addrmap_row_b12: u4,
        #[bits(8..=11, rw)]
        addrmap_row_b2_11: u4,
        #[bits(4..=7, rw)]
        addrmap_row_b1: u4,
        #[bits(0..=3, rw)]
        addrmap_row_b0: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DramOdt {
        #[bits(16..=17, rw)]
        phy_idle_local_odt: u2,
        #[bits(14..=15, rw)]
        phy_write_local_odt: u2,
        #[bits(12..=13, rw)]
        phy_read_local_odt: u2,
        #[bits(3..=5, rw)]
        rank0_wr_odt: u3,
        #[bits(0..=2, rw)]
        rank0_rd_odt: u3,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct PhyCmdTimeoutRdDataCpt {
        #[bits(28..=31, rw)]
        wrlvl_num_of_dq0: u4,
        #[bits(24..=27, rw)]
        gatelvl_num_of_dq0: u4,
        #[bit(19, rw)]
        clk_stall_level: bool,
        #[bit(18, rw)]
        dis_phy_ctrl_rstn: bool,
        #[bit(17, rw)]
        rdc_fifo_rst_err_cnt_clr: bool,
        #[bit(16, rw)]
        use_fixed_re: bool,
        #[bits(8..=11, rw)]
        rdc_we_to_re_delay: u4,
        #[bits(4..=7, rw)]
        wr_cmd_to_data: u4,
        #[bits(0..=3, rw)]
        rd_cmd_to_data: u4,
    }

    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug)]
    pub enum DllCalibSel {
        Periodic = 0,
        Manual = 1,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DllCalib {
        #[bit(16, rw)]
        sel: DllCalibSel,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct OdtDelayHold {
        #[bits(12..=15, rw)]
        wr_odt_hold: u4,
        #[bits(8..=11, rw)]
        rd_odt_hold: u4,
        #[bits(4..=7, rw)]
        wr_odt_delay: u4,
        #[bits(0..=3, rw)]
        rd_odt_delay: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CtrlReg1 {
        #[bit(12, rw)]
        selfref_enable: bool,
        #[bit(10, rw)]
        dis_collision_page_opt: bool,
        #[bit(9, rw)]
        dis_wc: bool,
        #[bit(8, rw)]
        refresh_update_level: bool,
        #[bit(7, rw)]
        auto_pre_en: bool,
        #[bits(1..=6, rw)]
        lpr_num_entries: u6,
        #[bit(0, rw)]
        pageclose: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CtrlReg2 {
        #[bit(17, rw)]
        go_2_critcal_enable: bool,
        #[bits(5..=12, rw)]
        go_2_critical_hysteresis: u8,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CtrlReg3 {
        #[bits(16..=25, rw)]
        dfi_t_wlmrd: u10,
        #[bits(8..=15, rw)]
        rdlvl_rr: u8,
        #[bits(0..=7, rw)]
        wrlvl_ww: u8,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CtrlReg4 {
        #[bits(8..=15, rw)]
        dfi_t_ctrlupd_interval_max_x1024: u8,
        #[bits(0..=7, rw)]
        dfi_t_ctrlupd_interval_min_x1024: u8,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CtrlReg5 {
        #[bits(20..=25, rw)]
        t_ckesr: u6,
        #[bits(16..=19, rw)]
        t_cksrx: u4,
        #[bits(12..=15, rw)]
        t_ckrse: u4,
        #[bits(8..=11, rw)]
        dfi_t_dram_clk_enable: u4,
        #[bits(4..=7, rw)]
        dfi_t_dram_clk_disable: u4,
        #[bits(0..=3, rw)]
        dfi_t_ctrl_delay: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CtrlReg6 {
        #[bits(16..=19, rw)]
        t_cksx: u4,
        #[bits(12..=15, rw)]
        t_ckdpdx: u4,
        #[bits(8..=11, rw)]
        t_ckdpde: u4,
        #[bits(4..=7, rw)]
        t_ckpdx: u4,
        #[bits(0..=3, rw)]
        t_ckpde: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CheTZq {
        #[bits(22..=31, rw)]
        t_zq_short_nop: u10,
        #[bits(12..=21, rw)]
        t_zq_long_nop: u10,
        #[bits(2..=11, rw)]
        t_mode: u10,
        #[bit(1, rw)]
        ddr3: bool,
        #[bit(0, rw)]
        dis_auto_zq: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CheTZqShortInterval {
        #[bits(20..=27, rw)]
        dram_rstn_x1024: u8,
        #[bits(0..=19, rw)]
        t_zq_short_interval: u20,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DeepPowerdown {
        #[bits(1..=8, rw)]
        deep_powerdown_to_x1024: u8,
        #[bit(0, rw)]
        enable: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct Reg2c {
        #[bit(28, rw)]
        dfi_rd_data_eye_train: bool,
        #[bit(27, rw)]
        dfi_rd_dqs_gate_level: bool,
        #[bit(26, rw)]
        dfi_wr_level_enable: bool,
        #[bit(25, rw)]
        trdlvl_max_error: bool,
        #[bit(24, rw)]
        twrlvl_max_error: bool,
        #[bits(12..=23, rw)]
        dfi_rdlvl_max_x1024: u12,
        #[bits(0..=11, rw)]
        dfi_wrlvl_max_x1024: u12,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct Reg2d {
        #[bit(9, rw)]
        skip_ocd: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct DfiTiming {
        #[bits(15..=24, rw)]
        dfi_t_ctrlup_max: u10,
        #[bits(5..=14, rw)]
        dfi_t_ctrlup_min: u10,
        #[bits(0..=4, rw)]
        dfi_t_rddata_enable: u5,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct CheEccControl {
        #[bit(1, rw)]
        clear_correctable_errors: bool,
        #[bit(0, rw)]
        clear_uncorrectable_errors: bool,
    }

    #[bitbybit::bitenum(u3, exhaustive = false)]
    #[derive(Debug)]
    pub enum EccMode {
        NoEcc = 0b000,
        SecDecOverOneBeat = 0b100,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct EccScrub {
        #[bit(3, rw)]
        disable_scrub: bool,
        #[bits(0..=2, rw)]
        ecc_mode: Option<EccMode>,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct PhyReceiverEnable {
        #[bits(4..=7, rw)]
        phy_dif_off: u4,
        #[bits(0..=3, rw)]
        phy_dif_on: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct PhyConfig {
        #[bits(24..=30, rw)]
        dq_offset: u7,
        #[bit(3, rw)]
        wrlvl_inc_mode: bool,
        #[bit(2, rw)]
        gatelvl_inc_mode: bool,
        #[bit(1, rw)]
        rdlvl_inc_mode: bool,
        #[bit(0, rw)]
        data_slice_in_use: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct PhyInitRatio {
        #[bits(10..=19, rw)]
        gatelvl_init_ratio: u10,
        #[bits(0..=9, rw)]
        wrlvl_init_ratio: u10,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct PhyDqsConfig {
        #[bits(11..=19, rw)]
        dqs_slave_delay: u9,
        #[bit(10, rw)]
        dqs_slave_force: bool,
        #[bits(0..=9, rw)]
        dqs_slave_ratio: u10,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct PhyWriteEnableConfig {
        #[bits(12..=20, rw)]
        fifo_we_in_delay: u9,
        #[bit(11, rw)]
        fifo_we_in_force: bool,
        #[bits(0..=10, rw)]
        fifo_we_slave_ratio: u11,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct PhyWriteDataSlaveConfig {
        #[bits(11..=19, rw)]
        wr_data_slave_delay: u9,
        #[bit(10, rw)]
        wr_data_slave_force: bool,
        #[bits(0..=9, rw)]
        wr_data_slave_ratio: u10,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct Reg64 {
        #[bit(30, rw)]
        cmd_latency: bool,
        #[bit(29, rw)]
        lpddr: bool,
        #[bits(21..=27, rw)]
        ctrl_slave_delay: u7,
        #[bit(20, rw)]
        ctrl_slave_force: bool,
        #[bits(10..=19, rw)]
        ctrl_slave_ratio: u10,
        #[bit(9, rw)]
        sel_logic: bool,
        #[bit(7, rw)]
        invert_clkout: bool,
        #[bit(1, rw)]
        bl2: bool,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct Reg65 {
        #[bits(18..=19, rw)]
        ctrl_slave_delay: u2,
        #[bit(17, rw)]
        dis_calib_rst: bool,
        #[bit(16, rw)]
        use_rd_data_eye_level: bool,
        #[bit(15, rw)]
        use_rd_dqs_gate_level: bool,
        #[bit(14, rw)]
        use_wr_level: bool,
        #[bits(10..=13, rw)]
        dll_lock_diff: u4,
        #[bits(5..=9, rw)]
        rd_rl_delay: u5,
        #[bits(0..=4, rw)]
        wr_rl_delay: u5,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct AxiPriorityWritePort {
        #[bit(18, rw)]
        disable_page_match: bool,
        #[bit(17, rw)]
        disable_urgent: bool,
        #[bit(16, rw)]
        disable_aging: bool,
        #[bits(0..=9, rw)]
        pri_wr_port: u10,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct AxiPriorityReadPort {
        #[bit(19, rw)]
        enable_hpr: bool,
        #[bit(18, rw)]
        disable_page_match: bool,
        #[bit(17, rw)]
        disable_urgent: bool,
        #[bit(16, rw)]
        disable_aging: bool,
        #[bits(0..=9, rw)]
        pri_rd_port_n: u10,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct ExclusiveAccessConfig {
        #[bits(9..=17, rw)]
        access_id1_port: u9,
        #[bits(0..=8, rw)]
        access_id0_port: u9,
    }

    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug)]
    pub enum LpddrBit {
        Ddr2Ddr3 = 0,
        Lpddr2 = 1,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct LpddrControl0 {
        #[bits(4..=11, rw)]
        mr4_margin: u8,
        #[bit(2, rw)]
        derate_enable: bool,
        #[bit(1, rw)]
        per_bank_refresh: bool,
        #[bit(0, rw)]
        lpddr2: LpddrBit,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct LpddrControl1 {
        #[bits(0..=31, rw)]
        mr4_read_interval: u32,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct LpddrControl2 {
        #[bits(12..=21, rw)]
        t_mrw: u10,
        #[bits(4..=11, rw)]
        idle_after_reset_x32: u8,
        #[bits(0..=3, rw)]
        min_stable_clock_x1: u4,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct LpddrControl3 {
        #[bits(8..=17, rw)]
        dev_zqinit_x32: u10,
        #[bits(0..=7, rw)]
        max_auto_init_x1024: u8,
    }

    #[bitbybit::bitenum(u3, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    pub enum OperatingMode {
        DdrcInit = 0,
        NormalOperation = 1,
        Powerdown = 2,
        SelfRefresh = 3,
        DeepPowerdown = 4,
        DeepPowerdownAlt1 = 5,
        DeepPowerdownAlt2 = 6,
        DeepPowerdownAlt3 = 7,
    }

    impl OperatingMode {
        pub fn is_deep_powerdown(&self) -> bool {
            matches!(
                self,
                OperatingMode::DeepPowerdown
                    | OperatingMode::DeepPowerdownAlt1
                    | OperatingMode::DeepPowerdownAlt2
                    | OperatingMode::DeepPowerdownAlt3
            )
        }
    }

    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    pub enum DebugStallBit {
        CommandsAccepted = 0,
        CommandsNotAccepted = 1,
    }

    #[bitbybit::bitfield(u32, default = 0x0, debug)]
    pub struct ModeStatus {
        #[bits(16..=20, r)]
        dbg_hpr_queue_depth: u5,
        #[bits(10..=15, r)]
        dbg_lpr_queue_depth: u6,
        #[bits(4..=9, r)]
        dbg_wr_queue_depth: u6,
        #[bit(3, r)]
        dbg_stall: DebugStallBit,
        #[bits(0..=2, r)]
        operating_mode: OperatingMode,
    }
}

use regs::*;

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct DdrController {
    ddrc_ctrl: DdrcControl,
    two_rank_cfg: TwoRankConfig,
    hpr_queue_ctrl: LprHprQueueControl,
    lpr_queue_ctrl: LprHprQueueControl,
    wr_reg: WriteQueueControl,
    dram_param_reg0: DramParamReg0,
    dram_param_reg1: DramParamReg1,
    dram_param_reg2: DramParamReg2,
    dram_param_reg3: DramParamReg3,
    dram_param_reg4: DramParamReg4,
    dram_init_param: DramInitParam,
    dram_emr: DramEmr,
    dram_emr_mr: DramEmrMr,
    dram_burst8_rdwr: DramBurst8ReadWrite,
    dram_disable_dq: DisableDq,
    dram_addr_map_bank: DramAddrMapBank,
    dram_addr_map_col: DramAddrMapColumn,
    dram_addr_map_row: DramAddrMapRow,
    dram_odt_reg: DramOdt,
    #[mmio(PureRead)]
    phy_debug_reg: u32,
    phy_cmd_timeout_rddata_cpt: PhyCmdTimeoutRdDataCpt,
    #[mmio(PureRead)]
    mode_status: ModeStatus,
    dll_calib: DllCalib,
    odt_delay_hold: OdtDelayHold,
    ctrl_reg1: CtrlReg1,
    ctrl_reg2: CtrlReg2,
    ctrl_reg3: CtrlReg3,
    ctrl_reg4: CtrlReg4,
    _reserved0: [u32; 0x2],
    ctrl_reg5: CtrlReg5,
    ctrl_reg6: CtrlReg6,

    _reserved1: [u32; 0x8],

    che_refresh_timer_01: u32,
    che_t_zq: CheTZq,
    che_t_zq_short_interval_reg: CheTZqShortInterval,
    deep_powerdown_reg: DeepPowerdown,
    reg_2c: Reg2c,
    reg_2d: Reg2d,
    dfi_timing: DfiTiming,
    _reserved2: [u32; 0x2],
    che_ecc_control: CheEccControl,
    #[mmio(PureRead)]
    che_corr_ecc_log: u32,
    #[mmio(PureRead)]
    che_corr_ecc_addr: u32,
    #[mmio(PureRead)]
    che_corr_ecc_data_31_0: u32,
    #[mmio(PureRead)]
    che_corr_ecc_data_63_32: u32,
    #[mmio(PureRead)]
    che_corr_ecc_data_71_64: u32,
    /// Clear on write, but the write is performed on another register.
    #[mmio(PureRead)]
    che_uncorr_ecc_log: u32,
    #[mmio(PureRead)]
    che_uncorr_ecc_addr: u32,
    #[mmio(PureRead)]
    che_uncorr_ecc_data_31_0: u32,
    #[mmio(PureRead)]
    che_uncorr_ecc_data_63_32: u32,
    #[mmio(PureRead)]
    che_uncorr_ecc_data_71_64: u32,
    #[mmio(PureRead)]
    che_ecc_stats: u32,
    ecc_scrub: EccScrub,
    #[mmio(PureRead)]
    che_ecc_corr_bit_mask_31_0: u32,
    #[mmio(PureRead)]
    che_ecc_corr_bit_mask_63_32: u32,

    _reserved3: [u32; 0x5],

    phy_receiver_enable: PhyReceiverEnable,
    phy_config: [PhyConfig; 0x4],
    _reserved4: u32,
    phy_init_ratio: [PhyInitRatio; 4],
    _reserved5: u32,
    phy_rd_dqs_cfg: [PhyDqsConfig; 4],
    _reserved6: u32,
    phy_wr_dqs_cfg: [PhyDqsConfig; 4],
    _reserved7: u32,
    phy_we_cfg: [PhyWriteEnableConfig; 4],
    _reserved8: u32,
    phy_wr_data_slave: [PhyWriteDataSlaveConfig; 4],

    _reserved9: u32,

    reg_64: Reg64,
    reg_65: Reg65,
    _reserved10: [u32; 3],
    #[mmio(PureRead)]
    reg69_6a0: u32,
    #[mmio(PureRead)]
    reg69_6a1: u32,
    _reserved11: u32,
    #[mmio(PureRead)]
    reg69_6d2: u32,
    #[mmio(PureRead)]
    reg69_6d3: u32,
    #[mmio(PureRead)]
    reg69_710: u32,
    #[mmio(PureRead)]
    reg6e_711: u32,
    #[mmio(PureRead)]
    reg6e_712: u32,
    #[mmio(PureRead)]
    reg6e_713: u32,
    _reserved12: u32,
    #[mmio(PureRead)]
    phy_dll_status: [u32; 4],
    _reserved13: u32,
    #[mmio(PureRead)]
    dll_lock_status: u32,
    #[mmio(PureRead)]
    phy_control_status: u32,
    #[mmio(PureRead)]
    phy_control_status_2: u32,

    _reserved14: [u32; 0x5],

    // DDRI registers.
    #[mmio(PureRead)]
    axi_id: u32,
    page_mask: u32,
    axi_priority_wr_port: [AxiPriorityWritePort; 0x4],
    axi_priority_rd_port: [AxiPriorityReadPort; 0x4],

    _reserved15: [u32; 0x1B],

    excl_access_cfg: [ExclusiveAccessConfig; 0x4],
    #[mmio(PureRead)]
    mode_reg_read: u32,
    lpddr_ctrl_0: LpddrControl0,
    lpddr_ctrl_1: LpddrControl1,
    lpddr_ctrl_2: LpddrControl2,
    lpddr_ctrl_3: LpddrControl3,
}

static_assertions::const_assert_eq!(core::mem::size_of::<DdrController>(), 0x2B8);

impl DdrController {
    /// Create a new DDR MMIO instance for the DDR controller at address [DDRC_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioDdrController<'static> {
        unsafe { Self::new_mmio_at(DDRC_BASE_ADDR) }
    }
}
