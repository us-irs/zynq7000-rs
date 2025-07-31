use arbitrary_int::{u2, u3, u4, u5, u6, u7, u10, u11, u12};

pub const DDRC_BASE_ADDR: usize = 0xF800_6000;

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
#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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
#[bitbybit::bitfield(u32, default = 0x0)]
pub struct LprHprQueueControl {
    #[bits(22..=25, rw)]
    xact_run_length: u4,
    #[bits(11..=21, rw)]
    max_starve_x32: u11,
    #[bits(0..=10, rw)]
    min_non_critical_x32: u11,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct WriteQueueControl {
    #[bits(15..=25, rw)]
    max_starve_x32: u11,
    #[bits(11..=14, rw)]
    xact_run_length: u4,
    #[bits(0..=10, rw)]
    min_non_critical_x32: u11,
}

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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
pub enum MobileSetting {
    Ddr2Ddr3 = 0,
    Lpddr2 = 1,
}
#[bitbybit::bitfield(u32, default = 0x0)]
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
pub enum ModeRegisterType {
    Write = 0,
    Read = 1,
}

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct DramInitParam {
    #[bits(11..=13, rw)]
    t_mrd: u3,
    #[bits(7..=10, rw)]
    pre_ocd_x32: u4,
    #[bits(0..=6, rw)]
    final_wait_x32: u7,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct DramEmr {
    #[bits(16..=31, rw)]
    emr3: u16,
    #[bits(0..=15, rw)]
    emr2: u16,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct DramEmrMr {
    #[bits(16..=31, rw)]
    emr: u16,
    #[bits(0..=15, rw)]
    mr: u16,
}

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct DisableDq {
    #[bit(1, rw)]
    dis_dq: bool,
    #[bit(0, rw)]
    force_low_pri_n: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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
pub enum DllCalibSel {
    Periodic = 0,
    Manual = 1,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct DllCalib {
    #[bit(16, rw)]
    sel: DllCalibSel,
}

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct CtrlReg2 {
    #[bit(17, rw)]
    go_2_critcal_enable: bool,
    #[bits(5..=12, rw)]
    go_2_critical_hysteresis: u8,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct CtrlReg3 {
    #[bits(16..=25, rw)]
    dfi_t_wlmrd: u10,
    #[bits(8..=15, rw)]
    rdlvl_rr: u8,
    #[bits(0..=7, rw)]
    wrlvl_ww: u8,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct CtrlReg4 {
    #[bits(8..=15, rw)]
    dfi_t_ctrlupd_interval_max_x1024: u8,
    #[bits(0..=7, rw)]
    dfi_t_ctrlupd_interval_min_x1024: u8,
}

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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

#[bitbybit::bitfield(u32, default = 0x0)]
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
    mode_status: u32,
    dll_calib: DllCalib,
    odt_delay_hold: u32,
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
    che_t_zq_short_interval_reg: u32,
    deep_powerdown_reg: u32,
    reg_2c: u32,
    reg_2d: u32,
    dfi_timing: u32,
    _reserved2: [u32; 0x2],
    che_corr_control: u32,
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
    ecc_scrub: u32,
    che_ecc_corr_bit_mask_31_0: u32,
    che_ecc_corr_bit_mask_63_32: u32,

    _reserved3: [u32; 0x5],

    phy_receiver_enable: u32,
    phy_config: [u32; 4],
    _reserved4: u32,
    phy_init_ratio: [u32; 4],
    _reserved5: u32,
    phy_rd_dqs_cfg: [u32; 4],
    _reserved6: u32,
    phy_wr_dqs_cfg: [u32; 4],
    _reserved7: u32,
    phy_we_cfg_0: [u32; 4],
    _reserved8: u32,
    wr_data_slave: [u32; 4],

    _reserved9: u32,

    reg_64: u32,
    reg_65: u32,
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
    axi_priority_wr_port: [u32; 4],
    axi_priority_rd_port: [u32; 4],

    _reserved15: [u32; 0x1B],

    excl_access_cfg: [u32; 4],
    #[mmio(PureRead)]
    mode_reg_read: u32,
    lpddr_ctrl_0: u32,
    lpddr_ctrl_1: u32,
    lpddr_ctrl_2: u32,
    lpddr_ctrl_3: u32,
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
