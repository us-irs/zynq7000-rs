/// Base address of the DDR memory controller register block.
pub const DDRC_BASE_ADDR: usize = 0xF800_6000;

/// Bitfield and enum types for the DDR controller registers.
pub mod regs {
    pub use crate::slcr::ddriob::DdriobConfig;
    use arbitrary_int::{u2, u3, u4, u5, u6, u7, u9, u10, u11, u12, u20};

    /// DRAM data bus width.
    #[bitbybit::bitenum(u2)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DataBusWidth {
        /// 32-bit wide data bus.
        _32Bit = 0b00,
        /// 16-bit wide data bus.
        _16Bit = 0b01,
    }

    /// Soft reset state of the DDR controller.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SoftReset {
        /// Controller is held in reset.
        Reset = 0,
        /// Controller is out of reset and active.
        Active = 1,
    }
    /// Top-level DDR controller configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DdrcControl {
        /// Disables automatic refresh generation.
        #[bit(16, rw)]
        disable_auto_refresh: bool,
        /// Disables bypassing of active commands in the queue.
        #[bit(15, rw)]
        disable_active_bypass: bool,
        /// Disables bypassing of read commands in the queue.
        #[bit(14, rw)]
        disable_read_bypass: bool,
        /// Idle cycles inserted between read and write bursts.
        #[bits(7..=13, rw)]
        read_write_idle_gap: u7,
        /// Number of burst-of-8 reads or writes allowed before a refresh.
        #[bits(4..=6, rw)]
        burst8_refresh: u3,
        /// DRAM data bus width.
        #[bits(2..=3, rw)]
        data_bus_width: Option<DataBusWidth>,
        /// Enables automatic power-down after idle.
        #[bit(1, rw)]
        power_down_enable: bool,
        /// Soft resets the DDR controller.
        #[bit(0, rw)]
        soft_reset: SoftReset,
    }

    /// Rank configuration and refresh timing.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct TwoRankConfig {
        /// Address bit used to select the chip select for rank 1.
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
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct LprHprQueueControl {
        /// Number of transactions serviced once the queue becomes critical.
        #[bits(22..=25, rw)]
        xact_run_length: u4,
        /// Number of cycles, in multiples of 32, before entries become critical.
        #[bits(11..=21, rw)]
        max_starve_x32: u11,
        /// Minimum number of cycles, in multiples of 32, an entry stays non-critical.
        #[bits(0..=10, rw)]
        min_non_critical_x32: u11,
    }

    /// Write queue control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct WriteQueueControl {
        /// Number of cycles, in multiples of 32, before entries become critical.
        #[bits(15..=25, rw)]
        max_starve_x32: u11,
        /// Number of transactions serviced once the queue becomes critical.
        #[bits(11..=14, rw)]
        xact_run_length: u4,
        /// Minimum number of cycles, in multiples of 32, an entry stays non-critical.
        #[bits(0..=10, rw)]
        min_non_critical_x32: u11,
    }

    /// DRAM timing parameters, part 0.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
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

    /// DRAM timing parameters, part 1.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramParamReg1 {
        /// tCKE - Minimum clock enable pulse width.
        #[bits(28..=31, rw)]
        t_cke: u4,
        /// tRAS(min) - Minimum time between activate and precharge to the same bank.
        #[bits(22..=26, rw)]
        t_ras_min: u5,
        /// tRAS(max) - Maximum time between activate and precharge to the same bank.
        #[bits(16..=21, rw)]
        t_ras_max: u6,
        /// tFAW - Four activate window.
        #[bits(10..=15, rw)]
        t_faw: u6,
        /// Minimum time in power-down before it may be exited, in multiples of 32 clocks.
        #[bits(5..=9, rw)]
        powerdown_to_x32: u5,
        /// Write to precharge minimum delay.
        #[bits(0..=4, rw)]
        wr2pre: u5,
    }

    /// DRAM timing parameters, part 2.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramParamReg2 {
        /// tRCD - Minimum time from activate to read or write.
        #[bits(28..=31, rw)]
        t_rcd: u4,
        /// Read to precharge minimum delay.
        #[bits(23..=27, rw)]
        rd2pre: u5,
        /// Pad power-down delay.
        #[bits(20..=22, rw)]
        pad_pd: u3,
        /// tXP - Minimum time from power-down exit to the next valid command.
        #[bits(15..=19, rw)]
        t_xp: u5,
        /// Write to read minimum delay.
        #[bits(10..=14, rw)]
        wr2rd: u5,
        /// Read to write minimum delay.
        #[bits(5..=9, rw)]
        rd2wr: u5,
        /// Write latency in clock cycles.
        #[bits(0..=4, rw)]
        write_latency: u5,
    }

    /// Weird naming.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum MobileSetting {
        /// DDR2 or DDR3 mode.
        Ddr2Ddr3 = 0,
        /// LPDDR2 mode.
        Lpddr2 = 1,
    }
    /// DRAM timing parameters, part 3.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramParamReg3 {
        /// Disables the pad power-down feature.
        #[bit(30, rw)]
        disable_pad_pd_feature: bool,
        /// Read latency in clock cycles.
        #[bits(24..=28, rw)]
        read_latency: u5,
        /// Enables the DFI DRAM clock disable signal.
        #[bit(23, rw)]
        enable_dfi_dram_clk_disable: bool,
        /// 0: DDR2 or DDR3. 1: LPDDR2.
        #[bit(22, rw)]
        mobile: MobileSetting,
        /// Must be set to 0.
        #[bit(21, rw)]
        sdram: bool,
        /// Maximum refresh interval, in multiples of 32 clocks.
        #[bits(16..=20, rw)]
        refresh_to_x32: u5,
        /// tRP - Minimum time from precharge to activate or refresh.
        #[bits(12..=15, rw)]
        t_rp: u4,
        /// Number of cycles a refresh may be delayed to avoid colliding with other commands.
        #[bits(8..=11, rw)]
        refresh_margin: u4,
        /// tRRD - Minimum time between activates to different banks.
        #[bits(5..=7, rw)]
        t_rrd: u3,
        /// tCCD - Minimum time between column accesses.
        #[bits(2..=4, rw)]
        t_ccd: u3,
    }

    /// Direction of a mode register access.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ModeRegisterType {
        /// Write to the mode register.
        Write = 0,
        /// Read from the mode register.
        Read = 1,
    }

    /// DRAM mode register access control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramParamReg4 {
        /// Indicates the mode register read data is valid.
        #[bit(27, rw)]
        mr_rdata_valid: bool,
        /// Selects a mode register write or read access.
        #[bit(26, rw)]
        mr_type: ModeRegisterType,
        /// Indicates a mode register write is in progress.
        #[bit(25, rw)]
        mr_wr_busy: bool,
        /// Data written to or read from the mode register.
        #[bits(9..=24, rw)]
        mr_data: u16,
        /// Selects which mode register is accessed.
        #[bits(7..=8, rw)]
        mr_addr: u2,
        /// Triggers a mode register access.
        #[bit(6, rw)]
        mr_wr: bool,
        /// Prioritizes writes over reads in the queue.
        #[bit(1, rw)]
        prefer_write: bool,
        /// Enables 2T command timing mode.
        #[bit(0, rw)]
        enable_2t_timing_mode: bool,
    }

    /// DRAM initialization timing parameters.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramInitParam {
        /// tMRD - Minimum time between mode register writes.
        #[bits(11..=13, rw)]
        t_mrd: u3,
        /// Time before OCD calibration, in multiples of 32 clocks.
        #[bits(7..=10, rw)]
        pre_ocd_x32: u4,
        /// Final wait time after initialization, in multiples of 32 clocks.
        #[bits(0..=6, rw)]
        final_wait_x32: u7,
    }

    /// DRAM extended mode registers 2 and 3.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramEmr {
        /// Value written to extended mode register 3.
        #[bits(16..=31, rw)]
        emr3: u16,
        /// Value written to extended mode register 2.
        #[bits(0..=15, rw)]
        emr2: u16,
    }

    /// DRAM mode register and extended mode register 1.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramEmrMr {
        /// Value written to extended mode register 1.
        #[bits(16..=31, rw)]
        emr: u16,
        /// Value written to the mode register.
        #[bits(0..=15, rw)]
        mr: u16,
    }

    /// Burst-of-8 read and write timing.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramBurst8ReadWrite {
        /// Burst length used for reads and writes.
        #[bits(0..=3, rw)]
        burst_rdwr: u4,
        /// Clock enable setup time before initialization, in multiples of 1024 clocks.
        #[bits(4..=13, rw)]
        pre_cke_x1024: u10,
        /// Clock enable hold time after initialization, in multiples of 1024 clocks.
        #[bits(16..=25, rw)]
        post_cke_x1024: u10,
        /// Enables burst chop mode.
        #[bit(26, rw)]
        burstchop: bool,
    }

    /// DQ bus disable control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DisableDq {
        /// Disables the DQ data bus.
        #[bit(1, rw)]
        dis_dq: bool,
        /// Forces low priority for the AXI port.
        #[bit(0, rw)]
        force_low_pri_n: bool,
    }

    /// Maps DRAM bank address bits to AXI address bits.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramAddrMapBank {
        /// Offset of the AXI address bit used for DRAM bank address bit 6.
        #[bits(16..=19, rw)]
        addrmap_bank_b6: u4,
        /// Offset of the AXI address bit used for DRAM bank address bit 5.
        #[bits(12..=15, rw)]
        addrmap_bank_b5: u4,
        /// Offset of the AXI address bit used for DRAM bank address bit 2.
        #[bits(8..=11, rw)]
        addrmap_bank_b2: u4,
        /// Offset of the AXI address bit used for DRAM bank address bit 1.
        #[bits(4..=7, rw)]
        addrmap_bank_b1: u4,
        /// Offset of the AXI address bit used for DRAM bank address bit 0.
        #[bits(0..=3, rw)]
        addrmap_bank_b0: u4,
    }

    /// Maps DRAM column address bits to AXI address bits.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramAddrMapColumn {
        /// Offset of the AXI address bit used for DRAM column address bit 11.
        #[bits(28..=31, rw)]
        addrmap_col_b11: u4,
        /// Offset of the AXI address bit used for DRAM column address bit 10.
        #[bits(24..=27, rw)]
        addrmap_col_b10: u4,
        /// Offset of the AXI address bit used for DRAM column address bit 9.
        #[bits(20..=23, rw)]
        addrmap_col_b9: u4,
        /// Offset of the AXI address bit used for DRAM column address bit 8.
        #[bits(16..=19, rw)]
        addrmap_col_b8: u4,
        /// Offset of the AXI address bit used for DRAM column address bit 7.
        #[bits(12..=15, rw)]
        addrmap_col_b7: u4,
        /// Offset of the AXI address bit used for DRAM column address bit 4.
        #[bits(8..=11, rw)]
        addrmap_col_b4: u4,
        /// Offset of the AXI address bit used for DRAM column address bit 3.
        #[bits(4..=7, rw)]
        addrmap_col_b3: u4,
        /// Offset of the AXI address bit used for DRAM column address bit 2.
        #[bits(0..=3, rw)]
        addrmap_col_b2: u4,
    }

    /// Maps DRAM row address bits to AXI address bits.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramAddrMapRow {
        /// Offset of the AXI address bit used for DRAM row address bit 15.
        #[bits(24..=27, rw)]
        addrmap_row_b15: u4,
        /// Offset of the AXI address bit used for DRAM row address bit 14.
        #[bits(20..=23, rw)]
        addrmap_row_b14: u4,
        /// Offset of the AXI address bit used for DRAM row address bit 13.
        #[bits(16..=19, rw)]
        addrmap_row_b13: u4,
        /// Offset of the AXI address bit used for DRAM row address bit 12.
        #[bits(12..=15, rw)]
        addrmap_row_b12: u4,
        /// Offset of the AXI address bits used for DRAM row address bits 2 through 11.
        #[bits(8..=11, rw)]
        addrmap_row_b2_11: u4,
        /// Offset of the AXI address bit used for DRAM row address bit 1.
        #[bits(4..=7, rw)]
        addrmap_row_b1: u4,
        /// Offset of the AXI address bit used for DRAM row address bit 0.
        #[bits(0..=3, rw)]
        addrmap_row_b0: u4,
    }

    /// On-die termination (ODT) configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DramOdt {
        /// Local ODT setting while the PHY is idle.
        #[bits(16..=17, rw)]
        phy_idle_local_odt: u2,
        /// Local ODT setting during writes.
        #[bits(14..=15, rw)]
        phy_write_local_odt: u2,
        /// Local ODT setting during reads.
        #[bits(12..=13, rw)]
        phy_read_local_odt: u2,
        /// ODT setting applied to rank 0 during writes.
        #[bits(3..=5, rw)]
        rank0_wr_odt: u3,
        /// ODT setting applied to rank 0 during reads.
        #[bits(0..=2, rw)]
        rank0_rd_odt: u3,
    }

    /// PHY command timeout and read data capture control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PhyCmdTimeoutRdDataCpt {
        /// DQ bit used as reference during write leveling.
        #[bits(28..=31, rw)]
        wrlvl_num_of_dq0: u4,
        /// DQ bit used as reference during read gate training.
        #[bits(24..=27, rw)]
        gatelvl_num_of_dq0: u4,
        /// Stalls the DFI clock while asserted.
        #[bit(19, rw)]
        clk_stall_level: bool,
        /// Disables the PHY control reset.
        #[bit(18, rw)]
        dis_phy_ctrl_rstn: bool,
        /// Clears the read data capture FIFO error counter.
        #[bit(17, rw)]
        rdc_fifo_rst_err_cnt_clr: bool,
        /// Uses a fixed read enable instead of the trained value.
        #[bit(16, rw)]
        use_fixed_re: bool,
        /// Delay from write enable to read enable in the read data capture FIFO.
        #[bits(8..=11, rw)]
        rdc_we_to_re_delay: u4,
        /// Delay from write command to write data.
        #[bits(4..=7, rw)]
        wr_cmd_to_data: u4,
        /// Delay from read command to read data.
        #[bits(0..=3, rw)]
        rd_cmd_to_data: u4,
    }

    /// DLL calibration trigger mode.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DllCalibSel {
        /// Calibration runs periodically.
        Periodic = 0,
        /// Calibration is triggered manually.
        Manual = 1,
    }

    /// DLL calibration control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DllCalib {
        /// Selects periodic or manual DLL calibration.
        #[bit(16, rw)]
        sel: DllCalibSel,
    }

    /// ODT delay and hold timing.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct OdtDelayHold {
        /// Number of cycles ODT is held after a write.
        #[bits(12..=15, rw)]
        wr_odt_hold: u4,
        /// Number of cycles ODT is held after a read.
        #[bits(8..=11, rw)]
        rd_odt_hold: u4,
        /// Delay before ODT is asserted for a write.
        #[bits(4..=7, rw)]
        wr_odt_delay: u4,
        /// Delay before ODT is asserted for a read.
        #[bits(0..=3, rw)]
        rd_odt_delay: u4,
    }

    /// Controller configuration, register 1.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CtrlReg1 {
        /// Puts the DRAM into self refresh.
        #[bit(12, rw)]
        selfref_enable: bool,
        /// Disables page hit optimization for colliding accesses.
        #[bit(10, rw)]
        dis_collision_page_opt: bool,
        /// Disables write combining.
        #[bit(9, rw)]
        dis_wc: bool,
        /// Updates the refresh rate immediately instead of on the next refresh.
        #[bit(8, rw)]
        refresh_update_level: bool,
        /// Enables automatic precharge.
        #[bit(7, rw)]
        auto_pre_en: bool,
        /// Number of entries reserved for the low priority read queue.
        #[bits(1..=6, rw)]
        lpr_num_entries: u6,
        /// Closes a page as soon as there are no more pending accesses to it.
        #[bit(0, rw)]
        pageclose: bool,
    }

    /// Controller configuration, register 2.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CtrlReg2 {
        /// Enables the go-to-critical mechanism for the write queue.
        #[bit(17, rw)]
        go_2_critcal_enable: bool,
        /// Hysteresis applied to the go-to-critical threshold.
        #[bits(5..=12, rw)]
        go_2_critical_hysteresis: u8,
    }

    /// Controller configuration, register 3.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CtrlReg3 {
        /// tWLMRD - Time from write leveling mode register set to the first DQS pulse.
        #[bits(16..=25, rw)]
        dfi_t_wlmrd: u10,
        /// Number of read gate training rounds.
        #[bits(8..=15, rw)]
        rdlvl_rr: u8,
        /// Number of write leveling rounds.
        #[bits(0..=7, rw)]
        wrlvl_ww: u8,
    }

    /// Controller configuration, register 4.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CtrlReg4 {
        /// Maximum interval between DFI controller updates, in multiples of 1024 clocks.
        #[bits(8..=15, rw)]
        dfi_t_ctrlupd_interval_max_x1024: u8,
        /// Minimum interval between DFI controller updates, in multiples of 1024 clocks.
        #[bits(0..=7, rw)]
        dfi_t_ctrlupd_interval_min_x1024: u8,
    }

    /// Controller configuration, register 5.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CtrlReg5 {
        /// tCKESR - Minimum clock enable pulse width during self refresh.
        #[bits(20..=25, rw)]
        t_ckesr: u6,
        /// tCKSRX - Minimum time before the clock is stable after self refresh exit.
        #[bits(16..=19, rw)]
        t_cksrx: u4,
        /// Time from clock enable to the first valid command after self refresh exit.
        #[bits(12..=15, rw)]
        t_ckrse: u4,
        /// Time from DFI clock enable to the DRAM clock being enabled.
        #[bits(8..=11, rw)]
        dfi_t_dram_clk_enable: u4,
        /// Time from DFI clock disable to the DRAM clock being disabled.
        #[bits(4..=7, rw)]
        dfi_t_dram_clk_disable: u4,
        /// Delay between a DFI control signal change and the DRAM command.
        #[bits(0..=3, rw)]
        dfi_t_ctrl_delay: u4,
    }

    /// Controller configuration, register 6.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CtrlReg6 {
        /// Minimum time the clock must be stable before exiting power-down.
        #[bits(16..=19, rw)]
        t_cksx: u4,
        /// Minimum time before exiting deep power-down.
        #[bits(12..=15, rw)]
        t_ckdpdx: u4,
        /// Minimum time before entering deep power-down.
        #[bits(8..=11, rw)]
        t_ckdpde: u4,
        /// Minimum time before exiting power-down.
        #[bits(4..=7, rw)]
        t_ckpdx: u4,
        /// Minimum time before entering power-down.
        #[bits(0..=3, rw)]
        t_ckpde: u4,
    }

    /// ZQ calibration timing.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CheTZq {
        /// Number of cycles the controller waits during a short ZQ calibration.
        #[bits(22..=31, rw)]
        t_zq_short_nop: u10,
        /// Number of cycles the controller waits during a long ZQ calibration.
        #[bits(12..=21, rw)]
        t_zq_long_nop: u10,
        /// tMOD - Time from mode register set to a non-mode-register-set command.
        #[bits(2..=11, rw)]
        t_mode: u10,
        /// Selects DDR3 mode for ZQ calibration commands.
        #[bit(1, rw)]
        ddr3: bool,
        /// Disables automatic ZQ calibration.
        #[bit(0, rw)]
        dis_auto_zq: bool,
    }

    /// ZQ calibration interval and DRAM reset timing.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CheTZqShortInterval {
        /// Width of the DRAM reset pulse, in multiples of 1024 clocks.
        #[bits(20..=27, rw)]
        dram_rstn_x1024: u8,
        /// Interval between automatic short ZQ calibrations.
        #[bits(0..=19, rw)]
        t_zq_short_interval: u20,
    }

    /// Deep power-down control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DeepPowerdown {
        /// Time spent in deep power-down, in multiples of 1024 clocks.
        #[bits(1..=8, rw)]
        deep_powerdown_to_x1024: u8,
        /// Enables deep power-down mode.
        #[bit(0, rw)]
        enable: bool,
    }

    /// Leveling and training status and control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Reg2c {
        /// Enables read data eye training.
        #[bit(28, rw)]
        dfi_rd_data_eye_train: bool,
        /// Enables read DQS gate leveling.
        #[bit(27, rw)]
        dfi_rd_dqs_gate_level: bool,
        /// Enables write leveling.
        #[bit(26, rw)]
        dfi_wr_level_enable: bool,
        /// Indicates the read leveling error exceeded the maximum.
        #[bit(25, rw)]
        trdlvl_max_error: bool,
        /// Indicates the write leveling error exceeded the maximum.
        #[bit(24, rw)]
        twrlvl_max_error: bool,
        /// Maximum time allowed for read leveling, in multiples of 1024 clocks.
        #[bits(12..=23, rw)]
        dfi_rdlvl_max_x1024: u12,
        /// Maximum time allowed for write leveling, in multiples of 1024 clocks.
        #[bits(0..=11, rw)]
        dfi_wrlvl_max_x1024: u12,
    }

    /// OCD calibration control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Reg2d {
        /// Skips off-chip driver calibration.
        #[bit(9, rw)]
        skip_ocd: bool,
    }

    /// DFI control update timing.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DfiTiming {
        /// Maximum time the DFI control update signal is asserted.
        #[bits(15..=24, rw)]
        dfi_t_ctrlup_max: u10,
        /// Minimum time the DFI control update signal is asserted.
        #[bits(5..=14, rw)]
        dfi_t_ctrlup_min: u10,
        /// Delay from a read command to the DFI read data enable signal.
        #[bits(0..=4, rw)]
        dfi_t_rddata_enable: u5,
    }

    /// ECC error clearing control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CheEccControl {
        /// Clears the logged correctable ECC error.
        #[bit(1, rw)]
        clear_correctable_errors: bool,
        /// Clears the logged uncorrectable ECC error.
        #[bit(0, rw)]
        clear_uncorrectable_errors: bool,
    }

    /// ECC operating mode.
    #[bitbybit::bitenum(u3, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum EccMode {
        /// ECC is disabled.
        NoEcc = 0b000,
        /// Single error correction, double error detection over one beat.
        SecDecOverOneBeat = 0b100,
    }

    /// ECC scrubbing control.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct EccScrub {
        /// Disables ECC scrubbing.
        #[bit(3, rw)]
        disable_scrub: bool,
        /// Selects the ECC operating mode.
        #[bits(0..=2, rw)]
        ecc_mode: Option<EccMode>,
    }

    /// PHY receiver enable timing.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PhyReceiverEnable {
        /// Cycles before the PHY input receiver is turned off.
        #[bits(4..=7, rw)]
        phy_dif_off: u4,
        /// Cycles before the PHY input receiver is turned on.
        #[bits(0..=3, rw)]
        phy_dif_on: u4,
    }

    /// PHY leveling configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PhyConfig {
        /// Offset applied to the DQ data eye training result.
        #[bits(24..=30, rw)]
        dq_offset: u7,
        /// Selects the increment mode used during write leveling.
        #[bit(3, rw)]
        wrlvl_inc_mode: bool,
        /// Selects the increment mode used during read gate leveling.
        #[bit(2, rw)]
        gatelvl_inc_mode: bool,
        /// Selects the increment mode used during read data eye training.
        #[bit(1, rw)]
        rdlvl_inc_mode: bool,
        /// Marks this data slice as in use.
        #[bit(0, rw)]
        data_slice_in_use: bool,
    }

    /// PHY initial leveling ratios.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PhyInitRatio {
        /// Initial ratio used for read gate leveling.
        #[bits(10..=19, rw)]
        gatelvl_init_ratio: u10,
        /// Initial ratio used for write leveling.
        #[bits(0..=9, rw)]
        wrlvl_init_ratio: u10,
    }

    /// PHY DQS slave delay configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PhyDqsConfig {
        /// DQS slave delay line setting.
        #[bits(11..=19, rw)]
        dqs_slave_delay: u9,
        /// Forces the DQS slave delay line to the programmed ratio.
        #[bit(10, rw)]
        dqs_slave_force: bool,
        /// DQS slave delay line ratio.
        #[bits(0..=9, rw)]
        dqs_slave_ratio: u10,
    }

    /// PHY write enable FIFO configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PhyWriteEnableConfig {
        /// Write enable FIFO input delay line setting.
        #[bits(12..=20, rw)]
        fifo_we_in_delay: u9,
        /// Forces the write enable FIFO input delay line to the programmed ratio.
        #[bit(11, rw)]
        fifo_we_in_force: bool,
        /// Write enable FIFO slave delay line ratio.
        #[bits(0..=10, rw)]
        fifo_we_slave_ratio: u11,
    }

    /// PHY write data slave delay configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PhyWriteDataSlaveConfig {
        /// Write data slave delay line setting.
        #[bits(11..=19, rw)]
        wr_data_slave_delay: u9,
        /// Forces the write data slave delay line to the programmed ratio.
        #[bit(10, rw)]
        wr_data_slave_force: bool,
        /// Write data slave delay line ratio.
        #[bits(0..=9, rw)]
        wr_data_slave_ratio: u10,
    }

    /// PHY control slave configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Reg64 {
        /// Adds an extra cycle of command latency.
        #[bit(30, rw)]
        cmd_latency: bool,
        /// Selects LPDDR mode for the PHY.
        #[bit(29, rw)]
        lpddr: bool,
        /// Control slave delay line setting.
        #[bits(21..=27, rw)]
        ctrl_slave_delay: u7,
        /// Forces the control slave delay line to the programmed ratio.
        #[bit(20, rw)]
        ctrl_slave_force: bool,
        /// Control slave delay line ratio.
        #[bits(10..=19, rw)]
        ctrl_slave_ratio: u10,
        /// Selects the logic used for the control slave delay line.
        #[bit(9, rw)]
        sel_logic: bool,
        /// Inverts the output clock.
        #[bit(7, rw)]
        invert_clkout: bool,
        /// Selects burst length 2 mode.
        #[bit(1, rw)]
        bl2: bool,
    }

    /// PHY leveling delay and status configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Reg65 {
        /// Additional control slave delay line setting.
        #[bits(18..=19, rw)]
        ctrl_slave_delay: u2,
        /// Disables reset of the calibration logic.
        #[bit(17, rw)]
        dis_calib_rst: bool,
        /// Uses the trained read data eye level result.
        #[bit(16, rw)]
        use_rd_data_eye_level: bool,
        /// Uses the trained read DQS gate level result.
        #[bit(15, rw)]
        use_rd_dqs_gate_level: bool,
        /// Uses the trained write level result.
        #[bit(14, rw)]
        use_wr_level: bool,
        /// Maximum allowed difference between DLL lock values.
        #[bits(10..=13, rw)]
        dll_lock_diff: u4,
        /// Additional read leveling delay.
        #[bits(5..=9, rw)]
        rd_rl_delay: u5,
        /// Additional write leveling delay.
        #[bits(0..=4, rw)]
        wr_rl_delay: u5,
    }

    /// AXI write port priority configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct AxiPriorityWritePort {
        /// Disables page match prioritization for this port.
        #[bit(18, rw)]
        disable_page_match: bool,
        /// Disables the urgent priority signal for this port.
        #[bit(17, rw)]
        disable_urgent: bool,
        /// Disables aging-based priority increase for this port.
        #[bit(16, rw)]
        disable_aging: bool,
        /// Static priority assigned to this write port.
        #[bits(0..=9, rw)]
        pri_wr_port: u10,
    }

    /// AXI read port priority configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct AxiPriorityReadPort {
        /// Routes this port's reads through the high priority read queue.
        #[bit(19, rw)]
        enable_hpr: bool,
        /// Disables page match prioritization for this port.
        #[bit(18, rw)]
        disable_page_match: bool,
        /// Disables the urgent priority signal for this port.
        #[bit(17, rw)]
        disable_urgent: bool,
        /// Disables aging-based priority increase for this port.
        #[bit(16, rw)]
        disable_aging: bool,
        /// Static priority assigned to this read port.
        #[bits(0..=9, rw)]
        pri_rd_port_n: u10,
    }

    /// Exclusive access AXI ID configuration.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ExclusiveAccessConfig {
        /// Second AXI ID recognized for exclusive access on this port.
        #[bits(9..=17, rw)]
        access_id1_port: u9,
        /// First AXI ID recognized for exclusive access on this port.
        #[bits(0..=8, rw)]
        access_id0_port: u9,
    }

    /// DRAM type selection for LPDDR control registers.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum LpddrBit {
        /// DDR2 or DDR3 mode.
        Ddr2Ddr3 = 0,
        /// LPDDR2 mode.
        Lpddr2 = 1,
    }

    /// LPDDR2 control, register 0.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct LpddrControl0 {
        /// Margin applied when comparing MR4 refresh rate readings.
        #[bits(4..=11, rw)]
        mr4_margin: u8,
        /// Enables timing derating based on MR4 temperature readings.
        #[bit(2, rw)]
        derate_enable: bool,
        /// Enables per-bank refresh.
        #[bit(1, rw)]
        per_bank_refresh: bool,
        /// Selects LPDDR2 mode.
        #[bit(0, rw)]
        lpddr2: LpddrBit,
    }

    /// LPDDR2 control, register 1.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct LpddrControl1 {
        /// Interval between automatic MR4 reads.
        #[bits(0..=31, rw)]
        mr4_read_interval: u32,
    }

    /// LPDDR2 control, register 2.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct LpddrControl2 {
        /// tMRW - Minimum time between mode register writes.
        #[bits(12..=21, rw)]
        t_mrw: u10,
        /// Idle time required after reset, in multiples of 32 clocks.
        #[bits(4..=11, rw)]
        idle_after_reset_x32: u8,
        /// Minimum number of stable clocks required before CKE is asserted.
        #[bits(0..=3, rw)]
        min_stable_clock_x1: u4,
    }

    /// LPDDR2 control, register 3.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct LpddrControl3 {
        /// Time reserved for device ZQ initialization, in multiples of 32 clocks.
        #[bits(8..=17, rw)]
        dev_zqinit_x32: u10,
        /// Maximum device auto-initialization time, in multiples of 1024 clocks.
        #[bits(0..=7, rw)]
        max_auto_init_x1024: u8,
    }

    /// Current operating mode of the DDR controller.
    #[bitbybit::bitenum(u3, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum OperatingMode {
        /// Controller is initializing the DRAM.
        DdrcInit = 0,
        /// Controller is running normally.
        NormalOperation = 1,
        /// Controller is in power-down.
        Powerdown = 2,
        /// Controller is in self refresh.
        SelfRefresh = 3,
        /// Controller is in deep power-down.
        DeepPowerdown = 4,
        /// Deep power-down, alternate encoding 1.
        DeepPowerdownAlt1 = 5,
        /// Deep power-down, alternate encoding 2.
        DeepPowerdownAlt2 = 6,
        /// Deep power-down, alternate encoding 3.
        DeepPowerdownAlt3 = 7,
    }

    impl OperatingMode {
        /// Whether the controller is in any deep power-down state.
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

    /// Debug command acceptance state.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DebugStallBit {
        /// Commands are being accepted.
        CommandsAccepted = 0,
        /// Commands are stalled and not accepted.
        CommandsNotAccepted = 1,
    }

    /// Debug status of the controller queues and operating mode.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ModeStatus {
        /// Number of entries in the high priority read queue.
        #[bits(16..=20, r)]
        dbg_hpr_queue_depth: u5,
        /// Number of entries in the low priority read queue.
        #[bits(10..=15, r)]
        dbg_lpr_queue_depth: u6,
        /// Number of entries in the write queue.
        #[bits(4..=9, r)]
        dbg_wr_queue_depth: u6,
        /// Whether commands are currently being accepted.
        #[bit(3, r)]
        dbg_stall: DebugStallBit,
        /// Current operating mode of the controller.
        #[bits(0..=2, r)]
        operating_mode: OperatingMode,
    }
}

use regs::*;

/// DDR controller register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
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

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x2B8);

impl Registers {
    /// Create a new DDR MMIO instance for the DDR controller at address [DDRC_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(DDRC_BASE_ADDR) }
    }
}
