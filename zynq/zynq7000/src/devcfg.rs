use arbitrary_int::{u4, u5, u7};

pub const DEVCFG_BASE_ADDR: usize = 0xF8007000;

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum PlConfigAccess {
    /// Used for JTAG access
    TapController = 0,
    /// Used for PCAP or ICAP access.
    ConfigAccessPort = 1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum ConfigAccessPortSelect {
    /// Internal Configuration Access Port (ICAP), using PL or PS-based software.
    Icap = 0,
    /// Processor Configuration Access Port (PCAP), using PS-based software.
    Pcap = 1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum TimerSelect {
    _64kTimer = 0,
    _4kTimer = 1,
}

#[bitbybit::bitenum(u3, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
pub enum AesEnable {
    Disable = 0b000,
    Enable = 0b111,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum PsBootMode {
    NonSecure = 0,
    Secure = 1,
}

#[bitbybit::bitenum(u3, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
pub enum ArmDapEnable {
    Enabled = 0b111,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Control {
    #[bit(31, rw)]
    force_reset: bool,
    /// Program singal used to reset the PL. It acts at the PROG_B signal in the PL.
    #[bit(30, rw)]
    prog_b_bit: bool,
    /// Called PCFG_POR_CNT_4K by Xilinx.
    #[bit(29, rw)]
    timer_select: TimerSelect,
    /// Called XDCFG_CTRL_PCAP_PR_MASK by Xilinx.
    #[bit(27, rw)]
    access_port_select: ConfigAccessPortSelect,
    #[bit(26, rw)]
    config_access_select: PlConfigAccess,
    #[bit(25, rw)]
    pcap_rate_enable: bool,
    #[bit(24, rw)]
    multiboot_enable: bool,
    #[bit(23, rw)]
    jtag_chain_disable: bool,
    #[bit(12, rw)]
    pcfg_aes_fuse: bool,
    #[bits(9..=11, rw)]
    pcfg_aes_enable: Option<AesEnable>,
    #[bit(8, rw)]
    seu_enable: bool,
    /// Read-only because this is set and locked by BootROM.
    #[bit(7, r)]
    ps_boot_mode: PsBootMode,
    /// SPNIDEN
    #[bit(6, rw)]
    secure_non_invasive_debug_enable: bool,
    /// SPIDEN
    #[bit(5, rw)]
    secure_invasive_debug_enable: bool,
    /// NIDEN
    #[bit(4, rw)]
    non_invasive_debug_enable: bool,
    /// DBGEN
    #[bit(3, rw)]
    invasive_debug_enable: bool,
    #[bits(0..=2, rw)]
    dap_enable: Option<ArmDapEnable>,
}

/// The bits in this register and read/write, set-only, which means that only a PS_POR_B reset
/// can clear the bits.
#[bitbybit::bitfield(u32, debug)]
pub struct Lock {
    #[bit(4, rw)]
    aes_fuse: bool,
    #[bit(3, rw)]
    aes: bool,
    #[bit(2, rw)]
    seu: bool,
    /// Locks the SEC_EN bit. BootROM will set this bit.
    #[bit(1, rw)]
    sec: bool,
    /// Locks SPNIDEN, SPIDEN, NIDEN, DBGEN and DAP_EN
    #[bit(0, rw)]
    debug: bool,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum EdgeConfig {
    Falling = 0,
    Rising = 1,
}

/// Related to the full level for reads, and the empty level for writes.
#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum FifoThresholdConfig {
    OneFourth = 0b00,
    HalfEmpty = 0b01,
    ThreeFourth = 0b10,
    EmptyOrFull = 0b11,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Config {
    #[bits(10..=11, rw)]
    read_fifo_threshhold: FifoThresholdConfig,
    #[bits(8..=9, rw)]
    write_fifo_threshold: FifoThresholdConfig,
    #[bit(7, rw)]
    read_data_active_clock_edge: EdgeConfig,
    #[bit(6, rw)]
    write_data_active_clock_edge: EdgeConfig,
    #[bit(5, rw)]
    disable_src_increment: bool,
    #[bit(4, rw)]
    disable_dst_incremenet: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Interrupt {
    /// Tri-state PL IO during HIZ.
    #[bit(31, rw)]
    gts_usr_b: bool,
    #[bit(30, rw)]
    first_config_done: bool,
    #[bit(29, rw)]
    global_powerdown: bool,
    /// Tri-state PL IO during configuration.
    #[bit(28, rw)]
    gts_cfg_b: bool,
    /// PSS_CFG_RESET_B_INT
    #[bit(27, rw)]
    pl_config_reset: bool,
    #[bit(23, rw)]
    axi_write_timeout: bool,
    #[bit(22, rw)]
    axi_write_response_error: bool,
    #[bit(21, rw)]
    axi_read_timeout: bool,
    #[bit(20, rw)]
    axi_read_response_error: bool,
    #[bit(18, rw)]
    rx_overflow: bool,
    #[bit(17, rw)]
    tx_fifo_below_threshold: bool,
    #[bit(16, rw)]
    rx_fifo_above_threshold: bool,
    #[bit(15, rw)]
    dma_illegal_command: bool,
    #[bit(14, rw)]
    dma_queue_overflow: bool,
    #[bit(13, rw)]
    dma_done: bool,
    #[bit(12, rw)]
    dma_pcap_done: bool,
    #[bit(11, rw)]
    inconsistent_pcap_to_dma_transfer_len: bool,
    #[bit(6, rw)]
    hamc_error: bool,
    #[bit(5, rw)]
    seu_error: bool,
    #[bit(4, rw)]
    pl_power_loss_por_b_low: bool,
    #[bit(3, rw)]
    pl_config_controller_under_reset: bool,
    #[bit(2, rw)]
    pl_programming_done: bool,
    #[bit(1, rw)]
    positive_edge_pl_init: bool,
    #[bit(0, rw)]
    negative_edge_pl_init: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct MiscControl {
    #[bits(28..=31, r)]
    ps_version: u4,

    #[bit(8, r)]
    por_b_signal: bool,

    #[bit(4, rw)]
    loopback: bool,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum UnacknowledgedDmaTransfers {
    None = 0b00,
    One = 0b01,
    Two = 0b10,
    ThreeOrMore = 0b11,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Status {
    #[bit(31, rw)]
    dma_command_queue_full: bool,
    #[bit(30, rw)]
    dma_command_queue_empty: bool,
    #[bits(28..=29, rw)]
    unacknowledged_dma_transfers: UnacknowledgedDmaTransfers,
    #[bits(20..=24, rw)]
    rx_fifo_level: u5,
    #[bits(12..=18, rw)]
    tx_fifo_level: u7,
    #[bit(11, rw)]
    gts_usr_b: bool,
    #[bit(10, rw)]
    first_config_done: bool,
    #[bit(9, rw)]
    global_powerdown: bool,
    #[bit(8, rw)]
    gts_cfg_b: bool,
    #[bit(7, rw)]
    secure_lockdown: bool,
    #[bit(6, rw)]
    illegal_apb_access: bool,
    /// Active low reset bit.
    #[bit(5, rw)]
    pl_reset_n: bool,
    #[bit(4, rw)]
    pcfg_init: bool,
    #[bit(3, rw)]
    efuse_bbram_aes_key_disabled: bool,
    #[bit(2, rw)]
    efuse_sec_enable: bool,
    #[bit(1, rw)]
    efuse_jtag_disabled: bool,
}

/// Device configuration register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    control: Control,
    lock: Lock,
    config: Config,
    /// Interrupt is cleared by writing to this register.
    interrupt_status: Interrupt,
    /// Bits can be set to one to mask the interrupts.
    interrupt_mask: Interrupt,
    status: Status,
    dma_source_addr: u32,
    dma_dest_addr: u32,
    dma_source_len: u32,
    dma_dest_len: u32,
    _reserved0: u32,
    multiboot_addr: u32,
    _reserved1: u32,
    unlock_control: u32,
    _reserved2: [u32; 0x12],
    misc_control: MiscControl,

    _reserved3: [u32; 0x1F],

    // Included here but not exposed to avoid providing multiple references to the same peripheral.
    // Exposed in [crate::xadc].
    _xadc: crate::xadc::Registers,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x11C);

impl Registers {
    /// Create a new DevCfg MMIO instance for for device configuration peripheral at address
    /// [DEVCFG_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(DEVCFG_BASE_ADDR) }
    }
}
