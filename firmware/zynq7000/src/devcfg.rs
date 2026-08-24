use arbitrary_int::{u4, u5, u7};

/// Base address of the device configuration (DevCfg) register block.
pub const DEVCFG_BASE_ADDR: usize = 0xF8007000;

/// Selects which port is used to access the PL configuration logic.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PlConfigAccess {
    /// Used for JTAG access
    TapController = 0,
    /// Used for PCAP or ICAP access.
    ConfigAccessPort = 1,
}

/// Selects which configuration access port is used.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigAccessPortSelect {
    /// Internal Configuration Access Port (ICAP), using PL or PS-based software.
    Icap = 0,
    /// Processor Configuration Access Port (PCAP), using PS-based software.
    Pcap = 1,
}

/// Selects the source of the PCAP configuration timeout counter.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimerSelect {
    /// 64k clock cycle timer.
    _64kTimer = 0,
    /// 4k clock cycle timer.
    _4kTimer = 1,
}

/// Enables AES decryption of the PL bitstream.
#[bitbybit::bitenum(u3, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AesEnable {
    /// AES decryption disabled.
    Disable = 0b000,
    /// AES decryption enabled.
    Enable = 0b111,
}

/// Boot mode latched by the BootROM, indicating whether secure boot was used.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PsBootMode {
    /// Non-secure boot.
    NonSecure = 0,
    /// Secure boot.
    Secure = 1,
}

/// Enables the ARM Debug Access Port.
#[bitbybit::bitenum(u3, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ArmDapEnable {
    /// DAP is enabled.
    Enabled = 0b111,
}

/// Control register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct Control {
    /// Resets the PL and the DevCfg interface, self-clearing.
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
    /// Selects between JTAG and PCAP/ICAP access to the PL configuration logic.
    #[bit(26, rw)]
    config_access_select: PlConfigAccess,
    /// Enables the PCAP-to-PL bitstream transfer at half the PCAP clock rate.
    #[bit(25, rw)]
    pcap_rate_enable: bool,
    /// Enables the multiboot fallback mechanism on a configuration failure.
    #[bit(24, rw)]
    multiboot_enable: bool,
    /// Disables the PS JTAG chain, isolating the PL from the JTAG scan chain.
    #[bit(23, rw)]
    jtag_chain_disable: bool,
    /// Selects the AES key source between the BBRAM and the eFUSE array.
    #[bit(12, rw)]
    pcfg_aes_fuse: bool,
    /// Enables AES decryption of the PL bitstream.
    #[bits(9..=11, rw)]
    pcfg_aes_enable: Option<AesEnable>,
    /// Enables single event upset (SEU) detection on the PL configuration memory.
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
    /// Enables the ARM Debug Access Port.
    #[bits(0..=2, rw)]
    dap_enable: Option<ArmDapEnable>,
}

/// The bits in this register and read/write, set-only, which means that only a PS_POR_B reset
/// can clear the bits.
#[bitbybit::bitfield(u32, debug, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
pub struct Lock {
    /// Locks the PCFG_AES_FUSE bit.
    #[bit(4, rw)]
    aes_fuse: bool,
    /// Locks the PCFG_AES_EN field.
    #[bit(3, rw)]
    aes: bool,
    /// Locks the SEU_EN bit.
    #[bit(2, rw)]
    seu: bool,
    /// Locks the SEC_EN bit. BootROM will set this bit.
    #[bit(1, rw)]
    sec: bool,
    /// Locks SPNIDEN, SPIDEN, NIDEN, DBGEN and DAP_EN
    #[bit(0, rw)]
    debug: bool,
}

/// Active clock edge used to latch or drive DMA data.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EdgeConfig {
    /// Falling clock edge.
    Falling = 0,
    /// Rising clock edge.
    Rising = 1,
}

/// Related to the full level for reads, and the empty level for writes.
#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoThresholdConfig {
    /// One quarter.
    OneFourth = 0b00,
    /// Half.
    HalfEmpty = 0b01,
    /// Three quarters.
    ThreeFourth = 0b10,
    /// Completely empty or completely full.
    EmptyOrFull = 0b11,
}

/// DMA configuration register.
#[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
pub struct Config {
    /// FIFO level at which the read DMA is considered full.
    #[bits(10..=11, rw)]
    read_fifo_threshhold: FifoThresholdConfig,
    /// FIFO level at which the write DMA is considered empty.
    #[bits(8..=9, rw)]
    write_fifo_threshold: FifoThresholdConfig,
    /// Active clock edge for read DMA data.
    #[bit(7, rw)]
    read_data_active_clock_edge: EdgeConfig,
    /// Active clock edge for write DMA data.
    #[bit(6, rw)]
    write_data_active_clock_edge: EdgeConfig,
    /// Keeps the DMA source address fixed instead of incrementing it.
    #[bit(5, rw)]
    disable_src_increment: bool,
    /// Keeps the DMA destination address fixed instead of incrementing it.
    #[bit(4, rw)]
    disable_dst_incremenet: bool,
}

/// Interrupt status and mask register layout, shared by the interrupt status and interrupt
/// mask registers.
#[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
pub struct Interrupt {
    /// Tri-state PL IO during HIZ.
    #[bit(31, rw)]
    gts_usr_b: bool,
    /// The PL configuration completed for the first time since power-up.
    #[bit(30, rw)]
    first_config_done: bool,
    /// The PL has been powered down.
    #[bit(29, rw)]
    global_powerdown: bool,
    /// Tri-state PL IO during configuration.
    #[bit(28, rw)]
    gts_cfg_b: bool,
    /// PSS_CFG_RESET_B_INT
    #[bit(27, rw)]
    pl_config_reset: bool,
    /// An AXI write transaction to the PCAP interface timed out.
    #[bit(23, rw)]
    axi_write_timeout: bool,
    /// An AXI write transaction to the PCAP interface received an error response.
    #[bit(22, rw)]
    axi_write_response_error: bool,
    /// An AXI read transaction to the PCAP interface timed out.
    #[bit(21, rw)]
    axi_read_timeout: bool,
    /// An AXI read transaction to the PCAP interface received an error response.
    #[bit(20, rw)]
    axi_read_response_error: bool,
    /// The receive FIFO overflowed.
    #[bit(18, rw)]
    rx_overflow: bool,
    /// The transmit FIFO level dropped below the configured threshold.
    #[bit(17, rw)]
    tx_fifo_below_threshold: bool,
    /// The receive FIFO level rose above the configured threshold.
    #[bit(16, rw)]
    rx_fifo_above_threshold: bool,
    /// An illegal DMA command was issued.
    #[bit(15, rw)]
    dma_illegal_command: bool,
    /// The DMA command queue overflowed.
    #[bit(14, rw)]
    dma_queue_overflow: bool,
    /// A DMA transfer completed.
    #[bit(13, rw)]
    dma_done: bool,
    /// The PCAP loopback DMA transfer completed.
    #[bit(12, rw)]
    dma_pcap_done: bool,
    /// The PCAP and DMA transfer lengths do not match.
    #[bit(11, rw)]
    inconsistent_pcap_to_dma_transfer_len: bool,
    /// A HMAC authentication error occurred.
    #[bit(6, rw)]
    hamc_error: bool,
    /// A single event upset (SEU) was detected in the PL configuration memory.
    #[bit(5, rw)]
    seu_error: bool,
    /// The PL power supply dropped and PL POR_B went low.
    #[bit(4, rw)]
    pl_power_loss_por_b_low: bool,
    /// The PL configuration controller is held under reset.
    #[bit(3, rw)]
    pl_config_controller_under_reset: bool,
    /// PL programming completed, DONE pin is high.
    #[bit(2, rw)]
    pl_programming_done: bool,
    /// Positive edge detected on the PL INIT signal.
    #[bit(1, rw)]
    positive_edge_pl_init: bool,
    /// Negative edge detected on the PL INIT signal.
    #[bit(0, rw)]
    negative_edge_pl_init: bool,
}

/// Miscellaneous control register.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct MiscControl {
    /// Silicon version of the processing system.
    #[bits(28..=31, r)]
    ps_version: u4,

    /// State of the power-on-reset signal.
    #[bit(8, r)]
    por_b_signal: bool,

    /// Enables PCAP loopback mode, routing PCAP write data back to the read path.
    #[bit(4, rw)]
    loopback: bool,
}

/// Number of DMA transfers still awaiting acknowledgment.
#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UnacknowledgedDmaTransfers {
    /// No unacknowledged transfers.
    None = 0b00,
    /// One unacknowledged transfer.
    One = 0b01,
    /// Two unacknowledged transfers.
    Two = 0b10,
    /// Three or more unacknowledged transfers.
    ThreeOrMore = 0b11,
}

/// Status register.
#[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
pub struct Status {
    /// The DMA command queue is full.
    #[bit(31, rw)]
    dma_command_queue_full: bool,
    /// The DMA command queue is empty.
    #[bit(30, rw)]
    dma_command_queue_empty: bool,
    /// Number of DMA transfers still awaiting acknowledgment.
    #[bits(28..=29, rw)]
    unacknowledged_dma_transfers: UnacknowledgedDmaTransfers,
    /// Current level of the receive FIFO.
    #[bits(20..=24, rw)]
    rx_fifo_level: u5,
    /// Current level of the transmit FIFO.
    #[bits(12..=18, rw)]
    tx_fifo_level: u7,
    /// State of the GTS_USR_B pin.
    #[bit(11, rw)]
    gts_usr_b: bool,
    /// The PL configuration completed for the first time since power-up.
    #[bit(10, rw)]
    first_config_done: bool,
    /// The PL has been powered down.
    #[bit(9, rw)]
    global_powerdown: bool,
    /// State of the GTS_CFG_B pin.
    #[bit(8, rw)]
    gts_cfg_b: bool,
    /// The PL configuration is locked down for security.
    #[bit(7, rw)]
    secure_lockdown: bool,
    /// An illegal APB access to the DevCfg interface was attempted.
    #[bit(6, rw)]
    illegal_apb_access: bool,
    /// Active low reset bit.
    #[bit(5, rw)]
    pl_reset_n: bool,
    /// State of the PCFG_INIT pin.
    #[bit(4, rw)]
    pcfg_init: bool,
    /// The eFUSE-based BBRAM AES key is disabled.
    #[bit(3, rw)]
    efuse_bbram_aes_key_disabled: bool,
    /// Secure boot is enabled via eFUSE.
    #[bit(2, rw)]
    efuse_sec_enable: bool,
    /// JTAG access is disabled via eFUSE.
    #[bit(1, rw)]
    efuse_jtag_disabled: bool,
}

/// Device configuration register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Control register.
    control: Control,
    /// Lock register.
    lock: Lock,
    /// DMA configuration register.
    config: Config,
    /// Interrupt is cleared by writing to this register.
    interrupt_status: Interrupt,
    /// Bits can be set to one to mask the interrupts.
    interrupt_mask: Interrupt,
    /// Status register.
    status: Status,
    /// DMA source address.
    dma_source_addr: u32,
    /// DMA destination address.
    dma_dest_addr: u32,
    /// DMA source transfer length in words.
    dma_source_len: u32,
    /// DMA destination transfer length in words.
    dma_dest_len: u32,
    _reserved0: u32,
    /// Address of the next bitstream image used on a multiboot fallback.
    multiboot_addr: u32,
    _reserved1: u32,
    /// Unlocks write access to the control register when written with the correct code.
    unlock_control: u32,
    _reserved2: [u32; 0x12],
    /// Miscellaneous control register.
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
