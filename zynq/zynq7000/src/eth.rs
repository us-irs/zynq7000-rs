//! # Gigabit Ethernet Module (GEM) register module.
use arbitrary_int::{u2, u5};

pub const GEM_0_BASE_ADDR: usize = 0xE000_B000;
pub const GEM_1_BASE_ADDR: usize = 0xE000_C000;

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct NetworkControl {
    #[bit(18, w)]
    flush_next_rx_dpram_pkt: bool,
    #[bit(17, w)]
    tx_pfc_pri_pause_frame: bool,
    #[bit(16, w)]
    enable_pfc_pri_pause_rx: bool,
    #[bit(12, w)]
    zero_pause_tx: bool,
    #[bit(11, w)]
    pause_tx: bool,
    #[bit(10, w)]
    stop_tx: bool,
    #[bit(9, w)]
    start_tx: bool,
    #[bit(8, rw)]
    back_pressure: bool,
    #[bit(7, rw)]
    statistics_write_enable: bool,
    #[bit(6, w)]
    increment_statistics: bool,
    #[bit(5, w)]
    clear_statistics: bool,
    #[bit(4, rw)]
    management_port_enable: bool,
    #[bit(3, rw)]
    tx_enable: bool,
    #[bit(2, rw)]
    rx_enable: bool,
    #[bit(1, rw)]
    loopback_local: bool,
}

/// The speed mode selects between 10 Mbps and 100 Mbps if the Gigabit enable bit is cleared.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum SpeedMode {
    Low10Mbps = 0,
    High100Mbps = 1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum PcsSelect {
    GmiiMii = 0,
    Tbi = 1,
}

#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum MdcClockDivisor {
    Div8 = 0,
    Div16 = 1,
    Div32 = 2,
    Div48 = 3,
    Div64 = 4,
    Div96 = 5,
    Div128 = 6,
    Div224 = 7,
}

impl MdcClockDivisor {
    pub fn divisor(&self) -> usize {
        match self {
            MdcClockDivisor::Div8 => 8,
            MdcClockDivisor::Div16 => 16,
            MdcClockDivisor::Div32 => 32,
            MdcClockDivisor::Div48 => 48,
            MdcClockDivisor::Div64 => 64,
            MdcClockDivisor::Div96 => 96,
            MdcClockDivisor::Div128 => 128,
            MdcClockDivisor::Div224 => 224,
        }
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct NetworkConfig {
    #[bit(30, rw)]
    ignore_ipg_rx_error: bool,
    #[bit(29, rw)]
    allow_bad_preamble: bool,
    #[bit(28, rw)]
    ipg_stretch_enable: bool,
    #[bit(27, rw)]
    sgmii_enable: bool,
    #[bit(26, rw)]
    ignore_rx_fcs: bool,
    #[bit(25, rw)]
    half_duplex_rx_enable: bool,
    #[bit(24, rw)]
    rx_checksum_enable: bool,
    #[bit(23, rw)]
    disable_copy_pause_frames: bool,
    /// Zynq defines this as 0b00 for 32-bit AMBA AHB data bus width.
    #[bits(21..=22, r)]
    dbus_width: u2,
    #[bits(18..=20, rw)]
    mdc_clk_div: MdcClockDivisor,
    #[bit(17, rw)]
    fcs_removal: bool,
    #[bit(16, rw)]
    length_field_error_discard: bool,
    #[bits(14..=15, rw)]
    rx_buf_offset: u2,
    #[bit(13, rw)]
    pause_enable: bool,
    #[bit(12, rw)]
    retry_test_enable: bool,
    #[bit(11, rw)]
    pcs_select: PcsSelect,
    #[bit(10, rw)]
    gigabit_enable: bool,
    #[bit(9, rw)]
    ext_addr_match_enable: bool,
    #[bit(8, rw)]
    rx_enable_1536: bool,
    #[bit(7, rw)]
    unicast_hash_enable: bool,
    #[bit(6, rw)]
    multicast_hash_enable: bool,
    #[bit(5, rw)]
    no_broadcast: bool,
    #[bit(4, rw)]
    copy_all_frames: bool,
    #[bit(2, rw)]
    discard_non_vlan: bool,
    #[bit(1, rw)]
    full_duplex: bool,
    #[bit(0, rw)]
    speed_mode: SpeedMode,
}

/// PHY management status information.
#[bitbybit::bitfield(u32, debug)]
pub struct NetworkStatus {
    #[bit(6, r)]
    pfc_pri_pause_neg: bool,
    #[bit(5, r)]
    pcs_autoneg_pause_tx_res: bool,
    #[bit(4, r)]
    pcs_autoneg_pause_rx_res: bool,
    #[bit(3, r)]
    pcs_autoneg_dup_res: bool,
    #[bit(2, r)]
    phy_mgmt_idle: bool,
    #[bit(1, r)]
    mdio_in: bool,
    #[bit(0, r)]
    pcs_link_state: bool,
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum BurstLength {
    Single,
    #[default]
    Incr4,
    Incr8,
    Incr16,
}

impl BurstLength {
    pub const fn reg_value(&self) -> u5 {
        u5::new(match self {
            BurstLength::Single => 0b1,
            BurstLength::Incr4 => 0b100,
            BurstLength::Incr8 => 0b1000,
            BurstLength::Incr16 => 0b10000,
        })
    }
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum AhbEndianess {
    Little = 0,
    Big = 1,
}

#[derive(Debug, PartialEq, Eq)]
pub struct DmaRxBufSize(u8);

impl DmaRxBufSize {
    pub const fn new_with_raw_value(size: u8) -> Self {
        Self(size)
    }

    pub const fn new(size: u8) -> Option<Self> {
        if size == 0 {
            return None;
        }
        Some(Self(size))
    }

    pub const fn raw_value(&self) -> u8 {
        self.0
    }

    pub const fn size_in_bytes(&self) -> usize {
        self.0 as usize * 64
    }

    pub const fn reg_value(&self) -> u8 {
        self.0
    }
}

#[bitbybit::bitfield(u32, debug)]
pub struct DmaConfig {
    #[bit(24, rw)]
    discard_when_ahb_full: bool,
    /// DMA receive buffer size in AHB system memory.
    #[bits(16..=23, rw)]
    dma_rx_ahb_buf_size_sel: DmaRxBufSize,
    /// Checksum offloading for TX.
    #[bit(11, rw)]
    chksum_offload_enable: bool,
    /// Select size for packet buffer SRAM. Should be set to 1 to use the full configurable address
    /// space of 4 kB for the packet buffer.
    #[bit(10, rw)]
    tx_packet_buf_size_sel: bool,
    /// Select size for packet buffer SRAM. Should be set to 0b11 to use the full configurable
    /// address space of 8 kB for the packet buffer.
    #[bits(8..=9, rw)]
    rx_packet_buf_size_sel: u2,
    /// Default value is 0x1 (big endian). It is recommended to set this to little endian (0x0).
    #[bit(7, rw)]
    endian_swap_packet_data: AhbEndianess,
    // Default value is 0x0 (little endian)
    #[bit(6, rw)]
    endian_swap_mgmt_descriptor: AhbEndianess,
    #[bits(0..=4, rw)]
    burst_length: u5,
}

#[bitbybit::bitfield(u32, debug)]
pub struct TxStatus {
    #[bit(8, rw)]
    hresp_not_ok: bool,
    #[bit(7, rw)]
    late_collision: bool,
    /// This bit should never be se because the DMA is configured for packet buffer mode.
    #[bit(6, rw)]
    underrun: bool,
    #[bit(5, rw)]
    complete: bool,
    #[bit(4, rw)]
    frame_corruption_ahb_error: bool,
    /// Its called "tx_go" inside the Zynq 7000 documentation, but I think this is just a
    /// TX active bit.
    #[bit(3, r)]
    go: bool,
    #[bit(2, rw)]
    retry_limit_reached: bool,
    #[bit(1, rw)]
    collision: bool,
    #[bit(0, rw)]
    read_when_used: bool,
}

impl TxStatus {
    pub fn new_clear_all() -> Self {
        Self::new_with_raw_value(0xFF)
    }
}

#[bitbybit::bitfield(u32, debug)]
pub struct RxStatus {
    #[bit(3, rw)]
    hresp_not_ok: bool,
    #[bit(2, rw)]
    overrun: bool,
    #[bit(1, rw)]
    frame_received: bool,
    #[bit(0, rw)]
    buf_not_available: bool,
}

impl RxStatus {
    pub fn new_clear_all() -> Self {
        Self::new_with_raw_value(0xF)
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct InterruptStatus {
    #[bit(26, rw)]
    tsu_sec_incr: bool,
    /// Marked N/A in datasheet.
    #[bit(17, rw)]
    partner_pg_rx: bool,
    /// Marked N/A in datasheet.
    #[bit(16, rw)]
    auto_negotiation_complete: bool,
    #[bit(15, rw)]
    external_interrupt: bool,
    #[bit(14, rw)]
    pause_transmitted: bool,
    #[bit(13, rw)]
    pause_time_zero: bool,
    #[bit(12, rw)]
    pause_with_non_zero_quantum: bool,
    #[bit(11, rw)]
    hresp_not_ok: bool,
    #[bit(10, rw)]
    rx_overrun: bool,
    /// Marked N/A in datasheet.
    #[bit(9, rw)]
    link_changed: bool,
    #[bit(7, rw)]
    frame_sent: bool,
    /// Cleared on read.
    #[bit(6, r)]
    tx_frame_corruption_ahb_error: bool,
    #[bit(5, rw)]
    tx_retry_limit_reached_or_late_collision: bool,
    #[bit(3, rw)]
    tx_descr_read_when_used: bool,
    #[bit(2, rw)]
    rx_descr_read_when_used: bool,
    #[bit(1, rw)]
    frame_received: bool,
    #[bit(0, rw)]
    mgmt_frame_sent: bool,
}

#[bitbybit::bitfield(u32, default = 0x00)]
#[derive(Debug)]
pub struct InterruptControl {
    #[bit(26, w)]
    tsu_sec_incr: bool,
    /// Marked N/A in datasheet. Probably because external PHYs are used.
    #[bit(17, w)]
    partner_pg_rx: bool,
    /// Marked N/A in datasheet. Probably because external PHYs are used.
    #[bit(16, w)]
    auto_negotiation_complete: bool,
    #[bit(15, w)]
    external_interrupt: bool,
    #[bit(14, w)]
    pause_transmitted: bool,
    #[bit(13, w)]
    pause_time_zero: bool,
    #[bit(12, w)]
    pause_with_non_zero_quantum: bool,
    #[bit(11, w)]
    hresp_not_ok: bool,
    #[bit(10, w)]
    rx_overrun: bool,
    /// Marked N/A in datasheet. Probably because external PHYs are used.
    #[bit(9, w)]
    link_changed: bool,
    #[bit(7, w)]
    frame_sent: bool,
    #[bit(6, w)]
    tx_frame_corruption_ahb_error: bool,
    #[bit(5, w)]
    tx_retry_limit_reached_or_late_collision: bool,
    #[bit(3, w)]
    tx_descr_read_when_used: bool,
    #[bit(2, w)]
    rx_descr_read_when_used: bool,
    #[bit(1, w)]
    frame_received: bool,
    #[bit(0, w)]
    mgmt_frame_sent: bool,
}

impl InterruptControl {
    pub fn new_clear_all() -> Self {
        Self::new_with_raw_value(0xFFFF_FFFF)
    }
}

#[bitbybit::bitenum(u2, exhaustive = false)]
#[derive(Debug)]
pub enum PhyOperation {
    Read = 0b10,
    Write = 0b01,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct PhyMaintenance {
    /// Must be 1 for Clause 22 operations.
    #[bit(30, rw)]
    clause_22: bool,
    #[bits(28..=29, rw)]
    op: Option<PhyOperation>,
    #[bits(23..=27, rw)]
    phy_addr: u5,
    #[bits(18..=22, rw)]
    reg_addr: u5,
    #[bits(16..=17, rw)]
    must_be_0b10: u2,
    #[bits(0..=15, rw)]
    data: u16,
}

#[bitbybit::bitfield(u32, debug)]
pub struct PauseQuantum {
    #[bits(0..=15, rw)]
    value: u16,
}

#[bitbybit::bitfield(u32, debug)]
pub struct MatchRegister {
    #[bit(31, rw)]
    copy_enable: bool,
    #[bits(0..=15, rw)]
    type_id: u16,
}

/// Gigabit Ethernet Controller (GEM) register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Ethernet {
    net_ctrl: NetworkControl,
    net_cfg: NetworkConfig,
    #[mmio(PureRead)]
    net_status: NetworkStatus,
    _reserved0: u32,
    dma_cfg: DmaConfig,
    tx_status: TxStatus,
    rx_buf_queue_base_addr: u32,
    tx_buf_queue_base_addr: u32,
    rx_status: RxStatus,
    interrupt_status: InterruptStatus,
    interrupt_enable: InterruptControl,
    interrupt_disable: InterruptControl,
    interrupt_mask: InterruptStatus,
    phy_maintenance: PhyMaintenance,
    #[mmio(PureRead)]
    rx_pause_quantum: PauseQuantum,
    tx_pause_quantum: PauseQuantum,
    _reserved1: [u32; 0x10],
    hash_low: u32,
    hash_high: u32,
    addr1_low: u32,
    addr1_high: u32,
    addr2_low: u32,
    addr2_high: u32,
    addr3_low: u32,
    addr3_high: u32,
    addr4_low: u32,
    addr4_high: u32,
    match_reg: [MatchRegister; 4],
    wake_on_lan: u32,
    ipg_stretch: u32,
    stacked_vlan: u32,
    tx_pfc: u32,
    addr1_mask_low: u32,
    addr1_mask_high: u32,
    _reserved2: [u32; 0x0B],
    /// Should be 0x20118.
    #[mmio(PureRead)]
    module_id: u32,
    #[mmio(Inner)]
    statistics: Statistics,
    _reserved3: [u32; 0x34],
    #[mmio(PureRead)]
    design_cfg_2: u32,
    #[mmio(PureRead)]
    design_cfg_3: u32,
    #[mmio(PureRead)]
    design_cfg_4: u32,
    #[mmio(PureRead)]
    design_cfg_5: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Ethernet>(), 0x294);

/// Ethernet statistics registers
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Statistics {
    #[mmio(PureRead)]
    tx_octets_low: u32,
    #[mmio(PureRead)]
    tx_octets_high: u32,
    #[mmio(PureRead)]
    tx_count: u32,
    #[mmio(PureRead)]
    tx_broadcast: u32,
    #[mmio(PureRead)]
    tx_multicast: u32,
    #[mmio(PureRead)]
    tx_pause: u32,
    #[mmio(PureRead)]
    tx_64_bits: u32,
    #[mmio(PureRead)]
    tx_65_to_127_bits: u32,
    #[mmio(PureRead)]
    tx_128_to_255_bits: u32,
    #[mmio(PureRead)]
    tx_256_to_511_bits: u32,
    #[mmio(PureRead)]
    tx_512_to_1023_bits: u32,
    #[mmio(PureRead)]
    tx_1024_to_1518_bits: u32,
    _reserved0: u32,
    #[mmio(PureRead)]
    tx_underruns: u32,
    #[mmio(PureRead)]
    single_collision_frames: u32,
    #[mmio(PureRead)]
    multi_collision_frames: u32,
    #[mmio(PureRead)]
    excessive_collisions: u32,
    #[mmio(PureRead)]
    late_collisions: u32,
    #[mmio(PureRead)]
    deferred_tx: u32,
    #[mmio(PureRead)]
    carrier_sense_errors: u32,
    #[mmio(PureRead)]
    rx_octets_low: u32,
    #[mmio(PureRead)]
    rx_octets_high: u32,
    #[mmio(PureRead)]
    rx_count: u32,
    #[mmio(PureRead)]
    rx_broadcast: u32,
    #[mmio(PureRead)]
    rx_multicast: u32,
    #[mmio(PureRead)]
    rx_pause: u32,
    #[mmio(PureRead)]
    rx_64_bits: u32,
    #[mmio(PureRead)]
    rx_65_to_127_bits: u32,
    #[mmio(PureRead)]
    rx_128_to_255_bits: u32,
    #[mmio(PureRead)]
    rx_256_to_511_bits: u32,
    #[mmio(PureRead)]
    rx_512_to_1023_bits: u32,
    #[mmio(PureRead)]
    rx_1024_to_1518_bits: u32,
    _reserved1: u32,
    #[mmio(PureRead)]
    rx_undersize: u32,
    #[mmio(PureRead)]
    rx_oversize: u32,
    #[mmio(PureRead)]
    rx_jabber: u32,
    #[mmio(PureRead)]
    rx_frame_check_sequence_errors: u32,
    #[mmio(PureRead)]
    rx_length_field_errors: u32,
    #[mmio(PureRead)]
    rx_symbol_errors: u32,
    #[mmio(PureRead)]
    rx_alignment_errors: u32,
    #[mmio(PureRead)]
    rx_resource_errors: u32,
    #[mmio(PureRead)]
    rx_overrun_errors: u32,
    #[mmio(PureRead)]
    rx_ip_header_checksum_errors: u32,
    #[mmio(PureRead)]
    rx_tcp_checksum_errors: u32,
    #[mmio(PureRead)]
    rx_udp_checksum_errors: u32,
}

impl Ethernet {
    /// Create a new Gigabit Ethernet MMIO instance for GEM 0 at address [GEM_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioEthernet<'static> {
        unsafe { Self::new_mmio_at(GEM_0_BASE_ADDR) }
    }

    /// Create a new Gigabit Ethernet MMIO instance for GEM 1 at address [GEM_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioEthernet<'static> {
        unsafe { Self::new_mmio_at(GEM_1_BASE_ADDR) }
    }
}
