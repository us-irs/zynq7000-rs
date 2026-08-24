//! # Gigabit Ethernet Module (GEM) register module.
use arbitrary_int::{u2, u5};

/// Base address of the GEM 0 register block.
pub const GEM_0_BASE_ADDR: usize = 0xE000_B000;
/// Base address of the GEM 1 register block.
pub const GEM_1_BASE_ADDR: usize = 0xE000_C000;

/// Network control register.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct NetworkControl {
    /// Flushes the next unread frame from the RX FIFO.
    #[bit(18, w)]
    flush_next_rx_dpram_pkt: bool,
    /// Transmits a priority-based flow control pause frame.
    #[bit(17, w)]
    tx_pfc_pri_pause_frame: bool,
    /// Enables reception of priority-based flow control pause frames.
    #[bit(16, w)]
    enable_pfc_pri_pause_rx: bool,
    /// Transmits a pause frame with a zero pause quantum.
    #[bit(12, w)]
    zero_pause_tx: bool,
    /// Transmits a pause frame.
    #[bit(11, w)]
    pause_tx: bool,
    /// Stops transmission after the current frame completes.
    #[bit(10, w)]
    stop_tx: bool,
    /// Starts transmission of a queued frame.
    #[bit(9, w)]
    start_tx: bool,
    /// Forces collisions on all received frames in half duplex mode.
    #[bit(8, rw)]
    back_pressure: bool,
    /// Enables writes to the statistics registers.
    #[bit(7, rw)]
    statistics_write_enable: bool,
    /// Increments all statistics registers by one, for test purposes.
    #[bit(6, w)]
    increment_statistics: bool,
    /// Clears all statistics registers.
    #[bit(5, w)]
    clear_statistics: bool,
    /// Enables the management port (MDIO).
    #[bit(4, rw)]
    management_port_enable: bool,
    /// Enables the transmitter.
    #[bit(3, rw)]
    tx_enable: bool,
    /// Enables the receiver.
    #[bit(2, rw)]
    rx_enable: bool,
    /// Enables local loopback mode.
    #[bit(1, rw)]
    loopback_local: bool,
}

/// The speed mode selects between 10 Mbps and 100 Mbps if the Gigabit enable bit is cleared.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq)]
pub enum SpeedMode {
    /// 10 Mbps.
    Low10Mbps = 0,
    /// 100 Mbps.
    High100Mbps = 1,
}

/// Selects the PCS/PMA interface used by the MAC.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PcsSelect {
    /// GMII or MII interface.
    GmiiMii = 0,
    /// TBI interface.
    Tbi = 1,
}

/// Divisor applied to the CPU clock to generate the MDC clock.
#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MdcClockDivisor {
    /// Divide by 8.
    Div8 = 0,
    /// Divide by 16.
    Div16 = 1,
    /// Divide by 32.
    Div32 = 2,
    /// Divide by 48.
    Div48 = 3,
    /// Divide by 64.
    Div64 = 4,
    /// Divide by 96.
    Div96 = 5,
    /// Divide by 128.
    Div128 = 6,
    /// Divide by 224.
    Div224 = 7,
}

impl MdcClockDivisor {
    /// Numeric divisor value.
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

/// Network configuration register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    forbid_overlaps,
    defmt_fields(feature = "defmt")
)]
pub struct NetworkConfig {
    /// Ignores IPG receive errors.
    #[bit(30, rw)]
    ignore_ipg_rx_error: bool,
    /// Accepts frames whose receive preamble is not all ones.
    #[bit(29, rw)]
    allow_bad_preamble: bool,
    /// Stretches the inter-packet gap based on the previous frame length.
    #[bit(28, rw)]
    ipg_stretch_enable: bool,
    /// Enables SGMII mode.
    #[bit(27, rw)]
    sgmii_enable: bool,
    /// Does not reject frames with an FCS error.
    #[bit(26, rw)]
    ignore_rx_fcs: bool,
    /// Enables reception while transmitting in half duplex mode.
    #[bit(25, rw)]
    half_duplex_rx_enable: bool,
    /// Enables IP, TCP and UDP checksum checking on received frames.
    #[bit(24, rw)]
    rx_checksum_enable: bool,
    /// Prevents pause frames from being copied to memory.
    #[bit(23, rw)]
    disable_copy_pause_frames: bool,
    /// Zynq defines this as 0b00 for 32-bit AMBA AHB data bus width.
    #[bits(21..=22, r)]
    dbus_width: u2,
    /// MDC clock divisor, see [MdcClockDivisor].
    #[bits(18..=20, rw)]
    mdc_clk_div: MdcClockDivisor,
    /// Removes the FCS from received frames before copying to memory.
    #[bit(17, rw)]
    fcs_removal: bool,
    /// Discards frames whose length field does not match the actual frame length.
    #[bit(16, rw)]
    length_field_error_discard: bool,
    /// Number of bytes by which the RX buffer start address is offset.
    #[bits(14..=15, rw)]
    rx_buf_offset: u2,
    /// Enables pause frame detection and transmit pausing.
    #[bit(13, rw)]
    pause_enable: bool,
    /// Reduces the retry limit, used to enable collision testing.
    #[bit(12, rw)]
    retry_test_enable: bool,
    /// Selects the PCS/PMA interface, see [PcsSelect].
    #[bit(11, rw)]
    pcs_select: PcsSelect,
    /// Enables gigabit mode.
    #[bit(10, rw)]
    gigabit_enable: bool,
    /// Enables the type ID match registers as an additional address match condition.
    #[bit(9, rw)]
    ext_addr_match_enable: bool,
    /// Allows reception of frames up to 1536 bytes.
    #[bit(8, rw)]
    rx_enable_1536: bool,
    /// Enables hash-based unicast address filtering.
    #[bit(7, rw)]
    unicast_hash_enable: bool,
    /// Enables hash-based multicast address filtering.
    #[bit(6, rw)]
    multicast_hash_enable: bool,
    /// Rejects all broadcast frames.
    #[bit(5, rw)]
    no_broadcast: bool,
    /// Copies all frames regardless of address matching (promiscuous mode).
    #[bit(4, rw)]
    copy_all_frames: bool,
    /// Discards frames that are not VLAN tagged.
    #[bit(2, rw)]
    discard_non_vlan: bool,
    /// Enables full duplex mode.
    #[bit(1, rw)]
    full_duplex: bool,
    /// Selects the link speed, see [SpeedMode].
    #[bit(0, rw)]
    speed_mode: SpeedMode,
}

/// PHY management status information.
#[bitbybit::bitfield(u32, debug, forbid_overlaps, defmt_fields(feature = "defmt"))]
pub struct NetworkStatus {
    /// Priority-based flow control pause was negotiated.
    #[bit(6, r)]
    pfc_pri_pause_neg: bool,
    /// PCS auto-negotiation result for transmit pause.
    #[bit(5, r)]
    pcs_autoneg_pause_tx_res: bool,
    /// PCS auto-negotiation result for receive pause.
    #[bit(4, r)]
    pcs_autoneg_pause_rx_res: bool,
    /// PCS auto-negotiation duplex result.
    #[bit(3, r)]
    pcs_autoneg_dup_res: bool,
    /// PHY management logic is idle.
    #[bit(2, r)]
    phy_mgmt_idle: bool,
    /// Current level of the MDIO input pin.
    #[bit(1, r)]
    mdio_in: bool,
    /// PCS block link state.
    #[bit(0, r)]
    pcs_link_state: bool,
}

/// AHB burst length used by the DMA for data transfers.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BurstLength {
    /// Single transfers only.
    Single,
    /// Burst of 4.
    #[default]
    Incr4,
    /// Burst of 8.
    Incr8,
    /// Burst of 16.
    Incr16,
}

impl BurstLength {
    /// Bit pattern written to the DMA configuration register.
    pub const fn reg_value(&self) -> u5 {
        u5::new(match self {
            BurstLength::Single => 0b1,
            BurstLength::Incr4 => 0b100,
            BurstLength::Incr8 => 0b1000,
            BurstLength::Incr16 => 0b10000,
        })
    }
}

/// Endianness used for AHB bus transfers.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AhbEndianess {
    /// Little endian.
    Little = 0,
    /// Big endian.
    Big = 1,
}

/// DMA receive buffer size in AHB system memory, in units of 64 bytes.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaRxBufSize(u8);

impl DmaRxBufSize {
    /// Create a new value from a raw register value without validation.
    pub const fn new_with_raw_value(size: u8) -> Self {
        Self(size)
    }

    /// Create a new value. Returns [None] if `size` is 0, which is reserved.
    pub const fn new(size: u8) -> Option<Self> {
        if size == 0 {
            return None;
        }
        Some(Self(size))
    }

    /// Raw register value.
    pub const fn raw_value(&self) -> u8 {
        self.0
    }

    /// Buffer size in bytes.
    pub const fn size_in_bytes(&self) -> usize {
        self.0 as usize * 64
    }

    /// Value written to the DMA configuration register.
    pub const fn reg_value(&self) -> u8 {
        self.0
    }
}

/// DMA configuration register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    forbid_overlaps,
    defmt_fields(feature = "defmt")
)]
pub struct DmaConfig {
    /// Discards incoming frames when the AHB packet buffer is full.
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
    /// Default value is 0x0 (little endian)
    #[bit(6, rw)]
    endian_swap_mgmt_descriptor: AhbEndianess,
    /// AHB burst length, see [BurstLength].
    #[bits(0..=4, rw)]
    burst_length: u5,
}

/// Transmit status register. Write 1 to a bit to clear it.
#[bitbybit::bitfield(u32, debug, forbid_overlaps, defmt_bitfields(feature = "defmt"))]
pub struct TxStatus {
    /// AHB response was not OK for a transmit access.
    #[bit(8, rw)]
    hresp_not_ok: bool,
    /// Late collision occurred.
    #[bit(7, rw)]
    late_collision: bool,
    /// This bit should never be se because the DMA is configured for packet buffer mode.
    #[bit(6, rw)]
    underrun: bool,
    /// Transmit of a frame completed.
    #[bit(5, rw)]
    complete: bool,
    /// Frame corrupted due to an AHB error.
    #[bit(4, rw)]
    frame_corruption_ahb_error: bool,
    /// Its called "tx_go" inside the Zynq 7000 documentation, but I think this is just a
    /// TX active bit.
    #[bit(3, r)]
    go: bool,
    /// Retry limit was reached without a successful transmission.
    #[bit(2, rw)]
    retry_limit_reached: bool,
    /// A collision occurred.
    #[bit(1, rw)]
    collision: bool,
    /// TX descriptor read from memory had its used bit set.
    #[bit(0, rw)]
    read_when_used: bool,
}

impl TxStatus {
    /// Value that clears all status bits when written back.
    pub fn new_clear_all() -> Self {
        Self::new_with_raw_value(0xFF)
    }
}

/// Receive status register. Write 1 to a bit to clear it.
#[bitbybit::bitfield(u32, debug, forbid_overlaps, defmt_bitfields(feature = "defmt"))]
pub struct RxStatus {
    /// AHB response was not OK for a receive access.
    #[bit(3, rw)]
    hresp_not_ok: bool,
    /// Receive buffer overrun occurred.
    #[bit(2, rw)]
    overrun: bool,
    /// A frame was received.
    #[bit(1, rw)]
    frame_received: bool,
    /// No RX buffer was available for the received frame.
    #[bit(0, rw)]
    buf_not_available: bool,
}

impl RxStatus {
    /// Value that clears all status bits when written back.
    pub fn new_clear_all() -> Self {
        Self::new_with_raw_value(0xF)
    }
}

/// Interrupt status register. Write 1 to a bit to clear it.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    forbid_overlaps,
    defmt_bitfields(feature = "defmt")
)]
pub struct InterruptStatus {
    /// TSU second counter incremented.
    #[bit(26, rw)]
    tsu_sec_incr: bool,
    /// Marked N/A in datasheet.
    #[bit(17, rw)]
    partner_pg_rx: bool,
    /// Marked N/A in datasheet.
    #[bit(16, rw)]
    auto_negotiation_complete: bool,
    /// External interrupt input was asserted.
    #[bit(15, rw)]
    external_interrupt: bool,
    /// A pause frame was transmitted.
    #[bit(14, rw)]
    pause_transmitted: bool,
    /// Pause time reached zero.
    #[bit(13, rw)]
    pause_time_zero: bool,
    /// A pause frame with a non-zero pause quantum was received.
    #[bit(12, rw)]
    pause_with_non_zero_quantum: bool,
    /// AHB response was not OK.
    #[bit(11, rw)]
    hresp_not_ok: bool,
    /// Receive buffer overrun occurred.
    #[bit(10, rw)]
    rx_overrun: bool,
    /// Marked N/A in datasheet.
    #[bit(9, rw)]
    link_changed: bool,
    /// A frame was transmitted.
    #[bit(7, rw)]
    frame_sent: bool,
    /// Cleared on read.
    #[bit(6, r)]
    tx_frame_corruption_ahb_error: bool,
    /// Retry limit was reached or a late collision occurred.
    #[bit(5, rw)]
    tx_retry_limit_reached_or_late_collision: bool,
    /// TX descriptor read from memory had its used bit set.
    #[bit(3, rw)]
    tx_descr_read_when_used: bool,
    /// RX descriptor read from memory had its used bit set.
    #[bit(2, rw)]
    rx_descr_read_when_used: bool,
    /// A frame was received.
    #[bit(1, rw)]
    frame_received: bool,
    /// A management frame was sent over the MDIO interface.
    #[bit(0, rw)]
    mgmt_frame_sent: bool,
}

/// Interrupt enable and disable register, sharing the bit layout of [InterruptStatus].
#[bitbybit::bitfield(u32, default = 0x00, forbid_overlaps)]
#[derive(Debug)]
pub struct InterruptControl {
    /// TSU second counter increment interrupt.
    #[bit(26, w)]
    tsu_sec_incr: bool,
    /// Marked N/A in datasheet. Probably because external PHYs are used.
    #[bit(17, w)]
    partner_pg_rx: bool,
    /// Marked N/A in datasheet. Probably because external PHYs are used.
    #[bit(16, w)]
    auto_negotiation_complete: bool,
    /// External interrupt.
    #[bit(15, w)]
    external_interrupt: bool,
    /// Pause frame transmitted interrupt.
    #[bit(14, w)]
    pause_transmitted: bool,
    /// Pause time zero interrupt.
    #[bit(13, w)]
    pause_time_zero: bool,
    /// Pause frame with non-zero quantum received interrupt.
    #[bit(12, w)]
    pause_with_non_zero_quantum: bool,
    /// AHB response not OK interrupt.
    #[bit(11, w)]
    hresp_not_ok: bool,
    /// Receive overrun interrupt.
    #[bit(10, w)]
    rx_overrun: bool,
    /// Marked N/A in datasheet. Probably because external PHYs are used.
    #[bit(9, w)]
    link_changed: bool,
    /// Frame sent interrupt.
    #[bit(7, w)]
    frame_sent: bool,
    /// Frame corruption due to AHB error interrupt.
    #[bit(6, w)]
    tx_frame_corruption_ahb_error: bool,
    /// Retry limit reached or late collision interrupt.
    #[bit(5, w)]
    tx_retry_limit_reached_or_late_collision: bool,
    /// TX descriptor used bit read interrupt.
    #[bit(3, w)]
    tx_descr_read_when_used: bool,
    /// RX descriptor used bit read interrupt.
    #[bit(2, w)]
    rx_descr_read_when_used: bool,
    /// Frame received interrupt.
    #[bit(1, w)]
    frame_received: bool,
    /// Management frame sent interrupt.
    #[bit(0, w)]
    mgmt_frame_sent: bool,
}

impl InterruptControl {
    /// Value that disables (or clears, depending on the register) all interrupt bits.
    pub fn new_clear_all() -> Self {
        Self::new_with_raw_value(0xFFFF_FFFF)
    }
}

/// Selects a read or write operation for PHY register access.
#[bitbybit::bitenum(u2, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhyOperation {
    /// Read the PHY register.
    Read = 0b10,
    /// Write the PHY register.
    Write = 0b01,
}

/// PHY maintenance register, used to issue MDIO read and write transactions.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    forbid_overlaps,
    defmt_fields(feature = "defmt")
)]
pub struct PhyMaintenance {
    /// Must be 1 for Clause 22 operations.
    #[bit(30, rw)]
    clause_22: bool,
    /// Read or write operation, see [PhyOperation].
    #[bits(28..=29, rw)]
    op: Option<PhyOperation>,
    /// PHY address.
    #[bits(23..=27, rw)]
    phy_addr: u5,
    /// PHY register address.
    #[bits(18..=22, rw)]
    reg_addr: u5,
    /// Must be set to 0b10.
    #[bits(16..=17, rw)]
    must_be_0b10: u2,
    /// Data read from or written to the PHY register.
    #[bits(0..=15, rw)]
    data: u16,
}

/// Pause quantum register, used for the RX and TX pause time.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    forbid_overlaps,
    defmt_bitfields(feature = "defmt")
)]
pub struct PauseQuantum {
    /// Pause quantum value.
    #[bits(0..=15, rw)]
    value: u16,
}

/// Type ID match register, used to match a frame's type/length field.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    forbid_overlaps,
    defmt_bitfields(feature = "defmt")
)]
pub struct MatchRegister {
    /// Enables copying frames with a matching type ID.
    #[bit(31, rw)]
    copy_enable: bool,
    /// Type ID value to match.
    #[bits(0..=15, rw)]
    type_id: u16,
}

/// Gigabit Ethernet Controller (GEM) register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Network control register.
    net_ctrl: NetworkControl,
    /// Network configuration register.
    net_cfg: NetworkConfig,
    /// Network status register.
    #[mmio(PureRead)]
    net_status: NetworkStatus,
    _reserved0: u32,
    /// DMA configuration register.
    dma_cfg: DmaConfig,
    /// Transmit status register.
    tx_status: TxStatus,
    /// Base address of the RX buffer queue descriptor list.
    rx_buf_queue_base_addr: u32,
    /// Base address of the TX buffer queue descriptor list.
    tx_buf_queue_base_addr: u32,
    /// Receive status register.
    rx_status: RxStatus,
    /// Interrupt status register.
    interrupt_status: InterruptStatus,
    /// Interrupt enable register.
    interrupt_enable: InterruptControl,
    /// Interrupt disable register.
    interrupt_disable: InterruptControl,
    /// Interrupt mask register, showing which interrupts are currently enabled.
    interrupt_mask: InterruptStatus,
    /// PHY maintenance register, used for MDIO transactions.
    phy_maintenance: PhyMaintenance,
    /// Pause quantum used for received pause frames.
    #[mmio(PureRead)]
    rx_pause_quantum: PauseQuantum,
    /// Pause quantum used for transmitted pause frames.
    tx_pause_quantum: PauseQuantum,
    _reserved1: [u32; 0x10],
    /// Lower 32 bits of the multicast hash register.
    hash_low: u32,
    /// Upper 32 bits of the multicast hash register.
    hash_high: u32,
    /// Lower 32 bits of specific address register 1.
    addr1_low: u32,
    /// Upper bits of specific address register 1.
    addr1_high: u32,
    /// Lower 32 bits of specific address register 2.
    addr2_low: u32,
    /// Upper bits of specific address register 2.
    addr2_high: u32,
    /// Lower 32 bits of specific address register 3.
    addr3_low: u32,
    /// Upper bits of specific address register 3.
    addr3_high: u32,
    /// Lower 32 bits of specific address register 4.
    addr4_low: u32,
    /// Upper bits of specific address register 4.
    addr4_high: u32,
    /// Type ID match registers.
    match_reg: [MatchRegister; 4],
    /// Wake on LAN register.
    wake_on_lan: u32,
    /// Inter packet gap stretch register.
    ipg_stretch: u32,
    /// Stacked VLAN register.
    stacked_vlan: u32,
    /// TX priority-based flow control register.
    tx_pfc: u32,
    /// Lower 32 bits of the specific address 1 mask register.
    addr1_mask_low: u32,
    /// Upper bits of the specific address 1 mask register.
    addr1_mask_high: u32,
    _reserved2: [u32; 0x0B],
    /// Should be 0x20118.
    #[mmio(PureRead)]
    module_id: u32,
    /// Ethernet statistics registers.
    #[mmio(Inner)]
    statistics: Statistics,
    _reserved3: [u32; 0x34],
    /// Design configuration register 2.
    #[mmio(PureRead)]
    design_cfg_2: u32,
    /// Design configuration register 3.
    #[mmio(PureRead)]
    design_cfg_3: u32,
    /// Design configuration register 4.
    #[mmio(PureRead)]
    design_cfg_4: u32,
    /// Design configuration register 5.
    #[mmio(PureRead)]
    design_cfg_5: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x294);

/// Ethernet statistics registers
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Statistics {
    /// Lower 32 bits of transmitted octets.
    #[mmio(PureRead)]
    tx_octets_low: u32,
    /// Upper bits of transmitted octets.
    #[mmio(PureRead)]
    tx_octets_high: u32,
    /// Number of frames transmitted.
    #[mmio(PureRead)]
    tx_count: u32,
    /// Number of broadcast frames transmitted.
    #[mmio(PureRead)]
    tx_broadcast: u32,
    /// Number of multicast frames transmitted.
    #[mmio(PureRead)]
    tx_multicast: u32,
    /// Number of pause frames transmitted.
    #[mmio(PureRead)]
    tx_pause: u32,
    /// Number of 64 byte frames transmitted.
    #[mmio(PureRead)]
    tx_64_bits: u32,
    /// Number of 65 to 127 byte frames transmitted.
    #[mmio(PureRead)]
    tx_65_to_127_bits: u32,
    /// Number of 128 to 255 byte frames transmitted.
    #[mmio(PureRead)]
    tx_128_to_255_bits: u32,
    /// Number of 256 to 511 byte frames transmitted.
    #[mmio(PureRead)]
    tx_256_to_511_bits: u32,
    /// Number of 512 to 1023 byte frames transmitted.
    #[mmio(PureRead)]
    tx_512_to_1023_bits: u32,
    /// Number of 1024 to 1518 byte frames transmitted.
    #[mmio(PureRead)]
    tx_1024_to_1518_bits: u32,
    _reserved0: u32,
    /// Number of transmit underrun errors.
    #[mmio(PureRead)]
    tx_underruns: u32,
    /// Number of frames transmitted with a single collision.
    #[mmio(PureRead)]
    single_collision_frames: u32,
    /// Number of frames transmitted with multiple collisions.
    #[mmio(PureRead)]
    multi_collision_frames: u32,
    /// Number of frames not transmitted due to excessive collisions.
    #[mmio(PureRead)]
    excessive_collisions: u32,
    /// Number of frames transmitted with a late collision.
    #[mmio(PureRead)]
    late_collisions: u32,
    /// Number of frames experiencing transmit deferral.
    #[mmio(PureRead)]
    deferred_tx: u32,
    /// Number of carrier sense errors during transmission.
    #[mmio(PureRead)]
    carrier_sense_errors: u32,
    /// Lower 32 bits of received octets.
    #[mmio(PureRead)]
    rx_octets_low: u32,
    /// Upper bits of received octets.
    #[mmio(PureRead)]
    rx_octets_high: u32,
    /// Number of frames received.
    #[mmio(PureRead)]
    rx_count: u32,
    /// Number of broadcast frames received.
    #[mmio(PureRead)]
    rx_broadcast: u32,
    /// Number of multicast frames received.
    #[mmio(PureRead)]
    rx_multicast: u32,
    /// Number of pause frames received.
    #[mmio(PureRead)]
    rx_pause: u32,
    /// Number of 64 byte frames received.
    #[mmio(PureRead)]
    rx_64_bits: u32,
    /// Number of 65 to 127 byte frames received.
    #[mmio(PureRead)]
    rx_65_to_127_bits: u32,
    /// Number of 128 to 255 byte frames received.
    #[mmio(PureRead)]
    rx_128_to_255_bits: u32,
    /// Number of 256 to 511 byte frames received.
    #[mmio(PureRead)]
    rx_256_to_511_bits: u32,
    /// Number of 512 to 1023 byte frames received.
    #[mmio(PureRead)]
    rx_512_to_1023_bits: u32,
    /// Number of 1024 to 1518 byte frames received.
    #[mmio(PureRead)]
    rx_1024_to_1518_bits: u32,
    _reserved1: u32,
    /// Number of undersized frames received.
    #[mmio(PureRead)]
    rx_undersize: u32,
    /// Number of oversized frames received.
    #[mmio(PureRead)]
    rx_oversize: u32,
    /// Number of jabber frames received.
    #[mmio(PureRead)]
    rx_jabber: u32,
    /// Number of frames received with an FCS error.
    #[mmio(PureRead)]
    rx_frame_check_sequence_errors: u32,
    /// Number of frames received with a length field error.
    #[mmio(PureRead)]
    rx_length_field_errors: u32,
    /// Number of frames received with a symbol error.
    #[mmio(PureRead)]
    rx_symbol_errors: u32,
    /// Number of frames received with an alignment error.
    #[mmio(PureRead)]
    rx_alignment_errors: u32,
    /// Number of frames dropped due to a lack of receive resources.
    #[mmio(PureRead)]
    rx_resource_errors: u32,
    /// Number of frames dropped due to a receive overrun.
    #[mmio(PureRead)]
    rx_overrun_errors: u32,
    /// Number of frames received with an IP header checksum error.
    #[mmio(PureRead)]
    rx_ip_header_checksum_errors: u32,
    /// Number of frames received with a TCP checksum error.
    #[mmio(PureRead)]
    rx_tcp_checksum_errors: u32,
    /// Number of frames received with a UDP checksum error.
    #[mmio(PureRead)]
    rx_udp_checksum_errors: u32,
}

impl Registers {
    /// Create a new Gigabit Ethernet MMIO instance for GEM 0 at address [GEM_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(GEM_0_BASE_ADDR) }
    }

    /// Create a new Gigabit Ethernet MMIO instance for GEM 1 at address [GEM_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(GEM_1_BASE_ADDR) }
    }
}
