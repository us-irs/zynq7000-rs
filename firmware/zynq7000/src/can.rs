//! # CAN register module

/// Base address of instance 0.
pub const BASE_ADDR_0: usize = 0xE000_8000;
/// Base address of instance 1.
pub const BASE_ADDR_1: usize = 0xE000_9000;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u2, u3, u4, u11, u18};

    /// Enable bit.
    ///
    /// If the CEN bit is changed during core
    /// operation, it is recommended to reset the core
    /// so that operations start afresh.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum EnableBit {
        ///The CAN controller is in the Configuration mode.
        Config = 0,
        /// The CAN controller is in Loop Back, Sleep or Normal mode depending on the LBACK and
        /// SLEEP bits in the MSR.
        NormalLoopBackOrSleep = 1,
    }

    /// Software reset register.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct SwReset {
        /// Enable bit (CEN).
        #[bit(1, rw)]
        enable: EnableBit,
        /// Reset bit.
        ///
        /// If a 1 is written to this bit, all the CAN controller
        /// configuration registers (including the SRR) are
        /// reset
        #[bit(0, rw)]
        reset: bool,
    }

    /// Mode select register.
    ///
    /// LBACK and SLEEP should never be set at the same time. If both are set, LBACK takes
    /// priority. SNOOP mode requires LBACK and SLEEP to be 0. These bits can only be written
    /// when CEN in the SRR is 0.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ModeSelect {
        /// Snoop mode select. The core receives but does not participate in bus communication.
        #[bit(2, rw)]
        snoop: bool,
        /// Loop back mode select.
        #[bit(1, rw)]
        loopback: bool,
        /// Sleep mode select. Cleared automatically once the core wakes up.
        #[bit(0, rw)]
        sleep: bool,
    }

    /// Baud rate prescaler register.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct BaudRatePrescaler {
        /// Prescaler value. The actual value used is one more than the value written here.
        #[bits(0..=7, rw)]
        prescaler: u8,
    }

    /// Bit timing register.
    ///
    /// Specifies the Synchronization Jump Width, Time Segment 1 and Time Segment 2 as defined
    /// in CAN 2.0A, CAN 2.0B and ISO 11891-1. The actual value of each field is one more than
    /// the value written here.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct BitTiming {
        /// Synchronization Jump Width.
        #[bits(7..=8, rw)]
        sync_jump_width: u2,
        /// Time Segment 2, the Phase Segment 2.
        #[bits(4..=6, rw)]
        time_segment_2: u3,
        /// Time Segment 1, the sum of the Propagation Segment and Phase Segment 1.
        #[bits(0..=3, rw)]
        time_segment_1: u4,
    }

    /// Error counter register. Read-only, mirrors the transmit and receive error counters of
    /// the CAN protocol engine.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ErrorCounter {
        /// Receive error counter.
        #[bits(8..=15, r)]
        rx_counter: u8,
        /// Transmit error counter.
        #[bits(0..=7, r)]
        tx_counter: u8,
    }

    /// Error status register. Each bit is set when the corresponding error occurs on the bus,
    /// and cleared by writing a 1 to it.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ErrorStatus {
        /// Acknowledgment error. Write 1 to clear.
        #[bit(4, rw)]
        acknowledgment_error: bool,
        /// Bit error, the received bit does not match the transmitted bit. Write 1 to clear.
        #[bit(3, rw)]
        bit_error: bool,
        /// Stuff error. Write 1 to clear.
        #[bit(2, rw)]
        stuff_error: bool,
        /// Form error, a fixed form field of the message frame is invalid. Write 1 to clear.
        /// Also set, instead of [ErrorStatus::crc_error], on a CRC error with CRC delimiter
        /// corruption.
        #[bit(1, rw)]
        form_error: bool,
        /// CRC error. Write 1 to clear.
        #[bit(0, rw)]
        crc_error: bool,
    }

    /// Error state of the CAN core, see [Status::error_state].
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ErrorState {
        /// The core is in Configuration mode. The error state is undefined.
        Config = 0b00,
        /// Error Active state.
        Active = 0b01,
        /// Bus Off state.
        BusOff = 0b10,
        /// Error Passive state.
        Passive = 0b11,
    }

    /// Status register. Read-only, reports the FIFO status, error state, bus state and
    /// configuration mode of the core.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct Status {
        /// The core is in Snoop mode.
        #[bit(12, r)]
        snoop: bool,
        /// The acceptance filter mask and ID registers are not writable. Set when a 0 is
        /// written to a valid enable bit in [AcceptanceFilterEnable] while it is in use.
        #[bit(11, r)]
        acceptance_filter_busy: bool,
        /// The TX FIFO is full.
        #[bit(10, r)]
        tx_fifo_full: bool,
        /// The TX high priority buffer is full.
        #[bit(9, r)]
        tx_high_prio_buf_full: bool,
        /// Error state, see [ErrorState].
        #[bits(7..=8, r)]
        error_state: ErrorState,
        /// One or both error counters have a value of 96 or more.
        #[bit(6, r)]
        error_warning: bool,
        /// The core is either receiving or transmitting a message.
        #[bit(5, r)]
        bus_busy: bool,
        /// No bus communication is taking place.
        #[bit(4, r)]
        bus_idle: bool,
        /// The core is in Normal mode.
        #[bit(3, r)]
        normal: bool,
        /// The core is in Sleep mode.
        #[bit(2, r)]
        sleep: bool,
        /// The core is in Loop Back mode.
        #[bit(1, r)]
        loopback: bool,
        /// The core is in Configuration mode.
        #[bit(0, r)]
        config: bool,
    }

    /// Shared bit layout of the Interrupt Status, Interrupt Enable and Interrupt Clear
    /// registers, one bit per interrupt source.
    ///
    /// The permissions declared on the fields here apply to the Interrupt Enable register.
    /// [InterruptStatus] and [InterruptClear] restrict them further to read-only and
    /// write-only respectively.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptEnable {
        /// TX FIFO empty.
        #[bit(14, rw)]
        tx_fifo_empty: bool,
        /// TX FIFO empty, based on watermark programming in [WatermarkInterrupt].
        #[bit(13, rw)]
        tx_fifo_watermark_empty: bool,
        /// RX FIFO full, based on watermark programming in [WatermarkInterrupt].
        #[bit(12, rw)]
        rx_fifo_watermark_full: bool,
        /// The core entered Normal mode from Sleep mode.
        #[bit(11, rw)]
        wakeup: bool,
        /// The core entered Sleep mode.
        #[bit(10, rw)]
        sleep: bool,
        /// The core entered Bus Off state.
        #[bit(9, rw)]
        bus_off: bool,
        /// An error occurred during message transmission or reception.
        #[bit(8, rw)]
        error: bool,
        /// The RX FIFO is not empty.
        #[bit(7, rw)]
        rx_fifo_not_empty: bool,
        /// A message was lost because the RX FIFO was full while a new message was received.
        #[bit(6, rw)]
        rx_fifo_overflow: bool,
        /// A read was attempted on an empty RX FIFO.
        #[bit(5, rw)]
        rx_fifo_underflow: bool,
        /// A message was received successfully and stored in the RX FIFO.
        #[bit(4, rw)]
        message_received: bool,
        /// The TX high priority buffer is full.
        #[bit(3, rw)]
        tx_high_prio_buf_full: bool,
        /// The TX FIFO is full.
        #[bit(2, rw)]
        tx_fifo_full: bool,
        /// A message was transmitted successfully.
        #[bit(1, rw)]
        message_transmitted: bool,
        /// Arbitration was lost during message transmission.
        #[bit(0, rw)]
        arbitration_lost: bool,
    }

    /// Interrupt status register. Read-only, bits here can only be cleared through
    /// [InterruptClear]. Refer to [InterruptEnable] for the individual bit documentation.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct InterruptStatus {
        /// Refer to [InterruptEnable::tx_fifo_empty].
        #[bit(14, r)]
        tx_fifo_empty: bool,
        /// Refer to [InterruptEnable::tx_fifo_watermark_empty].
        #[bit(13, r)]
        tx_fifo_watermark_empty: bool,
        /// Refer to [InterruptEnable::rx_fifo_watermark_full].
        #[bit(12, r)]
        rx_fifo_watermark_full: bool,
        /// Refer to [InterruptEnable::wakeup].
        #[bit(11, r)]
        wakeup: bool,
        /// Refer to [InterruptEnable::sleep].
        #[bit(10, r)]
        sleep: bool,
        /// Refer to [InterruptEnable::bus_off].
        #[bit(9, r)]
        bus_off: bool,
        /// Refer to [InterruptEnable::error].
        #[bit(8, r)]
        error: bool,
        /// Refer to [InterruptEnable::rx_fifo_not_empty].
        #[bit(7, r)]
        rx_fifo_not_empty: bool,
        /// Refer to [InterruptEnable::rx_fifo_overflow].
        #[bit(6, r)]
        rx_fifo_overflow: bool,
        /// Refer to [InterruptEnable::rx_fifo_underflow].
        #[bit(5, r)]
        rx_fifo_underflow: bool,
        /// Refer to [InterruptEnable::message_received].
        #[bit(4, r)]
        message_received: bool,
        /// Refer to [InterruptEnable::tx_high_prio_buf_full].
        #[bit(3, r)]
        tx_high_prio_buf_full: bool,
        /// Refer to [InterruptEnable::tx_fifo_full].
        #[bit(2, r)]
        tx_fifo_full: bool,
        /// Refer to [InterruptEnable::message_transmitted].
        #[bit(1, r)]
        message_transmitted: bool,
        /// Refer to [InterruptEnable::arbitration_lost].
        #[bit(0, r)]
        arbitration_lost: bool,
    }

    /// Interrupt clear register. Write-only, writing 1 to a bit clears the corresponding bit
    /// in [InterruptStatus]. Refer to [InterruptEnable] for the individual bit documentation.
    #[bitbybit::bitfield(u32, default = 0, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(Debug)]
    pub struct InterruptClear {
        /// Refer to [InterruptEnable::tx_fifo_empty].
        #[bit(14, w)]
        tx_fifo_empty: bool,
        /// Refer to [InterruptEnable::tx_fifo_watermark_empty].
        #[bit(13, w)]
        tx_fifo_watermark_empty: bool,
        /// Refer to [InterruptEnable::rx_fifo_watermark_full].
        #[bit(12, w)]
        rx_fifo_watermark_full: bool,
        /// Refer to [InterruptEnable::wakeup].
        #[bit(11, w)]
        wakeup: bool,
        /// Refer to [InterruptEnable::sleep].
        #[bit(10, w)]
        sleep: bool,
        /// Refer to [InterruptEnable::bus_off].
        #[bit(9, w)]
        bus_off: bool,
        /// Refer to [InterruptEnable::error].
        #[bit(8, w)]
        error: bool,
        /// Refer to [InterruptEnable::rx_fifo_not_empty].
        #[bit(7, w)]
        rx_fifo_not_empty: bool,
        /// Refer to [InterruptEnable::rx_fifo_overflow].
        #[bit(6, w)]
        rx_fifo_overflow: bool,
        /// Refer to [InterruptEnable::rx_fifo_underflow].
        #[bit(5, w)]
        rx_fifo_underflow: bool,
        /// Refer to [InterruptEnable::message_received].
        #[bit(4, w)]
        message_received: bool,
        /// Refer to [InterruptEnable::tx_high_prio_buf_full].
        #[bit(3, w)]
        tx_high_prio_buf_full: bool,
        /// Refer to [InterruptEnable::tx_fifo_full].
        #[bit(2, w)]
        tx_fifo_full: bool,
        /// Refer to [InterruptEnable::message_transmitted].
        #[bit(1, w)]
        message_transmitted: bool,
        /// Refer to [InterruptEnable::arbitration_lost].
        #[bit(0, w)]
        arbitration_lost: bool,
    }

    /// Timestamp control register.
    #[bitbybit::bitfield(u32, default = 0, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(Debug)]
    pub struct TimestampControl {
        /// Clear timestamp. Write 1 to clear the free running counter used for RX timestamps,
        /// see [RxDlc::rx_timestamp]. Self-clears back to 0, a single write of 1 is enough.
        #[bit(0, w)]
        clear: bool,
    }

    /// Watermark interrupt register. Programs the RX FIFO full and TX FIFO empty watermark
    /// levels, valid range 1 to 63. Can only be written when CEN in the SRR is 0.
    #[bitbybit::bitfield(
        u32,
        default = 0x0000_3F3F,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct WatermarkInterrupt {
        /// TX FIFO empty watermark. [InterruptEnable::tx_fifo_watermark_empty] asserts as long
        /// as the number of empty TX FIFO slots is greater than this value.
        #[bits(8..=15, rw)]
        empty: u8,
        /// RX FIFO full watermark. [InterruptEnable::rx_fifo_watermark_full] asserts as long as
        /// the RX FIFO fill level is above this value.
        #[bits(0..=7, rw)]
        full: u8,
    }

    /// Message identifier, shared bit layout of the TX FIFO and TX high priority buffer ID
    /// registers. Writing this register enqueues the message assembled from the previously
    /// written [Dlc], [DataWord1] and [DataWord2] registers.
    #[bitbybit::bitfield(u32, default = 0, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(Debug)]
    pub struct TxIdentifier {
        /// Standard message ID, valid for both Standard and Extended frames.
        #[bits(21..=31, w)]
        standard_id: u11,
        /// Substitute Remote Transmission Request. Valid only for Standard frames, for
        /// Extended frames the written value is sent as-is.
        #[bit(20, w)]
        substitute_remote_request: bool,
        /// Identifier Extension. 1 selects an Extended message identifier, 0 a Standard one.
        #[bit(19, w)]
        extended_frame: bool,
        /// Extended message ID, valid only for Extended frames.
        #[bits(1..=18, w)]
        extended_id: u18,
        /// Remote Transmission Request. Valid only for Extended frames, 1 marks a Remote
        /// frame.
        #[bit(0, w)]
        remote_transmission_request: bool,
    }

    /// Message identifier of a received message. Read-only counterpart of [TxIdentifier].
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct RxIdentifier {
        /// Refer to [TxIdentifier::with_standard_id].
        #[bits(21..=31, r)]
        standard_id: u11,
        /// Refer to [TxIdentifier::with_substitute_remote_request]. Always 1 for Extended
        /// frames.
        #[bit(20, r)]
        substitute_remote_request: bool,
        /// Refer to [TxIdentifier::with_extended_frame].
        #[bit(19, r)]
        extended_frame: bool,
        /// Refer to [TxIdentifier::with_extended_id].
        #[bits(1..=18, r)]
        extended_id: u18,
        /// Refer to [TxIdentifier::with_remote_transmission_request].
        #[bit(0, r)]
        remote_transmission_request: bool,
    }

    /// Data length code, shared by the TX FIFO and TX high priority buffer DLC registers.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Dlc {
        /// Number of valid data bytes in [DataWord1] and [DataWord2].
        #[bits(28..=31, rw)]
        data_length_code: u4,
    }

    /// Data length code and RX timestamp of a received message.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct RxDlc {
        /// Refer to [Dlc::data_length_code].
        #[bits(28..=31, rw)]
        data_length_code: u4,
        /// Timestamp of the received message, based on the free running counter cleared
        /// through [TimestampControl::with_clear].
        #[bits(0..=15, rw)]
        rx_timestamp: u16,
    }

    /// First data word, containing the first four data bytes of a message. Shared by the TX
    /// FIFO, TX high priority buffer and RX FIFO. Reading a byte for which the message does
    /// not have data returns an invalid value.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DataWord1 {
        /// Data byte 0.
        #[bits(24..=31, rw)]
        data_byte_0: u8,
        /// Data byte 1.
        #[bits(16..=23, rw)]
        data_byte_1: u8,
        /// Data byte 2.
        #[bits(8..=15, rw)]
        data_byte_2: u8,
        /// Data byte 3.
        #[bits(0..=7, rw)]
        data_byte_3: u8,
    }

    /// Second data word, containing the last four data bytes of a message. Refer to
    /// [DataWord1].
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DataWord2 {
        /// Data byte 4.
        #[bits(24..=31, rw)]
        data_byte_4: u8,
        /// Data byte 5.
        #[bits(16..=23, rw)]
        data_byte_5: u8,
        /// Data byte 6.
        #[bits(8..=15, rw)]
        data_byte_6: u8,
        /// Data byte 7.
        #[bits(0..=7, rw)]
        data_byte_7: u8,
    }

    /// Acceptance filter enable register. Selects which of the four acceptance filter pairs
    /// (mask and ID) are used for acceptance filtering. If all filter-enable bits are 0, all
    /// received messages are stored in the RX FIFO.
    ///
    /// To modify a filter pair in Normal mode, its enable bit must be cleared first and set
    /// again once the filter has been reprogrammed.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct AcceptanceFilterEnable {
        /// Use acceptance filter pair 4, see [AcceptanceMask]/[AcceptanceId].
        #[bit(3, rw)]
        filter_4_enable: bool,
        /// Use acceptance filter pair 3.
        #[bit(2, rw)]
        filter_3_enable: bool,
        /// Use acceptance filter pair 2.
        #[bit(1, rw)]
        filter_2_enable: bool,
        /// Use acceptance filter pair 1.
        #[bit(0, rw)]
        filter_1_enable: bool,
    }

    /// Acceptance filter mask register, shared layout of all four acceptance filter mask
    /// registers. Paired with an [AcceptanceId] register of the same index. A mask bit of 1
    /// means the corresponding bit in the ID register is compared against the incoming
    /// message identifier, a mask bit of 0 means it is ignored.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct AcceptanceMask {
        /// Standard message ID mask.
        #[bits(21..=31, rw)]
        standard_id_mask: u11,
        /// Substitute Remote Transmission Request mask, Standard frames.
        #[bit(20, rw)]
        substitute_remote_request_mask: bool,
        /// Identifier Extension mask. If set and the paired [AcceptanceId::extended_frame] is
        /// 0, the filter only applies to Standard frames. If set and
        /// [AcceptanceId::extended_frame] is 1, the filter only applies to Extended frames. If
        /// clear, the filter applies to both.
        #[bit(19, rw)]
        extended_frame_mask: bool,
        /// Extended message ID mask.
        #[bits(1..=18, rw)]
        extended_id_mask: u18,
        /// Remote Transmission Request mask, Extended frames.
        #[bit(0, rw)]
        remote_transmission_request_mask: bool,
    }

    /// Acceptance filter ID register, shared layout of all four acceptance filter ID
    /// registers. Refer to [AcceptanceMask] for the paired mask register.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct AcceptanceId {
        /// Standard message ID.
        #[bits(21..=31, rw)]
        standard_id: u11,
        /// Substitute Remote Transmission Request, Standard frames.
        #[bit(20, rw)]
        substitute_remote_request: bool,
        /// Identifier Extension, refer to [AcceptanceMask::extended_frame_mask].
        #[bit(19, rw)]
        extended_frame: bool,
        /// Extended message ID.
        #[bits(1..=18, rw)]
        extended_id: u18,
        /// Remote Transmission Request, Extended frames.
        #[bit(0, rw)]
        remote_transmission_request: bool,
    }
}

pub use types::*;

/// CAN register block.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    software_reset: SwReset,
    mode_select: ModeSelect,
    baudrate_prescaler: BaudRatePrescaler,
    /// Bit timing (BTR) register.
    bit_timing: BitTiming,
    #[mmio(PureRead)]
    error_counter: ErrorCounter,
    #[mmio(PureRead, Write)]
    error_status: ErrorStatus,
    #[mmio(PureRead)]
    status: Status,
    #[mmio(PureRead)]
    interrupt_status: InterruptStatus,
    interrupt_enable: InterruptEnable,
    #[mmio(Write)]
    interrupt_clear: InterruptClear,
    #[mmio(Write)]
    timestamp_control: TimestampControl,
    watermark_interrupt: WatermarkInterrupt,
    #[mmio(Write)]
    tx_fifo_id: TxIdentifier,
    tx_fifo_dlc: Dlc,
    tx_fifo_word1: DataWord1,
    tx_fifo_word2: DataWord2,
    #[mmio(Write)]
    tx_hpb_id: TxIdentifier,
    tx_hpb_dlc: Dlc,
    tx_hpb_word1: DataWord1,
    tx_hpb_word2: DataWord2,
    #[mmio(PureRead)]
    rx_fifo_id: RxIdentifier,
    rx_fifo_dlc: RxDlc,
    rx_fifo_word1: DataWord1,
    rx_fifo_word2: DataWord2,
    acceptance_filter: AcceptanceFilterEnable,
    acceptance_filter_mask1: AcceptanceMask,
    acceptance_filter_id1: AcceptanceId,
    acceptance_filter_mask2: AcceptanceMask,
    acceptance_filter_id2: AcceptanceId,
    acceptance_filter_mask3: AcceptanceMask,
    acceptance_filter_id3: AcceptanceId,
    acceptance_filter_mask4: AcceptanceMask,
    acceptance_filter_id4: AcceptanceId,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x84);

impl Registers {
    /// Create a new CAN MMIO instance for for CAN block at address [BASE_ADDR_0].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub fn new_mmio_fixed0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(BASE_ADDR_0) }
    }

    /// Create a new CAN MMIO instance for for CAN block at address [BASE_ADDR_1].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub fn new_mmio_fixed1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(BASE_ADDR_1) }
    }
}
