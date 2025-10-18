use arbitrary_int::{u2, u4, u6, u12};

pub const SDIO_BASE_ADDR_0: usize = 0xE010_0000;
pub const SDIO_BASE_ADDR_1: usize = 0xE010_1000;

#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum BufferSize {
    _4kB = 0b000,
    _8kB = 0b001,
    _16kB = 0b010,
    _32kB = 0b011,
    _64kB = 0b100,
    _128kB = 0b101,
    _256kB = 0b110,
    _512kB = 0b111,
}

#[bitbybit::bitfield(u32, debug)]
pub struct BlockParams {
    #[bits(16..=31, rw)]
    blocks_count: u16,
    #[bits(12..=14, rw)]
    buffer_size: BufferSize,
    #[bits(0..=11, rw)]
    block_size: u12,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum CommandType {
    Normal = 0b00,
    Suspend = 0b01,
    Resume = 0b10,
    Abort = 0b11,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum ResponseLength {
    NoResponse = 0b00,
    ResponseLength136 = 0b01,
    ResponseLength48 = 0b10,
    ResponseLength48Check = 0b11,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum BlockSelect {
    SingleBlock = 0,
    MultiBlock = 1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum TransferDirection {
    /// Host to card.
    Write = 0,
    /// Card to host.
    Read = 1,
}
#[bitbybit::bitfield(u32, debug)]
pub struct TransferModeAndCommand {
    /// Set to command number (CMD0-63, ACMD0-63)
    #[bits(24..=29, rw)]
    command_index: u6,
    #[bits(22..=23, rw)]
    command_type: CommandType,
    /// Set to [false] for the following:
    ///
    /// 1. Commands using only CMD line (ex. CMD52).
    /// 2. Commands with no data transfer but using busy signal on DAT\[0\].
    /// 3. Resume Command.
    #[bit(21, rw)]
    data_is_present: bool,
    /// When 1, the host controller checks the index field in the response to see if it has the
    /// same value as the command index.
    #[bit(20, rw)]
    command_index_check_enable: bool,
    /// When 1, the host controller checks the CRC field in the response.
    #[bit(18, rw)]
    command_crc_check_enable: bool,
    #[bits(16..=17, rw)]
    response_type_select: u2,
    #[bit(5, rw)]
    multi_single_block_select: BlockSelect,
    #[bit(4, rw)]
    data_transfer_direction: TransferDirection,
    /// Multiple block transfers for memory require CMD12 to stop the transaction. When this bit is
    /// 1, the host controller issues CMD12 automatically when completing the last block tranfer.
    #[bit(2, rw)]
    auto_cmd12_enable: bool,
    /// Enable block count register, which is only relevant for multiple block transfers.
    #[bit(1, rw)]
    block_count_enable: bool,
    #[bit(0, rw)]
    dma_enable: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct PresentState {
    #[bit(24, r)]
    cmd_line_signal_level: bool,
    #[bits(20..=23, r)]
    data_line_signal_level: u4,
    /// The Write Protect Switch is supported for memory and combo cards. This bit reflects the
    /// inversion of the SDx_WP pin.
    #[bit(19, r)]
    write_protect_switch_level: bool,
    /// This bit reflects the inverse value of the SDx_CDn pin.
    #[bit(18, r)]
    card_detect_pin_level: bool,
    /// This bit is used for testing. If it is 0, the Card Detect Pin Level is not stable. If this
    /// bit is set to 1, it means the Card Detect Pin Level is stable. The Software Reset For All
    /// in the Software Reset Register shall not affect this bit.
    #[bit(17, r)]
    card_state_stable: bool,
    /// This bit indicates whether a card has been inserted. Changing from 0 to 1 generates a Card
    /// Insertion interrupt in the Normal Interrupt Status register and changing from 1 to 0
    /// generates a Card Removal Interrupt in the Normal Interrupt Status register. The Software
    /// Reset For All in the Software Reset register shall not affect this bit. If a Card is
    /// removed while its power is on and its clock is oscillating, the HC shall clear SD Bus Power
    /// in the Power Control register and SD Clock Enable in the Clock control register. In
    /// addition the HD should clear the HC by the Software Reset For All in Software register. The
    /// card detect is active regardless of the SD Bus Power.
    #[bit(16, r)]
    card_inserted: bool,
    /// This status is used for non-DMA read transfers. This read only flag indicates that valid
    /// data exists in the host side buffer status. If this bit is 1, readable data exists in the
    /// buffer. A change of this bit from 1 to 0 occurs when all the block data is read from the
    /// buffer. A change of this bit from 0 to 1 occurs when all the block data is ready in the
    /// buffer and generates the Buffer Read Ready Interrupt.
    #[bit(11, r)]
    buffer_readable: bool,
    /// This status is used for non-DMA write transfers. This read only flag indicates if space is
    /// available for write data. If this bit is 1, data can be written to the buffer. A change of
    /// this bit from 1 to 0 occurs when all the block data is written to the buffer. A change of
    /// this bit from 0 to 1 occurs when top of block data can be written to the buffer and
    /// generates the Buffer Write Ready Interrupt.
    #[bit(10, r)]
    buffer_writable: bool,
    /// This status is used for detecting completion of a read transfer. This bit is set to 1 for
    /// either of the following conditions:
    ///
    ///  1. After the end bit of the read command
    ///  2. When writing a 1 to continue Request in the Block Gap Control register to restart a read
    ///  transfer.
    ///
    /// This bit is cleared to 0 for either of the following conditions:
    ///
    ///  1. When the last data block as specified by block length is transferred to the system.
    ///  2. When all valid data blocks have been transferred to the system and no current block
    ///  transfers are being sent as a result of the Stop At Block Gap Request set to 1. A transfer
    ///  complete interrupt is generated when this bit changes to 0.
    #[bit(9, r)]
    read_transfer_active: bool,
    /// This status indicates a write transfer is active. If this bit is 0, it means no valid write
    /// data exists in the HC. This bit is set in either of the following cases: 1. After the end
    /// bit of the write command. 2. When writing a 1 to Continue Request in the Block Gap Control
    /// register to restart a write transfer.
    ///
    /// This bit is cleared in either of the following cases:
    ///
    ///  1. After getting the CRC status of the last data block as specified by the transfer count
    ///  (Single or Multiple)
    ///  2. After getting a CRC status of any block where data transmission is about to be stopped
    ///  by a Stop At Block Gap Request.
    ///
    /// During a write transaction, a Block Gap Event interrupt is generated when this bit is
    /// changed to 0, as a result of the Stop At Block Gap Request being set. This status is useful
    /// for the HD in determining when to issue commands during write busy.
    #[bit(8, r)]
    write_transfer_active: bool,
    #[bit(2, r)]
    dat_line_active: bool,
    /// This status bit is generated if either the DAT Line Active or the Read transfer Active is
    /// set to 1. If this bit is 0, it indicates the HC can issue the next SD command. Commands
    /// with busy signal belong to Command Inhibit (DAT) (ex. R1b, R5b type). Changing from 1 to 0
    /// generates a Transfer Complete interrupt in the Normal interrupt status register.
    #[bit(1, r)]
    command_inhibit_dat: bool,
    /// 0 indicates the CMD line is not in use and the host controller can issue a SD command
    /// using the CMD line. This bit is set immediately after the Command register (00Fh) is
    /// written. This bit is cleared when the command response is received. Even if the Command
    /// Inhibit (DAT) is set to 1, Commands using only the CMD line can be issued if this bit is 0.
    /// Changing from 1 to 0 generates a Command complete interrupt in the Normal Interrupt Status
    /// register. If the HC cannot issue the command because of a command conflict error or because
    /// of Command Not Issued By Auto CMD12 Error, this bit shall remain 1 and the Command Complete
    /// is not set. Status issuing Auto CMD12 is not read from this bit.
    #[bit(0, r)]
    command_inhibit_cmd: bool,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum DataTransferWidth {
    _1bit = 0,
    _4bit = 1,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum DmaSelect {
    Sdma = 0b00,
    Adma1_32bits = 0b01,
    Adma2_32bits = 0b10,
    Adma2_64bits = 0b11,
}

#[bitbybit::bitenum(u3, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
pub enum SdBusVoltageSelect {
    _1_8V = 0b101,
    _3_0V = 0b110,
    _3_3V = 0b111,
}

#[bitbybit::bitfield(u32, debug)]
pub struct HostPowerBlockgapWakeupControl {
    #[bit(26, rw)]
    wakeup_event_enable_on_sd_card_removal: bool,
    #[bit(25, rw)]
    wakeup_event_enable_on_sd_card_insertion: bool,
    #[bit(24, rw)]
    wakeup_event_enable_on_card_interrupt: bool,
    #[bit(19, rw)]
    interrupt_at_block_gap: bool,
    #[bit(18, rw)]
    read_wait_control: bool,
    #[bit(17, rw)]
    continue_request: bool,
    #[bit(16, rw)]
    stop_as_block_gap_request: bool,
    #[bits(9..=11, rw)]
    sd_bus_voltage_select: Option<SdBusVoltageSelect>,
    #[bit(8, rw)]
    sd_bus_power: bool,
    #[bit(7, rw)]
    card_detect_signal_detection: bool,
    #[bit(6, rw)]
    card_detetect_test_level: bool,
    #[bits(3..=4, rw)]
    dma_select: DmaSelect,
    #[bit(2, rw)]
    high_speed_enable: bool,
    #[bit(1, rw)]
    data_transfer_width: DataTransferWidth,
    #[bit(0, rw)]
    led_control: bool,
}

#[bitbybit::bitenum(u8, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
pub enum SdclkFrequencySelect {
    Div256 = 0x80,
    Div128 = 0x40,
    Div64 = 0x20,
    Div32 = 0x10,
    Div16 = 0x08,
    Div8 = 0x04,
    Div4 = 0x02,
    Div2 = 0x01,
    Div1 = 0x00,
}

#[bitbybit::bitfield(u32, debug)]
pub struct ClockAndTimeoutAndSwResetControl {
    #[bit(26, rw)]
    software_reset_for_dat_line: bool,
    #[bit(25, rw)]
    software_reset_for_cmd_line: bool,
    #[bit(24, rw)]
    software_reset_for_all: bool,
    /// Interval: TMCLK * 2^(13 + register value)
    ///
    /// 0b1111 is reserved.
    #[bits(16..=19, rw)]
    data_timeout_counter_value: u4,
    #[bits(8..=15, rw)]
    sdclk_frequency_select: Option<SdclkFrequencySelect>,
    #[bit(2, rw)]
    sd_clock_enable: bool,
    #[bit(1, r)]
    internal_clock_stable: bool,
    #[bit(0, rw)]
    internal_clock_enable: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptStatus {
    #[bit(29, rw)]
    ceata_error_status: bool,
    #[bit(28, rw)]
    target_response_error: bool,
    #[bit(25, rw)]
    adma_error: bool,
    #[bit(24, rw)]
    auto_cmd12_error: bool,
    #[bit(23, rw)]
    current_limit_error: bool,
    #[bit(22, rw)]
    data_end_bit_error: bool,
    #[bit(21, rw)]
    data_crc_error: bool,
    #[bit(20, rw)]
    data_timeout_error: bool,
    #[bit(19, rw)]
    command_index_error: bool,
    #[bit(18, rw)]
    command_end_bit_error: bool,
    #[bit(17, rw)]
    command_crc_error: bool,
    #[bit(16, rw)]
    command_timeout_error: bool,
    #[bit(15, r)]
    error_interrupt: bool,
    #[bit(10, rw)]
    boot_terminate: bool,
    #[bit(9, rw)]
    boot_ack_recv: bool,
    #[bit(8, r)]
    card_interrupt: bool,
    #[bit(7, rw)]
    card_removal: bool,
    #[bit(6, rw)]
    card_insertion: bool,
    #[bit(5, rw)]
    buffer_read_ready: bool,
    #[bit(4, rw)]
    buffer_write_ready: bool,
    #[bit(3, rw)]
    dma_interrupt: bool,
    #[bit(2, rw)]
    blockgap_event: bool,
    #[bit(1, rw)]
    transfer_complete: bool,
    #[bit(0, rw)]
    command_complete: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptMask {
    #[bit(29, rw)]
    ceata_error_status: bool,
    #[bit(28, rw)]
    target_response_error: bool,
    #[bit(25, rw)]
    adma_error: bool,
    #[bit(24, rw)]
    auto_cmd12_error: bool,
    #[bit(23, rw)]
    current_limit_error: bool,
    #[bit(22, rw)]
    data_end_bit_error: bool,
    #[bit(21, rw)]
    data_crc_error: bool,
    #[bit(20, rw)]
    data_timeout_error: bool,
    #[bit(19, rw)]
    command_index_error: bool,
    #[bit(18, rw)]
    command_end_bit_error: bool,
    #[bit(17, rw)]
    command_crc_error: bool,
    #[bit(16, rw)]
    command_timeout_error: bool,
    #[bit(15, rw)]
    error_interrupt: bool,
    #[bit(10, rw)]
    boot_terminate: bool,
    #[bit(9, rw)]
    boot_ack_recv: bool,
    #[bit(8, rw)]
    card_interrupt: bool,
    #[bit(7, rw)]
    card_removal: bool,
    #[bit(6, rw)]
    card_insertion: bool,
    #[bit(5, rw)]
    buffer_read_ready: bool,
    #[bit(4, rw)]
    buffer_write_ready: bool,
    #[bit(3, rw)]
    dma_interrupt: bool,
    #[bit(2, rw)]
    blockgap_event: bool,
    #[bit(1, rw)]
    transfer_complete: bool,
    #[bit(0, rw)]
    command_complete: bool,

}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    sdma_system_addr: u32,
    block_params: u32,
    /// Bit 39-8 of Command-Format.
    argument: u32,
    transfer_mode_and_command: TransferModeAndCommand,
    #[mmio(PureRead)]
    responses: [u32; 4],
    buffer_data_port: u32,
    #[mmio(PureRead)]
    present_state: PresentState,
    host_power_blockgap_wakeup_control: HostPowerBlockgapWakeupControl,
    clock_timeout_sw_reset_control: ClockAndTimeoutAndSwResetControl,
    interrupt_status: InterruptStatus,
    interrupt_status_enable: InterruptMask,
    interrupt_signal_enable: InterruptMask,
    #[mmio(PureRead)]
    auto_cmd12_error_status: u32,
    #[mmio(PureRead)]
    capabilities: u32,
    _reserved_0: u32,
    #[mmio(PureRead)]
    maximum_current_capabilities: u32,
    _reserved_1: u32,
    force_event_register: u32,
    adma_error_status: u32,
    adma_system_address: u32,
    _reserved_2: u32,
    boot_timeout_control: u32,
    debug_selection: u32,
    _reserved_3: [u32; 0x22],
    spi_interrupt_support: u32,
    _reserved_4: [u32; 0x2],
    slot_interrupt_status_host_controll_version: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x100);
