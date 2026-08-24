/// Base address of the SDIO 0 register block.
pub const SDIO_BASE_ADDR_0: usize = 0xE010_0000;
/// Base address of the SDIO 1 register block.
pub const SDIO_BASE_ADDR_1: usize = 0xE010_1000;

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u2, u3, u4, u6, u12};

    /// SDMA host buffer boundary size, after which an SDMA interrupt is raised.
    #[bitbybit::bitenum(u3, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum BufferSize {
        /// 4 KB boundary.
        _4kB = 0b000,
        /// 8 KB boundary.
        _8kB = 0b001,
        /// 16 KB boundary.
        _16kB = 0b010,
        /// 32 KB boundary.
        _32kB = 0b011,
        /// 64 KB boundary.
        _64kB = 0b100,
        /// 128 KB boundary.
        _128kB = 0b101,
        /// 256 KB boundary.
        _256kB = 0b110,
        /// 512 KB boundary.
        _512kB = 0b111,
    }

    /// Combined block size and block count register for a data transfer.
    #[bitbybit::bitfield(
        u32,
        debug,
        default = 0x0,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct BlockParams {
        /// Number of blocks for a multiple block transfer.
        #[bits(16..=31, rw)]
        blocks_count: u16,
        /// SDMA buffer boundary, see [BufferSize].
        #[bits(12..=14, rw)]
        buffer_size: BufferSize,
        /// Transfer block size in bytes.
        #[bits(0..=11, rw)]
        block_size: u12,
    }

    /// Command type field of an SD command, used to control suspend, resume and abort.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Default, Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum CommandType {
        /// Regular command, neither suspend, resume nor abort.
        #[default]
        Normal = 0b00,
        /// Suspends a data transfer.
        Suspend = 0b01,
        /// Resumes a suspended data transfer.
        Resume = 0b10,
        /// Aborts a data transfer.
        Abort = 0b11,
    }

    /// Expected response type for an SD command.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ResponseType {
        /// No response.
        None = 0b00,
        /// 136-bit response.
        _136bits = 0b01,
        /// 48-bit response.
        _48bits = 0b10,
        /// 48-bit response with busy check.
        _48bitsWithCheck = 0b11,
    }

    /// Selects single or multiple block data transfer.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum BlockSelect {
        /// Single block transfer.
        SingleBlock = 0,
        /// Multiple block transfer.
        MultiBlock = 1,
    }

    /// Direction of the data transfer.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum TransferDirection {
        /// Host to card.
        Write = 0,
        /// Card to host.
        Read = 1,
    }

    /// SD command register, written to issue a command to the card.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CommandRegister {
        /// Set to command number (CMD0-63, ACMD0-63)
        #[bits(24..=29, rw)]
        command_index: u6,
        /// Suspend, resume or abort control, see [CommandType].
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
        /// Expected response type, see [ResponseType].
        #[bits(16..=17, rw)]
        response_type_select: ResponseType,
        /// Single or multiple block transfer selection.
        #[bit(5, rw)]
        block_select: BlockSelect,
        /// Direction of the data transfer.
        #[bit(4, rw)]
        data_transfer_direction: TransferDirection,
        /// Multiple block transfers for memory require CMD12 to stop the transaction. When this bit is
        /// 1, the host controller issues CMD12 automatically when completing the last block tranfer.
        #[bit(2, rw)]
        auto_cmd12_enable: bool,
        /// Enable block count register, which is only relevant for multiple block transfers.
        #[bit(1, rw)]
        block_count_enable: bool,
        /// Enables DMA for the data transfer.
        #[bit(0, rw)]
        dma_enable: bool,
    }

    /// Present state of the SD host controller and card.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct PresentState {
        /// Reflects the current level of the CMD line.
        #[bit(24, r)]
        cmd_line_signal_level: bool,
        /// Reflects the current level of the DAT\[3:0\] lines.
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
        /// DAT line is active, either for a read transfer or a command with a busy signal.
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

    /// Data bus width used for the DAT lines.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum BusWidth {
        /// 1-bit bus width.
        _1bit = 0,
        /// 4-bit bus width.
        _4bits = 1,
    }

    /// Selects the DMA mode used for a data transfer.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DmaSelect {
        /// Simple DMA.
        Sdma = 0b00,
        /// Advanced DMA 1, 32-bit addressing.
        Adma1_32bits = 0b01,
        /// Advanced DMA 2, 32-bit addressing.
        Adma2_32bits = 0b10,
        /// Advanced DMA 2, 64-bit addressing.
        Adma2_64bits = 0b11,
    }

    /// SD bus voltage selection.
    #[bitbybit::bitenum(u3, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SdBusVoltageSelect {
        /// SD bus power off.
        Off = 0b000,
        /// 1.8 V bus voltage.
        _1_8V = 0b101,
        /// 3.0 V bus voltage.
        _3_0V = 0b110,
        /// 3.3 V bus voltage.
        _3_3V = 0b111,
    }

    /// Combined host control, power control, block gap control and wakeup control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct HostPowerBlockgapWakeupControl {
        /// Enables a wakeup event on SD card removal.
        #[bit(26, rw)]
        wakeup_event_enable_on_sd_card_removal: bool,
        /// Enables a wakeup event on SD card insertion.
        #[bit(25, rw)]
        wakeup_event_enable_on_sd_card_insertion: bool,
        /// Enables a wakeup event on a card interrupt.
        #[bit(24, rw)]
        wakeup_event_enable_on_card_interrupt: bool,
        /// Enables the interrupt at the block gap for multiple block transfers.
        #[bit(19, rw)]
        interrupt_at_block_gap: bool,
        /// Enables the read wait mechanism used during suspend and resume.
        #[bit(18, rw)]
        read_wait_control: bool,
        /// Restarts a transfer previously stopped by a stop-at-block-gap request.
        #[bit(17, rw)]
        continue_request: bool,
        /// Stops the transfer at the next block gap.
        #[bit(16, rw)]
        stop_as_block_gap_request: bool,
        /// Selects the SD bus voltage, see [SdBusVoltageSelect].
        #[bits(9..=11, rw)]
        sd_bus_voltage_select: Option<SdBusVoltageSelect>,
        /// Turns power to the SD bus on or off.
        #[bit(8, rw)]
        sd_bus_power: bool,
        /// Selects the source of the card detect signal.
        #[bit(7, rw)]
        card_detect_signal_detection: bool,
        /// Card detect test level, used when the card detect signal source is overridden.
        #[bit(6, rw)]
        card_detetect_test_level: bool,
        /// Selects the DMA mode, see [DmaSelect].
        #[bits(3..=4, rw)]
        dma_select: DmaSelect,
        /// Enables high speed mode.
        #[bit(2, rw)]
        high_speed_enable: bool,
        /// Data bus width, see [BusWidth].
        #[bit(1, rw)]
        bus_width: BusWidth,
        /// Controls the LED indicating an active SD bus.
        #[bit(0, rw)]
        led_control: bool,
    }

    /// SD clock frequency divisor applied to the base clock.
    #[bitbybit::bitenum(u8, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SdClockDivisor {
        /// Divide the base clock by 256.
        Div256 = 0x80,
        /// Divide the base clock by 128.
        Div128 = 0x40,
        /// Divide the base clock by 64.
        Div64 = 0x20,
        /// Divide the base clock by 32.
        Div32 = 0x10,
        /// Divide the base clock by 16.
        Div16 = 0x08,
        /// Divide the base clock by 8.
        Div8 = 0x04,
        /// Divide the base clock by 4.
        Div4 = 0x02,
        /// Divide the base clock by 2.
        Div2 = 0x01,
        /// Use the base clock without division.
        Div1 = 0x00,
    }

    /// Combined clock control, timeout control and software reset register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ClockAndTimeoutAndSwResetControl {
        /// Resets the DAT line circuit only.
        #[bit(26, rw)]
        software_reset_for_dat_line: bool,
        /// Resets the CMD line circuit only.
        #[bit(25, rw)]
        software_reset_for_cmd_line: bool,
        /// Resets the entire host controller, except for the card detection circuit.
        #[bit(24, rw)]
        software_reset_for_all: bool,
        /// Interval: TMCLK * 2^(13 + register value)
        ///
        /// 0b1111 is reserved.
        #[bits(16..=19, rw)]
        data_timeout_counter_value: u4,
        /// SD clock frequency divisor, see [SdClockDivisor].
        #[bits(8..=15, rw)]
        sdclk_frequency_select: Option<SdClockDivisor>,
        /// Enables the SD clock output.
        #[bit(2, rw)]
        sd_clock_enable: bool,
        /// Indicates that the internal clock is stable and ready for use.
        #[bit(1, r)]
        internal_clock_stable: bool,
        /// Enables the internal clock oscillator.
        #[bit(0, rw)]
        internal_clock_enable: bool,
    }

    /// Normal and error interrupt status register. Write 1 to a bit to clear it.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct InterruptStatus {
        /// CE-ATA error status.
        #[bit(29, rw)]
        ceata_error_status: bool,
        /// Target response error.
        #[bit(28, rw)]
        target_response_error: bool,
        /// ADMA error occurred during a transfer.
        #[bit(25, rw)]
        adma_error: bool,
        /// Error occurred while executing an Auto CMD12 command.
        #[bit(24, rw)]
        auto_cmd12_error: bool,
        /// Power failed due to an overcurrent condition.
        #[bit(23, rw)]
        current_limit_error: bool,
        /// End bit of a read data block was not 1.
        #[bit(22, rw)]
        data_end_bit_error: bool,
        /// CRC check of read data failed.
        #[bit(21, rw)]
        data_crc_error: bool,
        /// Data transfer timed out.
        #[bit(20, rw)]
        data_timeout_error: bool,
        /// Command index in the response did not match the issued command.
        #[bit(19, rw)]
        command_index_error: bool,
        /// End bit of the command response was not 1.
        #[bit(18, rw)]
        command_end_bit_error: bool,
        /// CRC check of the command response failed.
        #[bit(17, rw)]
        command_crc_error: bool,
        /// Command response was not received within 64 SD clock cycles.
        #[bit(16, rw)]
        command_timeout_error: bool,
        /// Set if any error interrupt status bit is set.
        #[bit(15, r)]
        error_interrupt: bool,
        /// Boot operation was terminated.
        #[bit(10, rw)]
        boot_terminate: bool,
        /// Boot acknowledge was received.
        #[bit(9, rw)]
        boot_ack_recv: bool,
        /// Card raised an interrupt.
        #[bit(8, r)]
        card_interrupt: bool,
        /// Card was removed.
        #[bit(7, rw)]
        card_removal: bool,
        /// Card was inserted.
        #[bit(6, rw)]
        card_insertion: bool,
        /// Buffer is ready to be read.
        #[bit(5, rw)]
        buffer_read_ready: bool,
        /// Buffer is ready to be written.
        #[bit(4, rw)]
        buffer_write_ready: bool,
        /// A DMA buffer boundary was crossed.
        #[bit(3, rw)]
        dma_interrupt: bool,
        /// Transfer stopped at a block gap.
        #[bit(2, rw)]
        blockgap_event: bool,
        /// Data transfer has completed.
        #[bit(1, rw)]
        transfer_complete: bool,
        /// Command has completed.
        #[bit(0, rw)]
        command_complete: bool,
    }

    impl InterruptStatus {
        /// Mask of all defined interrupt status bits.
        pub const ALL_BITS: u32 = 0x33FF06FF;
    }

    /// Interrupt status enable / signal enable register, sharing the bit layout of
    /// [InterruptStatus]. Setting a bit enables the corresponding status or signal.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptMask {
        /// Enables the CE-ATA error status.
        #[bit(29, rw)]
        ceata_error_status: bool,
        /// Enables the target response error status.
        #[bit(28, rw)]
        target_response_error: bool,
        /// Enables the ADMA error status.
        #[bit(25, rw)]
        adma_error: bool,
        /// Enables the Auto CMD12 error status.
        #[bit(24, rw)]
        auto_cmd12_error: bool,
        /// Enables the current limit error status.
        #[bit(23, rw)]
        current_limit_error: bool,
        /// Enables the data end bit error status.
        #[bit(22, rw)]
        data_end_bit_error: bool,
        /// Enables the data CRC error status.
        #[bit(21, rw)]
        data_crc_error: bool,
        /// Enables the data timeout error status.
        #[bit(20, rw)]
        data_timeout_error: bool,
        /// Enables the command index error status.
        #[bit(19, rw)]
        command_index_error: bool,
        /// Enables the command end bit error status.
        #[bit(18, rw)]
        command_end_bit_error: bool,
        /// Enables the command CRC error status.
        #[bit(17, rw)]
        command_crc_error: bool,
        /// Enables the command timeout error status.
        #[bit(16, rw)]
        command_timeout_error: bool,
        /// Enables the error interrupt status.
        #[bit(15, rw)]
        error_interrupt: bool,
        /// Enables the boot terminate status.
        #[bit(10, rw)]
        boot_terminate: bool,
        /// Enables the boot acknowledge received status.
        #[bit(9, rw)]
        boot_ack_recv: bool,
        /// Enables the card interrupt status.
        #[bit(8, rw)]
        card_interrupt: bool,
        /// Enables the card removal status.
        #[bit(7, rw)]
        card_removal: bool,
        /// Enables the card insertion status.
        #[bit(6, rw)]
        card_insertion: bool,
        /// Enables the buffer read ready status.
        #[bit(5, rw)]
        buffer_read_ready: bool,
        /// Enables the buffer write ready status.
        #[bit(4, rw)]
        buffer_write_ready: bool,
        /// Enables the DMA interrupt status.
        #[bit(3, rw)]
        dma_interrupt: bool,
        /// Enables the block gap event status.
        #[bit(2, rw)]
        blockgap_event: bool,
        /// Enables the transfer complete status.
        #[bit(1, rw)]
        transfer_complete: bool,
        /// Enables the command complete status.
        #[bit(0, rw)]
        command_complete: bool,
    }

    /// Host controller capabilities.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct Capabilities {
        /// Supports SPI block mode.
        #[bit(30, rw)]
        spi_block_mode: bool,
        /// Supports SPI mode.
        #[bit(29, rw)]
        spi_mode: bool,
        /// Supports a 64-bit system bus.
        #[bit(28, rw)]
        _64_bit_system_bus_support: bool,
        /// Supports the interrupt mode instead of the INT_x pins.
        #[bit(27, rw)]
        interrupt_mode: bool,
        /// Supports 1.8 V bus voltage.
        #[bit(26, rw)]
        voltage_support_1_8v: bool,
        /// Supports 3.0 V bus voltage.
        #[bit(25, rw)]
        voltage_support_3_0v: bool,
        /// Supports 3.3 V bus voltage.
        #[bit(24, rw)]
        voltage_support_3_3v: bool,
        /// Supports suspend and resume.
        #[bit(23, rw)]
        suspend_resume_support: bool,
        /// Supports SDMA.
        #[bit(22, rw)]
        sdma_support: bool,
        /// Supports high speed mode.
        #[bit(21, rw)]
        high_speed_support: bool,
        /// Supports ADMA2.
        #[bit(19, rw)]
        adma2_support: bool,
        /// Supports the extended media bus (8-bit DAT).
        #[bit(18, rw)]
        extended_media_bus_support: bool,
        /// Maximum block length supported.
        #[bits(16..=17, rw)]
        max_block_length: u2,
        /// Unit of the timeout clock frequency.
        #[bit(7, rw)]
        timeout_clock_unit: bool,
    }

    /// Combined transfer block size, SDMA buffer boundary and block count register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct BlockSizeRegister {
        /// Enabled when block count enable in the transfer mode register is set to 1 and only
        /// valid for multiple block transfers.
        #[bits(16..=31, rw)]
        block_counts_for_current_transfer: u16,
        /// SDMA buffer boundary size.
        #[bits(12..=14, rw)]
        host_sdma_buffer_size: u3,
        /// Transfer block size in bytes.
        #[bits(0..=11, rw)]
        transfer_block_size: u12,
    }
}

/// SDIO controller register block.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// SDMA system address / Argument 2 register.
    sdma_system_addr: u32,
    /// Transfer block size, SDMA buffer boundary and block count.
    block_params: BlockSizeRegister,
    /// Bit 39-8 of Command-Format.
    argument: u32,
    /// Command register, writing it issues a command to the card.
    command: CommandRegister,
    /// Command response registers.
    #[mmio(PureRead)]
    responses: [u32; 4],
    /// Buffer data port for non-DMA data transfers.
    buffer_data_port: u32,
    /// Present state of the host controller and card.
    #[mmio(PureRead)]
    present_state: PresentState,
    /// Host control, power control, block gap control and wakeup control.
    host_power_blockgap_wakeup_control: HostPowerBlockgapWakeupControl,
    /// Clock control, timeout control and software reset.
    clock_timeout_sw_reset_control: ClockAndTimeoutAndSwResetControl,
    /// Normal and error interrupt status.
    interrupt_status: InterruptStatus,
    /// Enables reporting of interrupt status bits.
    interrupt_status_enable: InterruptMask,
    /// Enables signaling of interrupt status bits on the interrupt line.
    interrupt_signal_enable: InterruptMask,
    /// Error status of the last Auto CMD12 command.
    #[mmio(PureRead)]
    auto_cmd12_error_status: u32,
    /// Host controller capabilities.
    #[mmio(PureRead)]
    capabilities: Capabilities,
    _gap_0: u32,
    /// Maximum current capabilities for each supported bus voltage.
    #[mmio(PureRead)]
    maximum_current_capabilities: u32,
    _gap_1: u32,
    /// Forces an event for Auto CMD12 or CE-ATA testing.
    force_event_register: u32,
    /// ADMA error status.
    adma_error_status: u32,
    /// ADMA system address.
    adma_system_address: u32,
    _gap_2: u32,
    /// Timeout value for boot data transfers.
    boot_timeout_control: u32,
    /// Debug bus selection.
    debug_selection: u32,
    _gap_3: [u32; 0x22],
    /// SPI mode interrupt support.
    spi_interrupt_support: u32,
    _gap_4: [u32; 0x2],
    /// Slot interrupt status and host controller version.
    slot_interrupt_status_host_controll_version: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x100);

impl Registers {
    /// Create a new SDIO MMIO instance for SDIO 0 at address [SDIO_BASE_ADDR_0].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(SDIO_BASE_ADDR_0) }
    }

    /// Create a new SDIO MMIO instance for SDIO 1 at address [SDIO_BASE_ADDR_1].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(SDIO_BASE_ADDR_1) }
    }
}
