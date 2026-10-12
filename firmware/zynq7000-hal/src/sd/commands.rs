//! Command register values for the SD card commands.
use arbitrary_int::u6;
use zynq7000::sdio::{BlockSelect, CommandRegister, ResponseType};

use embedded_sdmmc_types::sdcard::{AcmdId, CmdId};

/// Command configuration.
pub struct CommandConfig {
    /// Command index.
    pub id: u6,
    /// Response type.
    pub response_type: ResponseType,
    /// Check the command index of the response.
    pub index_check: bool,
    /// Check the CRC of the response.
    pub crc_check: bool,
}

impl CommandConfig {
    /// Command without response.
    pub const fn new_no_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::None,
            index_check: false,
            crc_check: false,
        }
    }

    /// Command with an R1 response.
    pub const fn new_with_r1_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_48bits,
            index_check: true,
            crc_check: true,
        }
    }

    /// Command with an R2 response.
    pub const fn new_with_r2_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_136bits,
            index_check: false,
            crc_check: true,
        }
    }

    /// Command with an R3 response.
    pub const fn new_with_r3_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_48bits,
            index_check: false,
            crc_check: false,
        }
    }

    /// Command with an R6 response.
    pub const fn new_with_r6_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_48bitsWithCheck,
            index_check: false,
            crc_check: false,
        }
    }
}

/// Command register value for a command without data.
pub const fn build_command_without_data(config: CommandConfig) -> CommandRegister {
    CommandRegister::builder()
        .with_command_index(config.id)
        .with_command_type(zynq7000::sdio::CommandType::Normal)
        .with_data_is_present(false)
        .with_command_index_check_enable(config.index_check)
        .with_command_crc_check_enable(config.crc_check)
        .with_response_type_select(config.response_type)
        .with_block_select(zynq7000::sdio::BlockSelect::SingleBlock)
        .with_data_transfer_direction(zynq7000::sdio::TransferDirection::Write)
        .with_auto_cmd12_enable(false)
        .with_block_count_enable(false)
        .with_dma_enable(false)
        .build()
}

/// CMD0: Reset the card to the idle state.
pub const CMD0_GO_IDLE_MODE: CommandRegister = build_command_without_data(
    CommandConfig::new_no_response(CmdId::CMD0_GoIdleState.raw_value()),
);
/// CMD2: Request the CID.
pub const CMD2_ALL_SEND_CID: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r2_response(CmdId::CMD2_AllSendCid.raw_value()),
);
/// CMD3: Request the relative card address.
pub const CMD3_SEND_RELATIVE_ADDR: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r6_response(CmdId::CMD3_SendRelativeAddr.raw_value()),
);
/// CMD7: Select the card.
pub const CMD7_SELECT_SD_CARD: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD7_SelectCard.raw_value()),
);
/// CMD8: Send the interface condition.
pub const CMD8_SEND_IF_COND: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD8_SendIfCond.raw_value()),
);
/// CMD9: Request the CSD.
pub const CMD9_SEND_CSD: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r2_response(CmdId::CMD9_SendCsd.raw_value()),
);
/// CMD13: Request the card status.
pub const CMD13_SEND_STATUS: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD13_SendStatus.raw_value()),
);
/// CMD17: Read a single block.
pub const CMD17_READ_SINGLE_BLOCK: CommandRegister = CommandRegister::builder()
    .with_command_index(CmdId::CMD17_ReadSingleBlock.raw_value())
    .with_command_type(zynq7000::sdio::CommandType::Normal)
    .with_data_is_present(true)
    .with_command_index_check_enable(true)
    .with_command_crc_check_enable(true)
    .with_response_type_select(ResponseType::_48bits)
    .with_block_select(BlockSelect::SingleBlock)
    .with_data_transfer_direction(zynq7000::sdio::TransferDirection::Read)
    .with_auto_cmd12_enable(false)
    .with_block_count_enable(false)
    .with_dma_enable(false)
    .build();
/// CMD24: Write a single block.
pub const CMD24_WRITE_BLOCK: CommandRegister = CommandRegister::builder()
    .with_command_index(CmdId::CMD24_WriteBlock.raw_value())
    .with_command_type(zynq7000::sdio::CommandType::Normal)
    .with_data_is_present(true)
    .with_command_index_check_enable(true)
    .with_command_crc_check_enable(true)
    .with_response_type_select(ResponseType::_48bits)
    .with_block_select(BlockSelect::SingleBlock)
    .with_data_transfer_direction(zynq7000::sdio::TransferDirection::Write)
    .with_auto_cmd12_enable(false)
    .with_block_count_enable(false)
    .with_dma_enable(false)
    .build();

/// CMD55: The next command is an application specific command.
pub const CMD55_APP_CMD: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD55_AppCmd.raw_value()),
);
/// ACMD6: Set the bus width.
pub const ACMD6_SET_BUS_WIDTH: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(AcmdId::ACMD6_SetBusWidth.raw_value()),
);
/// ACMD41: Send the operating condition.
pub const ACMD41_SEND_IF_COND: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r3_response(AcmdId::ACMD41_SdSendOpCond.raw_value()),
);
