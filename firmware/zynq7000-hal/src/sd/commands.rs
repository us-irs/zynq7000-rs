use arbitrary_int::u6;
use zynq7000::sdio::{BlockSelect, CommandRegister, ResponseType};

use embedded_sdmmc::sdcard::{AcmdId, CmdId};

pub struct CommandConfig {
    pub id: u6,
    pub response_type: ResponseType,
    pub index_check: bool,
    pub crc_check: bool,
}

impl CommandConfig {
    pub const fn new_no_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::None,
            index_check: false,
            crc_check: false,
        }
    }

    pub const fn new_with_r1_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_48bits,
            index_check: true,
            crc_check: true,
        }
    }

    pub const fn new_with_r2_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_136bits,
            index_check: false,
            crc_check: true,
        }
    }

    pub const fn new_with_r3_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_48bits,
            index_check: false,
            crc_check: false,
        }
    }

    pub const fn new_with_r6_response(id: u6) -> Self {
        Self {
            id,
            response_type: ResponseType::_48bitsWithCheck,
            index_check: false,
            crc_check: false,
        }
    }
}

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

pub const CMD0_GO_IDLE_MODE: CommandRegister = build_command_without_data(
    CommandConfig::new_no_response(CmdId::CMD0_GoIdleState.raw_value()),
);
pub const CMD2_ALL_SEND_CID: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r2_response(CmdId::CMD2_AllSendCid.raw_value()),
);
pub const CMD3_SEND_RELATIVE_ADDR: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r6_response(CmdId::CMD3_SendRelativeAddr.raw_value()),
);
pub const CMD7_SELECT_SD_CARD: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD7_SelectCard.raw_value()),
);
pub const CMD8_SEND_IF_COND: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD8_SendIfCond.raw_value()),
);
pub const CMD9_SEND_CSD: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r2_response(CmdId::CMD9_SendCsd.raw_value()),
);
pub const CMD13_SEND_STATUS: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD13_SendStatus.raw_value()),
);
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

pub const CMD55_APP_CMD: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(CmdId::CMD55_AppCmd.raw_value()),
);
pub const ACMD6_SET_BUS_WIDTH: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r1_response(AcmdId::ACMD6_SetBusWidth.raw_value()),
);
pub const ACMD41_SEND_IF_COND: CommandRegister = build_command_without_data(
    CommandConfig::new_with_r3_response(AcmdId::ACMD41_SdSendOpCond.raw_value()),
);
