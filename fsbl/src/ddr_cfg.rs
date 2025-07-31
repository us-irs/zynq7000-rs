use zynq7000::slcr::ddriob::{DciType, DdriobConfig, InputType, OutputEnable};
use zynq7000_hal::ddr::DdriobConfigSet;

const DDRIOB_ADDR_CFG: DdriobConfig = DdriobConfig::builder()
    .with_pullup_enable(false)
    .with_output_enable(OutputEnable::OBuf)
    .with_term_disable_mode(false)
    .with_ibuf_disable_mode(false)
    .with_dci_type(DciType::Disabled)
    .with_termination_enable(false)
    .with_dci_update_enable(false)
    .with_inp_type(InputType::Off)
    .build();

const DDRIOB_DATA_CFG: DdriobConfig = DdriobConfig::builder()
    .with_pullup_enable(false)
    .with_output_enable(OutputEnable::OBuf)
    .with_term_disable_mode(false)
    .with_ibuf_disable_mode(false)
    .with_dci_type(DciType::DciTermination)
    .with_termination_enable(true)
    .with_dci_update_enable(false)
    .with_inp_type(InputType::VRefBasedDifferentialReceiverForSstlHstl)
    .build();

const DDRIOB_DIFF_CFG: DdriobConfig = DdriobConfig::builder()
    .with_pullup_enable(false)
    .with_output_enable(OutputEnable::OBuf)
    .with_term_disable_mode(false)
    .with_ibuf_disable_mode(false)
    .with_dci_type(DciType::DciTermination)
    .with_termination_enable(true)
    .with_dci_update_enable(false)
    .with_inp_type(InputType::DifferentialInputReceiver)
    .build();

const DDRIOB_CLOCK_CFG: DdriobConfig = DdriobConfig::builder()
    .with_pullup_enable(false)
    .with_output_enable(OutputEnable::OBuf)
    .with_term_disable_mode(false)
    .with_ibuf_disable_mode(false)
    .with_dci_type(DciType::Disabled)
    .with_termination_enable(false)
    .with_dci_update_enable(false)
    .with_inp_type(InputType::Off)
    .build();

pub const DDRIOB_CFG_SET_ZEDBOARD: DdriobConfigSet = DdriobConfigSet {
    addr0: DDRIOB_ADDR_CFG,
    addr1: DDRIOB_ADDR_CFG,
    data0: DDRIOB_DATA_CFG,
    data1: DDRIOB_DATA_CFG,
    diff0: DDRIOB_DIFF_CFG,
    diff1: DDRIOB_DIFF_CFG,
    clock: DDRIOB_CLOCK_CFG,
};
