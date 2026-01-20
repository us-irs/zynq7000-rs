use std::{collections::HashMap, ops::RangeInclusive, path::Path};

use clap::Parser as _;
use simple_logger::SimpleLogger;

const DDRC_ADDR_RANGE: RangeInclusive<u32> = 0xf800_6000..=0xf800_62b4;
const DDRIOB_ADDR_RANGE: RangeInclusive<u32> = 0xf800_0b40..=0xf800_0b68;

const DDRC_FILE_NAME: &str = "ddrc_config_autogen.rs";
const DDRIOB_FILE_NAME: &str = "ddriob_config_autogen.rs";

#[derive(clap::Parser, Debug)]
#[command(version, about)]
pub struct Cli {
    /// Path to ps7init.tcl file.
    #[arg(short, long)]
    path: String,
}

fn extract_hex_values(line: &str) -> Option<(u32, u32, u32)> {
    let re = regex::Regex::new(r"0[xX]([0-9A-Fa-f]+)").unwrap();

    let captures: Vec<u32> = re
        .captures_iter(line)
        .filter_map(|cap| u32::from_str_radix(&cap[1], 16).ok())
        .collect();

    if captures.len() == 3 {
        Some((captures[0], captures[1], captures[2]))
    } else {
        None
    }
}

#[derive(Default)]
pub struct RegisterToValueMap(pub HashMap<u32, u32>);

impl RegisterToValueMap {
    fn val_as_token(&self, reg_name: &str, addr: u32) -> proc_macro2::TokenStream {
        let val = self.0.get(&addr).unwrap_or_else(|| {
            panic!(
                "failed to retrieve register value for register {}",
                reg_name
            )
        });
        format!("{:#010x}", val)
            .parse::<proc_macro2::TokenStream>()
            .unwrap()
    }
}

enum ParsingMode {
    DdrRev3,
    MioRev3,
}

fn main() -> std::io::Result<()> {
    SimpleLogger::new().init().unwrap();
    let cli = Cli::parse();
    let ps7init_tcl = Path::new(&cli.path);
    if !ps7init_tcl.exists() {
        log::error!("File not found: {}", ps7init_tcl.display());
        std::process::exit(1);
    }
    let mut parsing_mode = None;

    let mut reg_to_values = RegisterToValueMap::default();

    for line in std::fs::read_to_string(ps7init_tcl)?.lines() {
        match parsing_mode {
            None => {
                if line.contains("ps7_ddr_init_data_3_0") {
                    parsing_mode = Some(ParsingMode::DdrRev3);
                } else if line.contains("ps7_mio_init_data_3_0") {
                    parsing_mode = Some(ParsingMode::MioRev3);
                }
                continue;
            }
            Some(ParsingMode::MioRev3) => {
                if line.contains("}") {
                    parsing_mode = None;
                    continue;
                }
            }
            Some(ParsingMode::DdrRev3) => {
                if line.contains("}") {
                    parsing_mode = None;
                    continue;
                }
            }
        }

        if let Some((addr, _mask, value)) = extract_hex_values(line)
            && (DDRC_ADDR_RANGE.contains(&addr) || DDRIOB_ADDR_RANGE.contains(&addr))
            && addr % 4 == 0
        {
            // Only use first value.
            if reg_to_values.0.contains_key(&addr) {
                if addr != 0xF800_6000 {
                    log::warn!("detected duplicate register value for address {}", addr);
                }
                continue;
            }
            reg_to_values.0.insert(addr, value);
        }
    }

    log::info!("generating DDRC config files: {}", DDRC_FILE_NAME);
    generate_ddrc_config(&reg_to_values, DDRC_FILE_NAME)?;

    log::info!("generating DDRIOB config files: {}", DDRIOB_FILE_NAME);
    generate_ddriob_config(&reg_to_values, DDRIOB_FILE_NAME)?;

    Ok(())
}

fn generate_ddrc_config(
    reg_to_values: &RegisterToValueMap,
    file_name: &str,
) -> std::io::Result<()> {
    // Format as hex strings
    let ddrc = reg_to_values.val_as_token("DDRC Control", 0xF800_6000);
    let two_rank = reg_to_values.val_as_token("Two Rank", 0xF800_6004);
    let hpr = reg_to_values.val_as_token("HPR", 0xF800_6008);
    let lpr = reg_to_values.val_as_token("LPR", 0xF800_600C);
    let wr = reg_to_values.val_as_token("WR", 0xF800_6010);
    let dram_param_0 = reg_to_values.val_as_token("DRAM Reg0", 0xF800_6014);
    let dram_param_1 = reg_to_values.val_as_token("DRAM Reg1", 0xF800_6018);
    let dram_param_2 = reg_to_values.val_as_token("DRAM Reg2", 0xF800_601C);
    let dram_param_3 = reg_to_values.val_as_token("DRAM Reg3", 0xF800_6020);
    let dram_param_4 = reg_to_values.val_as_token("DRAM Reg4", 0xF800_6024);
    let dram_init_param = reg_to_values.val_as_token("DRAM Init Param", 0xF800_6028);
    let dram_emr = reg_to_values.val_as_token("DRAM EMR", 0xF800_602C);
    let dram_emr_mr = reg_to_values.val_as_token("DRAM EMR MR", 0xF800_6030);
    let dram_burst8_rdwr = reg_to_values.val_as_token("DRAM Burst8 RDWR", 0xF800_6034);
    let dram_disable_dq = reg_to_values.val_as_token("DRAM Disable DQ", 0xF800_6038);
    let dram_addr_map_bank = reg_to_values.val_as_token("DRAM Addr Map Bank", 0xF800_603C);
    let dram_addr_map_col = reg_to_values.val_as_token("DRAM Addr Map Col", 0xF800_6040);
    let dram_addr_map_row = reg_to_values.val_as_token("DRAM Addr Map Row", 0xF800_6044);
    let dram_odt = reg_to_values.val_as_token("DRAM ODT", 0xF800_6048);
    let phy_cmd_timeout_rddata_cpt = reg_to_values.val_as_token("PHY CMD Timeout", 0xF800_6050);
    let dll_calib = reg_to_values.val_as_token("DLL Calib", 0xF800_6058);
    let odt_delay_hold = reg_to_values.val_as_token("ODT Delay Hold", 0xF800_605C);
    let ctrl_reg1 = reg_to_values.val_as_token("CTRL Reg 1", 0xF800_6060);
    let ctrl_reg2 = reg_to_values.val_as_token("CTRL Reg 2", 0xF800_6064);
    let ctrl_reg3 = reg_to_values.val_as_token("CTRL Reg 3", 0xF800_6068);
    let ctrl_reg4 = reg_to_values.val_as_token("CTRL Reg 4", 0xF800_606C);
    let ctrl_reg5 = reg_to_values.val_as_token("CTRL Reg 5", 0xF800_6078);
    let ctrl_reg6 = reg_to_values.val_as_token("CTRL Reg 6", 0xF800_607C);
    let che_t_zq = reg_to_values.val_as_token("CHE T ZQ", 0xF800_60A4);
    let che_t_zq_short_interval_reg =
        reg_to_values.val_as_token("CHE T ZQ Short Interval", 0xF800_60A8);
    let deep_powerdown = reg_to_values.val_as_token("Deep Powerdown", 0xF800_60AC);
    let reg_2c = reg_to_values.val_as_token("Reg 2C", 0xF800_60B0);
    let reg_2d = reg_to_values.val_as_token("Reg 2D", 0xF800_60B4);
    let dfi_timing = reg_to_values.val_as_token("DFI Timing", 0xF800_60B8);
    let che_ecc_ctrl = reg_to_values.val_as_token("CHE ECC CTRL", 0xF800_60C4);
    let ecc_scrub = reg_to_values.val_as_token("ECC Scrub", 0xF800_60F4);
    let phy_receiver_enable = reg_to_values.val_as_token("PHY Receiver Enable", 0xF800_6114);
    let phy_config_0 = reg_to_values.val_as_token("PHY Config 0", 0xF800_6118);
    let phy_config_1 = reg_to_values.val_as_token("PHY Config 1", 0xF800_611C);
    let phy_config_2 = reg_to_values.val_as_token("PHY Config 2", 0xF800_6120);
    let phy_config_3 = reg_to_values.val_as_token("PHY Config 3", 0xF800_6124);
    let phy_init_ratio_0 = reg_to_values.val_as_token("PHY Init Ratio 0", 0xF800_612C);
    let phy_init_ratio_1 = reg_to_values.val_as_token("PHY Init Ratio 1", 0xF800_6130);
    let phy_init_ratio_2 = reg_to_values.val_as_token("PHY Init Ratio 2", 0xF800_6134);
    let phy_init_ratio_3 = reg_to_values.val_as_token("PHY Init Ratio 3", 0xF800_6138);
    let phy_rd_dqs_config_0 = reg_to_values.val_as_token("PHY RD DQS Config 0", 0xF800_6140);
    let phy_rd_dqs_config_1 = reg_to_values.val_as_token("PHY RD DQS Config 1", 0xF800_6144);
    let phy_rd_dqs_config_2 = reg_to_values.val_as_token("PHY RD DQS Config 2", 0xF800_6148);
    let phy_rd_dqs_config_3 = reg_to_values.val_as_token("PHY RD DQS Config 3", 0xF800_614C);
    let phy_wr_dqs_config_0 = reg_to_values.val_as_token("PHY WR DQS Config 0", 0xF800_6154);
    let phy_wr_dqs_config_1 = reg_to_values.val_as_token("PHY WR DQS Config 1", 0xF800_6158);
    let phy_wr_dqs_config_2 = reg_to_values.val_as_token("PHY WR DQS Config 2", 0xF800_615C);
    let phy_wr_dqs_config_3 = reg_to_values.val_as_token("PHY WR DQS Config 3", 0xF800_6160);
    let phy_we_cfg_0 = reg_to_values.val_as_token("PHY WE Config 0", 0xF800_6168);
    let phy_we_cfg_1 = reg_to_values.val_as_token("PHY WE Config 1", 0xF800_616C);
    let phy_we_cfg_2 = reg_to_values.val_as_token("PHY WE Config 2", 0xF800_6170);
    let phy_we_cfg_3 = reg_to_values.val_as_token("PHY WE Config 3", 0xF800_6174);
    let phy_wr_data_slv_0 = reg_to_values.val_as_token("PHY WR Data Slv 0", 0xF800_617C);
    let phy_wr_data_slv_1 = reg_to_values.val_as_token("PHY WR Data Slv 1", 0xF800_6180);
    let phy_wr_data_slv_2 = reg_to_values.val_as_token("PHY WR Data Slv 2", 0xF800_6184);
    let phy_wr_data_slv_3 = reg_to_values.val_as_token("PHY WR Data Slv 3", 0xF800_6188);
    let reg64 = reg_to_values.val_as_token("Reg64", 0xF800_6190);
    let reg65 = reg_to_values.val_as_token("Reg65", 0xF800_6194);
    let page_mask = reg_to_values.val_as_token("Page Mask", 0xF800_6204);
    let axi_priority_wr_port_0 = reg_to_values.val_as_token("AXI Priority WR Port 0", 0xF800_6208);
    let axi_priority_wr_port_1 = reg_to_values.val_as_token("AXI Priority WR Port 1", 0xF800_620C);
    let axi_priority_wr_port_2 = reg_to_values.val_as_token("AXI Priority WR Port 2", 0xF800_6210);
    let axi_priority_wr_port_3 = reg_to_values.val_as_token("AXI Priority WR Port 3", 0xF800_6214);
    let axi_priority_rd_port_0 = reg_to_values.val_as_token("AXI Priority RD Port 0", 0xF800_6218);
    let axi_priority_rd_port_1 = reg_to_values.val_as_token("AXI Priority RD Port 1", 0xF800_621C);
    let axi_priority_rd_port_2 = reg_to_values.val_as_token("AXI Priority RD Port 2", 0xF800_6220);
    let axi_priority_rd_port_3 = reg_to_values.val_as_token("AXI Priority RD Port 3", 0xF800_6224);
    let lpddr_ctrl_0 = reg_to_values.val_as_token("LPDDR CTRL 0", 0xF800_62A8);
    let lpddr_ctrl_1 = reg_to_values.val_as_token("LPDDR CTRL 1", 0xF800_62AC);
    let lpddr_ctrl_2 = reg_to_values.val_as_token("LPDDR CTRL 2", 0xF800_62B0);
    let lpddr_ctrl_3 = reg_to_values.val_as_token("LPDDR CTRL 3", 0xF800_62B4);

    let generated = quote::quote! {
        //!This file was auto-generated by the [zynq7000-ps7init-extract](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/tools/zynq7000-ps7init-extract) program.
        //!
        //!This configuration file contains static DDR configuration parameters extracted from the
        //!AMD ps7init.tcl file
        use zynq7000::ddrc::regs;
        use zynq7000_hal::ddr::DdrcConfigSet;

        pub const DDRC_CONFIG_ZEDBOARD: DdrcConfigSet = DdrcConfigSet {
            ctrl: regs::DdrcControl::new_with_raw_value(#ddrc),
            two_rank: regs::TwoRankConfig::new_with_raw_value(#two_rank),
            hpr: regs::LprHprQueueControl::new_with_raw_value(#hpr),
            lpr: regs::LprHprQueueControl::new_with_raw_value(#lpr),
            wr: regs::WriteQueueControl::new_with_raw_value(#wr),
            dram_param_0: regs::DramParamReg0::new_with_raw_value(#dram_param_0),
            dram_param_1: regs::DramParamReg1::new_with_raw_value(#dram_param_1),
            dram_param_2: regs::DramParamReg2::new_with_raw_value(#dram_param_2),
            dram_param_3: regs::DramParamReg3::new_with_raw_value(#dram_param_3),
            dram_param_4: regs::DramParamReg4::new_with_raw_value(#dram_param_4),
            dram_init_param: regs::DramInitParam::new_with_raw_value(#dram_init_param),
            dram_emr: regs::DramEmr::new_with_raw_value(#dram_emr),
            dram_emr_mr: regs::DramEmrMr::new_with_raw_value(#dram_emr_mr),
            dram_burst8_rdwr: regs::DramBurst8ReadWrite::new_with_raw_value(#dram_burst8_rdwr),
            disable_dq: regs::DisableDq::new_with_raw_value(#dram_disable_dq),
            dram_addr_map_bank: regs::DramAddrMapBank::new_with_raw_value(#dram_addr_map_bank),
            dram_addr_map_col: regs::DramAddrMapColumn::new_with_raw_value(#dram_addr_map_col),
            dram_addr_map_row: regs::DramAddrMapRow::new_with_raw_value(#dram_addr_map_row),
            dram_odt: regs::DramOdt::new_with_raw_value(#dram_odt),
            phy_cmd_timeout_rddata_cpt: regs::PhyCmdTimeoutRdDataCpt::new_with_raw_value(#phy_cmd_timeout_rddata_cpt),
            dll_calib: regs::DllCalib::new_with_raw_value(#dll_calib),
            odt_delay_hold: regs::OdtDelayHold::new_with_raw_value(#odt_delay_hold),
            ctrl_reg1: regs::CtrlReg1::new_with_raw_value(#ctrl_reg1),
            ctrl_reg2: regs::CtrlReg2::new_with_raw_value(#ctrl_reg2),
            ctrl_reg3: regs::CtrlReg3::new_with_raw_value(#ctrl_reg3),
            ctrl_reg4: regs::CtrlReg4::new_with_raw_value(#ctrl_reg4),
            ctrl_reg5: regs::CtrlReg5::new_with_raw_value(#ctrl_reg5),
            ctrl_reg6: regs::CtrlReg6::new_with_raw_value(#ctrl_reg6),
            che_t_zq: regs::CheTZq::new_with_raw_value(#che_t_zq),
            che_t_zq_short_interval_reg: regs::CheTZqShortInterval::new_with_raw_value(#che_t_zq_short_interval_reg),
            deep_powerdown: regs::DeepPowerdown::new_with_raw_value(#deep_powerdown),
            reg_2c: regs::Reg2c::new_with_raw_value(#reg_2c),
            reg_2d: regs::Reg2d::new_with_raw_value(#reg_2d),
            dfi_timing: regs::DfiTiming::new_with_raw_value(#dfi_timing),
            che_ecc_ctrl: regs::CheEccControl::new_with_raw_value(#che_ecc_ctrl),
            ecc_scrub: regs::EccScrub::new_with_raw_value(#ecc_scrub),
            phy_receiver_enable: regs::PhyReceiverEnable::new_with_raw_value(#phy_receiver_enable),
            phy_config: [
                regs::PhyConfig::new_with_raw_value(#phy_config_0),
                regs::PhyConfig::new_with_raw_value(#phy_config_1),
                regs::PhyConfig::new_with_raw_value(#phy_config_2),
                regs::PhyConfig::new_with_raw_value(#phy_config_3),
            ],
            phy_init_ratio: [
                regs::PhyInitRatio::new_with_raw_value(#phy_init_ratio_0),
                regs::PhyInitRatio::new_with_raw_value(#phy_init_ratio_1),
                regs::PhyInitRatio::new_with_raw_value(#phy_init_ratio_2),
                regs::PhyInitRatio::new_with_raw_value(#phy_init_ratio_3),
            ],
            phy_rd_dqs_config: [
                regs::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_0),
                regs::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_1),
                regs::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_2),
                regs::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_3),
            ],
            phy_wr_dqs_config: [
                regs::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_0),
                regs::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_1),
                regs::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_2),
                regs::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_3),
            ],
            phy_we_cfg: [
                regs::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_0),
                regs::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_1),
                regs::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_2),
                regs::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_3),
            ],
            phy_wr_data_slv: [
                regs::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_0),
                regs::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_1),
                regs::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_2),
                regs::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_3),
            ],
            reg64: regs::Reg64::new_with_raw_value(#reg64),
            reg65: regs::Reg65::new_with_raw_value(#reg65),
            page_mask: #page_mask,
            axi_priority_wr_port: [
                regs::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_0),
                regs::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_1),
                regs::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_2),
                regs::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_3),
            ],
            axi_priority_rd_port: [
                regs::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_0),
                regs::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_1),
                regs::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_2),
                regs::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_3),
            ],
            lpddr_ctrl_0: regs::LpddrControl0::new_with_raw_value(#lpddr_ctrl_0),
            lpddr_ctrl_1: regs::LpddrControl1::new_with_raw_value(#lpddr_ctrl_1),
            lpddr_ctrl_2: regs::LpddrControl2::new_with_raw_value(#lpddr_ctrl_2),
            lpddr_ctrl_3: regs::LpddrControl3::new_with_raw_value(#lpddr_ctrl_3),
        };
    };

    std::fs::write(file_name, generated.to_string())?;
    Ok(())
}

fn generate_ddriob_config(
    reg_to_values: &RegisterToValueMap,
    file_name: &str,
) -> std::io::Result<()> {
    // Format as hex strings
    let addr0 = reg_to_values.val_as_token("DDRIOB Addr 0", 0xF800_0B40);
    let addr1 = reg_to_values.val_as_token("DDRIOB Addr 1", 0xF800_0B44);
    let data0 = reg_to_values.val_as_token("DDRIOB Data 0", 0xF800_0B48);
    let data1 = reg_to_values.val_as_token("DDRIOB Data 1", 0xF800_0B4C);
    let diff0 = reg_to_values.val_as_token("DDRIOB Diff 0", 0xF800_0B50);
    let diff1 = reg_to_values.val_as_token("DDRIOB Diff 1", 0xF800_0B54);
    let clock = reg_to_values.val_as_token("DDRIOB Clock", 0xF800_0B58);
    let generated = quote::quote! {
        //!This file was auto-generated by the [zynq7000-ps7init-extract](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/tools/zynq7000-ps7init-extract) program.
        //!
        //!This configuration file contains static DDRIOB configuration parameters extracted from the
        //!AMD ps7init.tcl file
        use zynq7000::ddrc::regs;
        use zynq7000_hal::ddr::DdriobConfigSet;

        pub const DDRIOB_CONFIG_SET_ZEDBOARD: DdriobConfigSet = DdriobConfigSet {
            addr0: regs::DdriobConfig::new_with_raw_value(#addr0),
            addr1: regs::DdriobConfig::new_with_raw_value(#addr1),
            data0: regs::DdriobConfig::new_with_raw_value(#data0),
            data1: regs::DdriobConfig::new_with_raw_value(#data1),
            diff0: regs::DdriobConfig::new_with_raw_value(#diff0),
            diff1: regs::DdriobConfig::new_with_raw_value(#diff1),
            clock: regs::DdriobConfig::new_with_raw_value(#clock),
        };
    };

    std::fs::write(file_name, generated.to_string())?;
    Ok(())
}
