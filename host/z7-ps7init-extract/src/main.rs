use std::{collections::HashMap, ops::RangeInclusive, path::Path};

use clap::Parser as _;
use simple_logger::SimpleLogger;
use z7_ps7init::{
    DDRIOB_ADDR_RANGE, ParsingMode, PsInitOps, RegOp, RegOpKind, SLCR_LOCK_ADDR, SLCR_UNLOCK_ADDR,
    extract_all_hex, parse_op, register_name,
};

const DDRC_ADDR_RANGE: RangeInclusive<u32> = 0xf800_6000..=0xf800_62b4;

// Written twice in `ps7_ddr_init_data_3_0`: once early to configure the DDRC (controller not yet
// started), and again near the end to actually trigger DRAM init. The second write is a distinct,
// meaningful op (see `record_settled_value`), not a redundant repeat.
const DDRC_CTRL_ADDR: u32 = 0xF800_6000;

const DDRC_FILE_NAME: &str = "ddrc_config_autogen.rs";
const DDRIOB_FILE_NAME: &str = "ddriob_config_autogen.rs";

const OPS_FILE_NAME: &str = "ps7_ops_autogen.rs";
const OPS_RON_FILE_NAME: &str = "z7_init_regs.ron";
const OPS_JSON_FILE_NAME: &str = "z7_init_regs.json";

impl From<&OpsFileInput<'_>> for PsInitOps {
    fn from(input: &OpsFileInput<'_>) -> Self {
        Self {
            pll_init_ops: input.pll_ops.to_vec(),
            clock_init_ops: input.clock_ops.to_vec(),
            ddr_init_ops: input.ddr_ops.to_vec(),
            ddriob_init_ops: input.ddriob_ops.to_vec(),
            mio_init_ops: input.mio_ops.to_vec(),
            post_config_ops: input.post_config_ops.to_vec(),
        }
    }
}

/// Bundles the PS7 init op sequences plus the output file name, so the `generate_ops_*_file`
/// functions don't each need to repeat the same multi-argument signature.
#[derive(Clone, Copy)]
struct OpsFileInput<'a> {
    pll_ops: &'a [RegOp],
    clock_ops: &'a [RegOp],
    ddr_ops: &'a [RegOp],
    ddriob_ops: &'a [RegOp],
    mio_ops: &'a [RegOp],
    post_config_ops: &'a [RegOp],
    file_name: &'a str,
}

#[derive(clap::Parser, Debug)]
#[command(version, about)]
pub struct Cli {
    /// Path to ps7init.tcl file.
    #[arg(short, long)]
    path: String,

    /// Also emit the PS7 register op sequences as a Rust source file (`ps7_ops_autogen.rs`),
    /// for on-target/compiled consumers. The RON file is always generated.
    #[arg(long)]
    rust: bool,

    /// Also emit the PS7 register op sequences as a JSON file (`z7_init_regs.json`), for
    /// consumers outside the Rust ecosystem. The RON file is always generated.
    #[arg(long)]
    json: bool,
}

#[inline]
fn extract_hex_values(line: &str) -> Option<(u32, u32, u32)> {
    let captures = extract_all_hex(line);

    if captures.len() == 3 {
        Some((captures[0], captures[1], captures[2]))
    } else {
        None
    }
}

#[derive(Default)]
pub struct RegisterToValueMap(pub HashMap<u32, u32>);

impl RegisterToValueMap {
    fn val_as_token(&self, addr: u32) -> proc_macro2::TokenStream {
        let val = self.0.get(&addr).unwrap_or_else(|| {
            panic!(
                "failed to retrieve register value for register {} ({addr:#010x})",
                register_name(addr).unwrap_or("unknown"),
            )
        });
        format!("{:#010x}", val)
            .parse::<proc_macro2::TokenStream>()
            .unwrap()
    }
}

/// Records a line's register value in the settled-config map used for the DDRC/DDRIOB Rust
/// codegen, if the line writes to a DDRC/DDRIOB register. Only the first value seen for a given
/// address is kept: DDRC/DDRIOB config registers written more than once (e.g. 0xF800_6000, which
/// is written once early to configure the DDRC and again later to trigger DRAM init) settle on
/// their last value in practice, but capturing "first write wins" here is enough for the
/// settled-config struct - the exact final execution order/timing is what `*_ops` is for instead.
fn record_settled_value(reg_to_values: &mut RegisterToValueMap, line: &str) {
    let Some((addr, _mask, value)) = extract_hex_values(line) else {
        return;
    };
    if !(DDRC_ADDR_RANGE.contains(&addr) || DDRIOB_ADDR_RANGE.contains(&addr)) || addr % 4 != 0 {
        return;
    }
    if reg_to_values.0.contains_key(&addr) {
        if addr != DDRC_CTRL_ADDR {
            log::warn!("detected duplicate register value for address {}", addr);
        }
        return;
    }
    reg_to_values.0.insert(addr, value);
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
    let mut pll_ops: Vec<RegOp> = Vec::new();
    let mut clock_ops: Vec<RegOp> = Vec::new();
    let mut ddr_ops: Vec<RegOp> = Vec::new();
    let mut ddriob_ops: Vec<RegOp> = Vec::new();
    let mut mio_ops: Vec<RegOp> = Vec::new();
    let mut post_config_ops: Vec<RegOp> = Vec::new();

    for line in std::fs::read_to_string(ps7init_tcl)?.lines() {
        // Outside any proc body: the only thing a line can do is open one.
        let Some(mode) = parsing_mode else {
            parsing_mode = ParsingMode::detect(line);
            continue;
        };
        // Inside a proc body: its closing brace ends it. None of the `ps7_*_init_data_3_0`
        // procs nest braces, so a bare `}` unambiguously means "end of this proc".
        if line.contains('}') {
            parsing_mode = None;
            continue;
        }

        // This updates the register value map with the final settled target values which is
        // required for the DDRC and DDRIOB configuration rust modules.
        record_settled_value(&mut reg_to_values, line);

        // Unlike the settled-value map above, ops are collected in source order and are not
        // deduplicated: PLL bring-up in particular relies on writing the same register several
        // times in sequence (bypass, reset, poll lock, un-bypass).
        let Some(op) = parse_op(line) else {
            continue;
        };
        match mode {
            ParsingMode::PllRev3 => pll_ops.push(op),
            ParsingMode::ClockRev3 => clock_ops.push(op),
            ParsingMode::DdrRev3 => ddr_ops.push(op),
            ParsingMode::PostConfigRev3 => post_config_ops.push(op),
            ParsingMode::MioRev3 => {
                let addr = op.addr();
                if DDRIOB_ADDR_RANGE.contains(&addr)
                    || addr == SLCR_UNLOCK_ADDR
                    || addr == SLCR_LOCK_ADDR
                {
                    ddriob_ops.push(op.clone());
                }
                mio_ops.push(op);
            }
        }
    }

    log::info!("generating DDRC config files: {}", DDRC_FILE_NAME);
    generate_ddrc_config(&reg_to_values, DDRC_FILE_NAME)?;

    log::info!("generating DDRIOB config files: {}", DDRIOB_FILE_NAME);
    generate_ddriob_config(&reg_to_values, DDRIOB_FILE_NAME)?;

    let ops_input = OpsFileInput {
        pll_ops: &pll_ops,
        clock_ops: &clock_ops,
        ddr_ops: &ddr_ops,
        ddriob_ops: &ddriob_ops,
        mio_ops: &mio_ops,
        post_config_ops: &post_config_ops,
        file_name: OPS_RON_FILE_NAME,
    };

    log::info!(
        "generating PS7 register op sequences: {}",
        ops_input.file_name
    );
    generate_ops_ron_file(ops_input)?;

    if cli.json {
        let ops_input = OpsFileInput {
            file_name: OPS_JSON_FILE_NAME,
            ..ops_input
        };
        log::info!(
            "generating PS7 register op sequences: {}",
            ops_input.file_name
        );
        generate_ops_json_file(ops_input)?;
    }

    if cli.rust {
        let ops_input = OpsFileInput {
            file_name: OPS_FILE_NAME,
            ..ops_input
        };
        log::info!(
            "generating PS7 register op sequences: {}",
            ops_input.file_name
        );
        generate_ops_file(ops_input)?;
    }

    Ok(())
}

fn generate_ddrc_config(
    reg_to_values: &RegisterToValueMap,
    file_name: &str,
) -> std::io::Result<()> {
    // Format as hex strings
    let ddrc = reg_to_values.val_as_token(DDRC_CTRL_ADDR);
    let two_rank = reg_to_values.val_as_token(0xF800_6004);
    let hpr = reg_to_values.val_as_token(0xF800_6008);
    let lpr = reg_to_values.val_as_token(0xF800_600C);
    let wr = reg_to_values.val_as_token(0xF800_6010);
    let dram_param_0 = reg_to_values.val_as_token(0xF800_6014);
    let dram_param_1 = reg_to_values.val_as_token(0xF800_6018);
    let dram_param_2 = reg_to_values.val_as_token(0xF800_601C);
    let dram_param_3 = reg_to_values.val_as_token(0xF800_6020);
    let dram_param_4 = reg_to_values.val_as_token(0xF800_6024);
    let dram_init_param = reg_to_values.val_as_token(0xF800_6028);
    let dram_emr = reg_to_values.val_as_token(0xF800_602C);
    let dram_emr_mr = reg_to_values.val_as_token(0xF800_6030);
    let dram_burst8_rdwr = reg_to_values.val_as_token(0xF800_6034);
    let dram_disable_dq = reg_to_values.val_as_token(0xF800_6038);
    let dram_addr_map_bank = reg_to_values.val_as_token(0xF800_603C);
    let dram_addr_map_col = reg_to_values.val_as_token(0xF800_6040);
    let dram_addr_map_row = reg_to_values.val_as_token(0xF800_6044);
    let dram_odt = reg_to_values.val_as_token(0xF800_6048);
    let phy_cmd_timeout_rddata_cpt = reg_to_values.val_as_token(0xF800_6050);
    let dll_calib = reg_to_values.val_as_token(0xF800_6058);
    let odt_delay_hold = reg_to_values.val_as_token(0xF800_605C);
    let ctrl_reg1 = reg_to_values.val_as_token(0xF800_6060);
    let ctrl_reg2 = reg_to_values.val_as_token(0xF800_6064);
    let ctrl_reg3 = reg_to_values.val_as_token(0xF800_6068);
    let ctrl_reg4 = reg_to_values.val_as_token(0xF800_606C);
    let ctrl_reg5 = reg_to_values.val_as_token(0xF800_6078);
    let ctrl_reg6 = reg_to_values.val_as_token(0xF800_607C);
    let che_t_zq = reg_to_values.val_as_token(0xF800_60A4);
    let che_t_zq_short_interval_reg = reg_to_values.val_as_token(0xF800_60A8);
    let deep_powerdown = reg_to_values.val_as_token(0xF800_60AC);
    let reg_2c = reg_to_values.val_as_token(0xF800_60B0);
    let reg_2d = reg_to_values.val_as_token(0xF800_60B4);
    let dfi_timing = reg_to_values.val_as_token(0xF800_60B8);
    let che_ecc_ctrl = reg_to_values.val_as_token(0xF800_60C4);
    let ecc_scrub = reg_to_values.val_as_token(0xF800_60F4);
    let phy_receiver_enable = reg_to_values.val_as_token(0xF800_6114);
    let phy_config_0 = reg_to_values.val_as_token(0xF800_6118);
    let phy_config_1 = reg_to_values.val_as_token(0xF800_611C);
    let phy_config_2 = reg_to_values.val_as_token(0xF800_6120);
    let phy_config_3 = reg_to_values.val_as_token(0xF800_6124);
    let phy_init_ratio_0 = reg_to_values.val_as_token(0xF800_612C);
    let phy_init_ratio_1 = reg_to_values.val_as_token(0xF800_6130);
    let phy_init_ratio_2 = reg_to_values.val_as_token(0xF800_6134);
    let phy_init_ratio_3 = reg_to_values.val_as_token(0xF800_6138);
    let phy_rd_dqs_config_0 = reg_to_values.val_as_token(0xF800_6140);
    let phy_rd_dqs_config_1 = reg_to_values.val_as_token(0xF800_6144);
    let phy_rd_dqs_config_2 = reg_to_values.val_as_token(0xF800_6148);
    let phy_rd_dqs_config_3 = reg_to_values.val_as_token(0xF800_614C);
    let phy_wr_dqs_config_0 = reg_to_values.val_as_token(0xF800_6154);
    let phy_wr_dqs_config_1 = reg_to_values.val_as_token(0xF800_6158);
    let phy_wr_dqs_config_2 = reg_to_values.val_as_token(0xF800_615C);
    let phy_wr_dqs_config_3 = reg_to_values.val_as_token(0xF800_6160);
    let phy_we_cfg_0 = reg_to_values.val_as_token(0xF800_6168);
    let phy_we_cfg_1 = reg_to_values.val_as_token(0xF800_616C);
    let phy_we_cfg_2 = reg_to_values.val_as_token(0xF800_6170);
    let phy_we_cfg_3 = reg_to_values.val_as_token(0xF800_6174);
    let phy_wr_data_slv_0 = reg_to_values.val_as_token(0xF800_617C);
    let phy_wr_data_slv_1 = reg_to_values.val_as_token(0xF800_6180);
    let phy_wr_data_slv_2 = reg_to_values.val_as_token(0xF800_6184);
    let phy_wr_data_slv_3 = reg_to_values.val_as_token(0xF800_6188);
    let reg64 = reg_to_values.val_as_token(0xF800_6190);
    let reg65 = reg_to_values.val_as_token(0xF800_6194);
    let page_mask = reg_to_values.val_as_token(0xF800_6204);
    let axi_priority_wr_port_0 = reg_to_values.val_as_token(0xF800_6208);
    let axi_priority_wr_port_1 = reg_to_values.val_as_token(0xF800_620C);
    let axi_priority_wr_port_2 = reg_to_values.val_as_token(0xF800_6210);
    let axi_priority_wr_port_3 = reg_to_values.val_as_token(0xF800_6214);
    let axi_priority_rd_port_0 = reg_to_values.val_as_token(0xF800_6218);
    let axi_priority_rd_port_1 = reg_to_values.val_as_token(0xF800_621C);
    let axi_priority_rd_port_2 = reg_to_values.val_as_token(0xF800_6220);
    let axi_priority_rd_port_3 = reg_to_values.val_as_token(0xF800_6224);
    let lpddr_ctrl_0 = reg_to_values.val_as_token(0xF800_62A8);
    let lpddr_ctrl_1 = reg_to_values.val_as_token(0xF800_62AC);
    let lpddr_ctrl_2 = reg_to_values.val_as_token(0xF800_62B0);
    let lpddr_ctrl_3 = reg_to_values.val_as_token(0xF800_62B4);

    let generated = quote::quote! {
        //!This file was auto-generated by the [z7-ps7init-extract](https://github.com/us-irs/zynq7000-rs/tree/main/host/z7-ps7init-extract) program.
        //!
        //!This configuration file contains static DDR configuration parameters extracted from the
        //!AMD ps7init.tcl file
        use zynq7000::ddrc::types;
        use zynq7000_hal::ddr::DdrcConfigSet;

        pub const DDRC_CONFIG_ZEDBOARD: DdrcConfigSet = DdrcConfigSet {
            ctrl: types::DdrcControl::new_with_raw_value(#ddrc),
            two_rank: types::TwoRankConfig::new_with_raw_value(#two_rank),
            hpr: types::LprHprQueueControl::new_with_raw_value(#hpr),
            lpr: types::LprHprQueueControl::new_with_raw_value(#lpr),
            wr: types::WriteQueueControl::new_with_raw_value(#wr),
            dram_param_0: types::DramParamReg0::new_with_raw_value(#dram_param_0),
            dram_param_1: types::DramParamReg1::new_with_raw_value(#dram_param_1),
            dram_param_2: types::DramParamReg2::new_with_raw_value(#dram_param_2),
            dram_param_3: types::DramParamReg3::new_with_raw_value(#dram_param_3),
            dram_param_4: types::DramParamReg4::new_with_raw_value(#dram_param_4),
            dram_init_param: types::DramInitParam::new_with_raw_value(#dram_init_param),
            dram_emr: types::DramEmr::new_with_raw_value(#dram_emr),
            dram_emr_mr: types::DramEmrMr::new_with_raw_value(#dram_emr_mr),
            dram_burst8_rdwr: types::DramBurst8ReadWrite::new_with_raw_value(#dram_burst8_rdwr),
            disable_dq: types::DisableDq::new_with_raw_value(#dram_disable_dq),
            dram_addr_map_bank: types::DramAddrMapBank::new_with_raw_value(#dram_addr_map_bank),
            dram_addr_map_col: types::DramAddrMapColumn::new_with_raw_value(#dram_addr_map_col),
            dram_addr_map_row: types::DramAddrMapRow::new_with_raw_value(#dram_addr_map_row),
            dram_odt: types::DramOdt::new_with_raw_value(#dram_odt),
            phy_cmd_timeout_rddata_cpt: types::PhyCmdTimeoutRdDataCpt::new_with_raw_value(#phy_cmd_timeout_rddata_cpt),
            dll_calib: types::DllCalib::new_with_raw_value(#dll_calib),
            odt_delay_hold: types::OdtDelayHold::new_with_raw_value(#odt_delay_hold),
            ctrl_reg1: types::CtrlReg1::new_with_raw_value(#ctrl_reg1),
            ctrl_reg2: types::CtrlReg2::new_with_raw_value(#ctrl_reg2),
            ctrl_reg3: types::CtrlReg3::new_with_raw_value(#ctrl_reg3),
            ctrl_reg4: types::CtrlReg4::new_with_raw_value(#ctrl_reg4),
            ctrl_reg5: types::CtrlReg5::new_with_raw_value(#ctrl_reg5),
            ctrl_reg6: types::CtrlReg6::new_with_raw_value(#ctrl_reg6),
            che_t_zq: types::CheTZq::new_with_raw_value(#che_t_zq),
            che_t_zq_short_interval_reg: types::CheTZqShortInterval::new_with_raw_value(#che_t_zq_short_interval_reg),
            deep_powerdown: types::DeepPowerdown::new_with_raw_value(#deep_powerdown),
            reg_2c: types::Reg2c::new_with_raw_value(#reg_2c),
            reg_2d: types::Reg2d::new_with_raw_value(#reg_2d),
            dfi_timing: types::DfiTiming::new_with_raw_value(#dfi_timing),
            che_ecc_ctrl: types::CheEccControl::new_with_raw_value(#che_ecc_ctrl),
            ecc_scrub: types::EccScrub::new_with_raw_value(#ecc_scrub),
            phy_receiver_enable: types::PhyReceiverEnable::new_with_raw_value(#phy_receiver_enable),
            phy_config: [
                types::PhyConfig::new_with_raw_value(#phy_config_0),
                types::PhyConfig::new_with_raw_value(#phy_config_1),
                types::PhyConfig::new_with_raw_value(#phy_config_2),
                types::PhyConfig::new_with_raw_value(#phy_config_3),
            ],
            phy_init_ratio: [
                types::PhyInitRatio::new_with_raw_value(#phy_init_ratio_0),
                types::PhyInitRatio::new_with_raw_value(#phy_init_ratio_1),
                types::PhyInitRatio::new_with_raw_value(#phy_init_ratio_2),
                types::PhyInitRatio::new_with_raw_value(#phy_init_ratio_3),
            ],
            phy_rd_dqs_config: [
                types::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_0),
                types::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_1),
                types::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_2),
                types::PhyDqsConfig::new_with_raw_value(#phy_rd_dqs_config_3),
            ],
            phy_wr_dqs_config: [
                types::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_0),
                types::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_1),
                types::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_2),
                types::PhyDqsConfig::new_with_raw_value(#phy_wr_dqs_config_3),
            ],
            phy_we_cfg: [
                types::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_0),
                types::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_1),
                types::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_2),
                types::PhyWriteEnableConfig::new_with_raw_value(#phy_we_cfg_3),
            ],
            phy_wr_data_slv: [
                types::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_0),
                types::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_1),
                types::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_2),
                types::PhyWriteDataSlaveConfig::new_with_raw_value(#phy_wr_data_slv_3),
            ],
            reg64: types::Reg64::new_with_raw_value(#reg64),
            reg65: types::Reg65::new_with_raw_value(#reg65),
            page_mask: #page_mask,
            axi_priority_wr_port: [
                types::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_0),
                types::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_1),
                types::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_2),
                types::AxiPriorityWritePort::new_with_raw_value(#axi_priority_wr_port_3),
            ],
            axi_priority_rd_port: [
                types::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_0),
                types::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_1),
                types::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_2),
                types::AxiPriorityReadPort::new_with_raw_value(#axi_priority_rd_port_3),
            ],
            lpddr_ctrl_0: types::LpddrControl0::new_with_raw_value(#lpddr_ctrl_0),
            lpddr_ctrl_1: types::LpddrControl1::new_with_raw_value(#lpddr_ctrl_1),
            lpddr_ctrl_2: types::LpddrControl2::new_with_raw_value(#lpddr_ctrl_2),
            lpddr_ctrl_3: types::LpddrControl3::new_with_raw_value(#lpddr_ctrl_3),
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
    let ddr_control = reg_to_values.val_as_token(0xF800_0B6C);
    let addr0 = reg_to_values.val_as_token(0xF800_0B40);
    let addr1 = reg_to_values.val_as_token(0xF800_0B44);
    let data0 = reg_to_values.val_as_token(0xF800_0B48);
    let data1 = reg_to_values.val_as_token(0xF800_0B4C);
    let diff0 = reg_to_values.val_as_token(0xF800_0B50);
    let diff1 = reg_to_values.val_as_token(0xF800_0B54);
    let clock = reg_to_values.val_as_token(0xF800_0B58);
    let generated = quote::quote! {
        //!This file was auto-generated by the [z7-ps7init-extract](https://github.com/us-irs/zynq7000-rs/tree/main/host/z7-ps7init-extract) program.
        //!
        //!This configuration file contains static DDRIOB configuration parameters extracted from the
        //!AMD ps7init.tcl file
        use zynq7000::ddrc::types;
        use zynq7000_hal::ddr::DdriobConfigSet;

        pub const DDRIOB_CONFIG_SET_ZEDBOARD: DdriobConfigSet = DdriobConfigSet {
            ddr_control: zynq7000::slcr::ddriob::DdrControl::new_with_raw_value(#ddr_control),
            addr0: types::DdriobConfig::new_with_raw_value(#addr0),
            addr1: types::DdriobConfig::new_with_raw_value(#addr1),
            data0: types::DdriobConfig::new_with_raw_value(#data0),
            data1: types::DdriobConfig::new_with_raw_value(#data1),
            diff0: types::DdriobConfig::new_with_raw_value(#diff0),
            diff1: types::DdriobConfig::new_with_raw_value(#diff1),
            clock: types::DdriobConfig::new_with_raw_value(#clock),
        };
    };

    std::fs::write(file_name, generated.to_string())?;
    Ok(())
}

fn hex_token(val: u32) -> proc_macro2::TokenStream {
    format!("{:#010x}", val).parse().unwrap()
}

fn reg_op_tokens(op: &RegOp) -> proc_macro2::TokenStream {
    match op.kind {
        RegOpKind::Write { addr, val } => {
            let addr = hex_token(addr);
            let val = hex_token(val);
            quote::quote! { RegOp::Write { addr: #addr, val: #val } }
        }
        RegOpKind::MaskWrite { addr, mask, val } => {
            let addr = hex_token(addr);
            let mask = hex_token(mask);
            let val = hex_token(val);
            quote::quote! { RegOp::MaskWrite { addr: #addr, mask: #mask, val: #val } }
        }
        RegOpKind::MaskPoll { addr, mask } => {
            let addr = hex_token(addr);
            let mask = hex_token(mask);
            quote::quote! { RegOp::MaskPoll { addr: #addr, mask: #mask } }
        }
    }
}

fn generate_ops_file(input: OpsFileInput) -> std::io::Result<()> {
    let pll_tokens: Vec<_> = input.pll_ops.iter().map(reg_op_tokens).collect();
    let clock_tokens: Vec<_> = input.clock_ops.iter().map(reg_op_tokens).collect();
    let ddr_tokens: Vec<_> = input.ddr_ops.iter().map(reg_op_tokens).collect();
    let ddriob_tokens: Vec<_> = input.ddriob_ops.iter().map(reg_op_tokens).collect();
    let mio_tokens: Vec<_> = input.mio_ops.iter().map(reg_op_tokens).collect();
    let post_config_tokens: Vec<_> = input.post_config_ops.iter().map(reg_op_tokens).collect();

    let generated = quote::quote! {
        //! This file was auto-generated by the [z7-ps7init-extract](https://github.com/us-irs/zynq7000-rs/tree/main/host/z7-ps7init-extract) program.
        //!
        //! This file contains the PS7 PLL/clock/DDR register init sequences extracted from the
        //! AMD ps7init.tcl file, in the exact order they need to be executed in. Unlike the
        //! DDRC/DDRIOB config structs, these are meant to be interpreted by a program that talks
        //! to the target directly (e.g. over JTAG via probe-rs), not compiled into on-target
        //! firmware. Register names (see the RON/JSON outputs) aren't carried over here.

        /// A single register operation, in the order it needs to be executed in.
        #[derive(Debug, Clone, Copy)]
        pub enum RegOp {
            /// Unconditional 32-bit register write.
            Write { addr: u32, val: u32 },
            /// Read-modify-write: `(read() & !mask) | (val & mask)`.
            MaskWrite { addr: u32, mask: u32, val: u32 },
            /// Poll the register until `read() & mask != 0`.
            MaskPoll { addr: u32, mask: u32 },
        }

        /// PLL bring-up sequence (ARM/DDR/IO PLL), extracted from `ps7_pll_init_data_3_0`.
        /// Includes the SLCR unlock/lock writes bracketing the sequence.
        pub const PLL_INIT_OPS: &[RegOp] = &[ #(#pll_tokens),* ];

        /// Clock control sequence, extracted from `ps7_clock_init_data_3_0`. Includes the SLCR
        /// unlock/lock writes bracketing the sequence.
        pub const CLOCK_INIT_OPS: &[RegOp] = &[ #(#clock_tokens),* ];

        /// DDR controller (DDRC) init sequence, extracted from `ps7_ddr_init_data_3_0`, including
        /// the trailing `mask_poll` steps that wait for DDR calibration/initialization to finish.
        pub const DDR_INIT_OPS: &[RegOp] = &[ #(#ddr_tokens),* ];

        /// The DDR-relevant subset of `MIO_INIT_OPS`, extracted from the DDRIOB/DCI register
        /// writes nested inside `ps7_mio_init_data_3_0`.
        pub const DDRIOB_INIT_OPS: &[RegOp] = &[ #(#ddriob_tokens),* ];

        /// The full `ps7_mio_init_data_3_0` proc body: general-purpose MIO pin muxing alongside
        /// the DDRIOB/DCI config also captured separately in `DDRIOB_INIT_OPS`.
        pub const MIO_INIT_OPS: &[RegOp] = &[ #(#mio_tokens),* ];

        /// Post-config sequence, extracted from `ps7_post_config_3_0`: enables the AXI level
        /// shifters and deasserts the PL reset, bracketed by the SLCR unlock/lock writes. Run
        /// this after `DDR_INIT_OPS` to put the PL out of its power-on reset state.
        pub const POST_CONFIG_OPS: &[RegOp] = &[ #(#post_config_tokens),* ];
    };

    std::fs::write(input.file_name, generated.to_string())?;
    Ok(())
}

fn generate_ops_ron_file(input: OpsFileInput) -> std::io::Result<()> {
    let ops = PsInitOps::from(&input);
    let ron = ron::ser::to_string_pretty(&ops, ron::ser::PrettyConfig::default())
        .expect("failed to serialize PS7 init ops to RON");
    std::fs::write(input.file_name, ron)
}

fn generate_ops_json_file(input: OpsFileInput) -> std::io::Result<()> {
    let ops = PsInitOps::from(&input);
    let json =
        serde_json::to_string_pretty(&ops).expect("failed to serialize PS7 init ops to JSON");
    std::fs::write(input.file_name, json)
}
