use std::{collections::HashMap, ops::RangeInclusive, path::Path};

use clap::Parser as _;
use simple_logger::SimpleLogger;
use z7_run_data::{PsInitOps, RegOp, RegOpKind};

const DDRC_ADDR_RANGE: RangeInclusive<u32> = 0xf800_6000..=0xf800_62b4;
// Extends through 0xF8000B70 (DDRIOB_DCI_CTRL) and 0xF8000B74 (DDRIOB_DCI_STATUS): the impedance
// calibration (DCI) enable sequence for the DDR I/O pads lives right after the DDRIOB config
// registers proper, and was previously being cut off (range used to end at 0xF8000B6C), silently
// dropping the writes that actually kick off DCI calibration for the DDR pads' drive
// strength/termination.
const DDRIOB_ADDR_RANGE: RangeInclusive<u32> = 0xf800_0b40..=0xf800_0b74;

// PLL_INIT_OPS and CLOCK_INIT_OPS naturally include their own SLCR unlock/lock bracket, because
// those addresses fall inside the (unfiltered) PLL/clock proc bodies. ddriob_init_ops doesn't:
// it's filtered out of the much larger MIO proc body by DDRIOB_ADDR_RANGE, and the bracket
// addresses fall outside that range, so they'd otherwise get silently dropped - leaving
// ddriob_init_ops executed against a still-locked SLCR (a no-op write, not an error) whenever it
// runs before PLL_INIT_OPS unlocks it.
const SLCR_UNLOCK_ADDR: u32 = 0xF800_0008;
const SLCR_LOCK_ADDR: u32 = 0xF800_0004;

// Written twice in `ps7_ddr_init_data_3_0`: once early to configure the DDRC (controller not yet
// started), and again near the end to actually trigger DRAM init. The second write is a distinct,
// meaningful op (see `record_settled_value`), not a redundant repeat.
const DDRC_CTRL_ADDR: u32 = 0xF800_6000;

/// Human-readable names for registers this tool touches, so log messages and panics can point at
/// e.g. "DDRC Control" instead of a bare (and, until recently, decimal-formatted) address. Not
/// exhaustive - just the DDRC/DDRIOB registers the settled-config codegen names explicitly, plus
/// the handful of addresses called out by name elsewhere in this file.
const REGISTER_NAMES: &[(u32, &str)] = &[
    (DDRC_CTRL_ADDR, "DDRC Control"),
    (SLCR_UNLOCK_ADDR, "SLCR Unlock"),
    (SLCR_LOCK_ADDR, "SLCR Lock"),
    (0xF800_0B5C, "DDRIOB Drive Slew Addr"),
    (0xF800_0B60, "DDRIOB Drive Slew Data"),
    (0xF800_0B64, "DDRIOB Drive Slew Diff"),
    (0xF800_0B68, "DDRIOB Drive Slew Clock"),
    (0xF800_0B70, "DDRIOB DCI Control"),
    (0xF800_0B74, "DDRIOB DCI Status"),
    (0xF800_6004, "Two Rank"),
    (0xF800_6008, "HPR"),
    (0xF800_600C, "LPR"),
    (0xF800_6010, "WR"),
    (0xF800_6014, "DRAM Reg0"),
    (0xF800_6018, "DRAM Reg1"),
    (0xF800_601C, "DRAM Reg2"),
    (0xF800_6020, "DRAM Reg3"),
    (0xF800_6024, "DRAM Reg4"),
    (0xF800_6028, "DRAM Init Param"),
    (0xF800_602C, "DRAM EMR"),
    (0xF800_6030, "DRAM EMR MR"),
    (0xF800_6034, "DRAM Burst8 RDWR"),
    (0xF800_6038, "DRAM Disable DQ"),
    (0xF800_603C, "DRAM Addr Map Bank"),
    (0xF800_6040, "DRAM Addr Map Col"),
    (0xF800_6044, "DRAM Addr Map Row"),
    (0xF800_6048, "DRAM ODT"),
    (0xF800_6050, "PHY CMD Timeout"),
    (0xF800_6058, "DLL Calib"),
    (0xF800_605C, "ODT Delay Hold"),
    (0xF800_6060, "CTRL Reg 1"),
    (0xF800_6064, "CTRL Reg 2"),
    (0xF800_6068, "CTRL Reg 3"),
    (0xF800_606C, "CTRL Reg 4"),
    (0xF800_6078, "CTRL Reg 5"),
    (0xF800_607C, "CTRL Reg 6"),
    (0xF800_60A4, "CHE T ZQ"),
    (0xF800_60A8, "CHE T ZQ Short Interval"),
    (0xF800_60AC, "Deep Powerdown"),
    (0xF800_60B0, "Reg 2C"),
    (0xF800_60B4, "Reg 2D"),
    (0xF800_60B8, "DFI Timing"),
    (0xF800_60C4, "CHE ECC CTRL"),
    (0xF800_60F4, "ECC Scrub"),
    (0xF800_6114, "PHY Receiver Enable"),
    (0xF800_6118, "PHY Config 0"),
    (0xF800_611C, "PHY Config 1"),
    (0xF800_6120, "PHY Config 2"),
    (0xF800_6124, "PHY Config 3"),
    (0xF800_612C, "PHY Init Ratio 0"),
    (0xF800_6130, "PHY Init Ratio 1"),
    (0xF800_6134, "PHY Init Ratio 2"),
    (0xF800_6138, "PHY Init Ratio 3"),
    (0xF800_6140, "PHY RD DQS Config 0"),
    (0xF800_6144, "PHY RD DQS Config 1"),
    (0xF800_6148, "PHY RD DQS Config 2"),
    (0xF800_614C, "PHY RD DQS Config 3"),
    (0xF800_6154, "PHY WR DQS Config 0"),
    (0xF800_6158, "PHY WR DQS Config 1"),
    (0xF800_615C, "PHY WR DQS Config 2"),
    (0xF800_6160, "PHY WR DQS Config 3"),
    (0xF800_6168, "PHY WE Config 0"),
    (0xF800_616C, "PHY WE Config 1"),
    (0xF800_6170, "PHY WE Config 2"),
    (0xF800_6174, "PHY WE Config 3"),
    (0xF800_617C, "PHY WR Data Slv 0"),
    (0xF800_6180, "PHY WR Data Slv 1"),
    (0xF800_6184, "PHY WR Data Slv 2"),
    (0xF800_6188, "PHY WR Data Slv 3"),
    (0xF800_6190, "Reg64"),
    (0xF800_6194, "Reg65"),
    (0xF800_6204, "Page Mask"),
    (0xF800_6208, "AXI Priority WR Port 0"),
    (0xF800_620C, "AXI Priority WR Port 1"),
    (0xF800_6210, "AXI Priority WR Port 2"),
    (0xF800_6214, "AXI Priority WR Port 3"),
    (0xF800_6218, "AXI Priority RD Port 0"),
    (0xF800_621C, "AXI Priority RD Port 1"),
    (0xF800_6220, "AXI Priority RD Port 2"),
    (0xF800_6224, "AXI Priority RD Port 3"),
    (0xF800_62A8, "LPDDR CTRL 0"),
    (0xF800_62AC, "LPDDR CTRL 1"),
    (0xF800_62B0, "LPDDR CTRL 2"),
    (0xF800_62B4, "LPDDR CTRL 3"),
    (0xF800_0B6C, "DDRIOB DDR Control"),
    (0xF800_0B40, "DDRIOB Addr 0"),
    (0xF800_0B44, "DDRIOB Addr 1"),
    (0xF800_0B48, "DDRIOB Data 0"),
    (0xF800_0B4C, "DDRIOB Data 1"),
    (0xF800_0B50, "DDRIOB Diff 0"),
    (0xF800_0B54, "DDRIOB Diff 1"),
    (0xF800_0B58, "DDRIOB Clock"),
];

/// `REGISTER_NAMES` as an actual O(1) lookup table, built once on first use.
static REGISTER_NAME_MAP: std::sync::LazyLock<HashMap<u32, &'static str>> =
    std::sync::LazyLock::new(|| REGISTER_NAMES.iter().copied().collect());

/// Looks up the human-readable name for a register address, if this tool has a name for it.
#[inline]
fn register_name(addr: u32) -> Option<&'static str> {
    REGISTER_NAME_MAP.get(&addr).copied()
}

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

fn extract_all_hex(line: &str) -> Vec<u32> {
    let re = regex::Regex::new(r"0[xX]([0-9A-Fa-f]+)").unwrap();

    re.captures_iter(line)
        .filter_map(|cap| u32::from_str_radix(&cap[1], 16).ok())
        .collect()
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

/// Parses a single ps7init line into a [`RegOp`], based on which command/macro it uses. Unlike
/// [`extract_hex_values`], this looks at the keyword rather than just the number of hex literals
/// on the line, since a 2-value line is ambiguous between a plain write and a mask poll. The
/// resulting op's `name` is looked up immediately, since the address is already at hand here.
fn parse_op(line: &str) -> Option<RegOp> {
    let hex = extract_all_hex(line);
    let kind = if (line.contains("mask_write") || line.contains("EMIT_MASKWRITE")) && hex.len() == 3
    {
        RegOpKind::MaskWrite {
            addr: hex[0],
            mask: hex[1],
            val: hex[2],
        }
    } else if (line.contains("mask_poll") || line.contains("EMIT_MASKPOLL")) && hex.len() == 2 {
        RegOpKind::MaskPoll {
            addr: hex[0],
            mask: hex[1],
        }
    } else if (line.contains("mwr") || line.contains("EMIT_WRITE")) && hex.len() == 2 {
        RegOpKind::Write {
            addr: hex[0],
            val: hex[1],
        }
    } else {
        return None;
    };
    Some(RegOp::new(
        kind,
        register_name(kind.addr()).map(String::from),
    ))
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

/// Which `ps7_*_init_data_3_0` proc body the line-by-line scan is currently inside, if any.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ParsingMode {
    DdrRev3,
    MioRev3,
    PllRev3,
    ClockRev3,
}

impl ParsingMode {
    /// Returns the mode a line switches into, if it's the opening line of one of the
    /// `ps7_*_init_data_3_0` procs.
    fn detect(line: &str) -> Option<Self> {
        if line.contains("ps7_ddr_init_data_3_0") {
            Some(Self::DdrRev3)
        } else if line.contains("ps7_mio_init_data_3_0") {
            Some(Self::MioRev3)
        } else if line.contains("ps7_pll_init_data_3_0") {
            Some(Self::PllRev3)
        } else if line.contains("ps7_clock_init_data_3_0") {
            Some(Self::ClockRev3)
        } else {
            None
        }
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
        //!This file was auto-generated by the [z7-ps7init-extract](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/host/z7-ps7init-extract) program.
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
    let ddr_control = reg_to_values.val_as_token(0xF800_0B6C);
    let addr0 = reg_to_values.val_as_token(0xF800_0B40);
    let addr1 = reg_to_values.val_as_token(0xF800_0B44);
    let data0 = reg_to_values.val_as_token(0xF800_0B48);
    let data1 = reg_to_values.val_as_token(0xF800_0B4C);
    let diff0 = reg_to_values.val_as_token(0xF800_0B50);
    let diff1 = reg_to_values.val_as_token(0xF800_0B54);
    let clock = reg_to_values.val_as_token(0xF800_0B58);
    let generated = quote::quote! {
        //!This file was auto-generated by the [z7-ps7init-extract](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/host/z7-ps7init-extract) program.
        //!
        //!This configuration file contains static DDRIOB configuration parameters extracted from the
        //!AMD ps7init.tcl file
        use zynq7000::ddrc::regs;
        use zynq7000_hal::ddr::DdriobConfigSet;

        pub const DDRIOB_CONFIG_SET_ZEDBOARD: DdriobConfigSet = DdriobConfigSet {
            ddr_control: zynq7000::slcr::ddriob::DdrControl::new_with_raw_value(#ddr_control),
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

    let generated = quote::quote! {
        //! This file was auto-generated by the [z7-ps7init-extract](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/host/z7-ps7init-extract) program.
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
