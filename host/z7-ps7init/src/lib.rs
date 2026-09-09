//! Data model and parser for the PS7 PLL/clock/DDR/DDRIOB register init sequences that AMD's
//! Vivado tooling emits as a `ps7_init.tcl` script.
//!
//! [`parse_ps7_init_tcl`] turns that script into a [`PsInitOps`] directly.
//! [`z7-ps7init-extract`](https://github.com/us-irs/zynq7000-rs/tree/main/host/z7-ps7init-extract)
//! uses the same parsing primitives to also emit RON/JSON/Rust versions of it, plus DDRC/DDRIOB
//! Rust config structs for on-target firmware.
//! [`z7-run`](https://github.com/us-irs/zynq7000-rs/tree/main/host/z7-run) executes a
//! [`PsInitOps`] over JTAG via probe-rs, built either way. Sharing this crate keeps the two in
//! sync instead of duplicating the parser or the types.

use std::{collections::HashMap, ops::RangeInclusive};

use serde::{Deserialize, Serialize};

/// A single register operation after it was extracted from the initialization file.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegOp {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    pub kind: RegOpKind,
}

impl RegOp {
    #[inline]
    pub const fn new(kind: RegOpKind, name: Option<String>) -> Self {
        Self { name, kind }
    }

    /// The address this op targets, regardless of variant.
    #[inline]
    pub const fn addr(&self) -> u32 {
        self.kind.addr()
    }
}

/// Operation kind.
///
/// The operation itself: mirrors the AMD `EMIT_*`/`mask_write`/`mask_poll` vocabulary closely
/// enough to be executed verbatim instead of only capturing the final settled register value.
/// This matters for registers like the PLL control registers, which are written multiple times
/// in sequence (bypass -> assert reset -> deassert reset -> poll lock -> remove bypass) before
/// they reach their final value; collapsing that down to a single write would skip the reset
/// pulse the PLL needs to relock.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum RegOpKind {
    /// Unconditional 32-bit register write (`mwr -force` / `EMIT_WRITE`).
    Write {
        #[serde(with = "hex_u32")]
        addr: u32,
        #[serde(with = "hex_u32")]
        val: u32,
    },
    /// Read-modify-write: `(read() & !mask) | (val & mask)` (`mask_write` / `EMIT_MASKWRITE`).
    MaskWrite {
        #[serde(with = "hex_u32")]
        addr: u32,
        #[serde(with = "hex_u32")]
        mask: u32,
        #[serde(with = "hex_u32")]
        val: u32,
    },
    /// Poll the register until `read() & mask != 0` (`mask_poll` / `EMIT_MASKPOLL`).
    MaskPoll {
        #[serde(with = "hex_u32")]
        addr: u32,
        #[serde(with = "hex_u32")]
        mask: u32,
    },
}

impl RegOpKind {
    /// The address this op targets, regardless of variant.
    pub const fn addr(&self) -> u32 {
        match *self {
            RegOpKind::Write { addr, .. }
            | RegOpKind::MaskWrite { addr, .. }
            | RegOpKind::MaskPoll { addr, .. } => addr,
        }
    }
}

/// Bundles the PS7 init op sequences extracted from an AMD `ps7_init.tcl` script.
///
/// These are meant to be interpreted by a program that talks to the target directly (e.g. over
/// JTAG via probe-rs), not compiled into on-target firmware.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PsInitOps {
    /// PLL bring-up sequence (ARM/DDR/IO PLL), extracted from `ps7_pll_init_data_3_0`. Includes
    /// the SLCR unlock/lock writes bracketing the sequence.
    pub pll_init_ops: Vec<RegOp>,
    /// Clock control sequence, extracted from `ps7_clock_init_data_3_0`. Includes the SLCR
    /// unlock/lock writes bracketing the sequence.
    pub clock_init_ops: Vec<RegOp>,
    /// DDR controller (DDRC) init sequence, extracted from `ps7_ddr_init_data_3_0`, including the
    /// trailing `mask_poll` steps that wait for DDR calibration/initialization to finish.
    pub ddr_init_ops: Vec<RegOp>,
    /// The DDR-relevant subset of `mio_init_ops`: everything in that proc *except* the
    /// general-purpose MIO pin muxing block, which is unrelated to DDR bring-up.
    /// This is a subset of the MIO initialization operation.
    pub ddriob_init_ops: Vec<RegOp>,
    /// The full `ps7_mio_init_data_3_0` proc body, captured unconditionally (same as the three
    /// lists above): general-purpose MIO pin muxing (UART/GPIO/SPI/etc.) alongside the DDRIOB I/O
    /// buffer config, DCI impedance calibration, and the SLCR unlock/lock bracket.
    pub mio_init_ops: Vec<RegOp>,
    /// Post-config sequence, extracted from `ps7_post_config_3_0`: enables the AXI level
    /// shifters (`LVL_SHFTR_EN`) and deasserts the PL reset (`FPGA_RST_CTRL`), bracketed by the
    /// SLCR unlock/lock writes. The PL comes up in reset after power-on; on real hardware this is
    /// called right after `ps7_init` (independent of whether/when a bitstream gets loaded), which
    /// is why it's kept as its own op list rather than folded into `ddr_init_ops`.
    pub post_config_ops: Vec<RegOp>,
}

// PLL_INIT_OPS and CLOCK_INIT_OPS naturally include their own SLCR unlock/lock bracket, because
// those addresses fall inside the (unfiltered) PLL/clock proc bodies. ddriob_init_ops doesn't:
// it's filtered out of the much larger MIO proc body by DDRIOB_ADDR_RANGE, and the bracket
// addresses fall outside that range, so they'd otherwise get silently dropped - leaving
// ddriob_init_ops executed against a still-locked SLCR (a no-op write, not an error) whenever it
// runs before PLL_INIT_OPS unlocks it.
pub const SLCR_UNLOCK_ADDR: u32 = 0xF800_0008;
pub const SLCR_LOCK_ADDR: u32 = 0xF800_0004;
const LVL_SHFTR_EN_ADDR: u32 = 0xF800_0900;
const FPGA_RST_CTRL_ADDR: u32 = 0xF800_0240;
const DDRC_CTRL_ADDR: u32 = 0xF800_6000;

// Extends through 0xF8000B70 (DDRIOB_DCI_CTRL) and 0xF8000B74 (DDRIOB_DCI_STATUS): the impedance
// calibration (DCI) enable sequence for the DDR I/O pads lives right after the DDRIOB config
// registers proper. Starts at 0xF8000B00 (GPIOB_CTRL), not 0xF8000B40: GPIOB_CTRL's VREF_EN bit
// has to be set before the DCI Control trigger below runs, since DCI calibration only runs once,
// using whatever VREF state exists at trigger time.
pub const DDRIOB_ADDR_RANGE: RangeInclusive<u32> = 0xf800_0b00..=0xf800_0b74;

// `mio_pins` in `zynq7000::slcr::Registers`: 54 identical 32-bit pin config registers, named
// MIO_PIN_00..MIO_PIN_53 individually rather than listed out in REGISTER_NAMES.
const MIO_PIN_ADDR_RANGE: RangeInclusive<u32> = 0xF800_0700..=0xF800_07D4;

/// Human-readable names for registers this crate can name, so log messages and panics can point
/// at e.g. "DDRC Control" instead of a bare address. Not exhaustive - just the DDRC/DDRIOB
/// registers `z7-ps7init-extract`'s settled-config codegen names explicitly, plus the handful of
/// addresses called out by name elsewhere.
const REGISTER_NAMES: &[(u32, &str)] = &[
    (DDRC_CTRL_ADDR, "DDRC Control"),
    (SLCR_UNLOCK_ADDR, "SLCR Unlock"),
    (SLCR_LOCK_ADDR, "SLCR Lock"),
    (LVL_SHFTR_EN_ADDR, "LVL_SHFTR_EN"),
    (FPGA_RST_CTRL_ADDR, "FPGA_RST_CTRL"),
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
    // SLCR PLL/clock control block (base 0xF8000100, `zynq7000::slcr::clocks::ClockControlRegisters`).
    (0xF800_0100, "ARM_PLL_CTRL"),
    (0xF800_0104, "DDR_PLL_CTRL"),
    (0xF800_0108, "IO_PLL_CTRL"),
    (0xF800_010C, "PLL_STATUS"),
    (0xF800_0110, "ARM_PLL_CFG"),
    (0xF800_0114, "DDR_PLL_CFG"),
    (0xF800_0118, "IO_PLL_CFG"),
    (0xF800_0120, "ARM_CLK_CTRL"),
    (0xF800_0124, "DDR_CLK_CTRL"),
    (0xF800_0128, "DCI_CLK_CTRL"),
    (0xF800_012C, "APER_CLK_CTRL"),
    (0xF800_0130, "USB0_CLK_CTRL"),
    (0xF800_0134, "USB1_CLK_CTRL"),
    (0xF800_0138, "GEM0_RCLK_CTRL"),
    (0xF800_013C, "GEM1_RCLK_CTRL"),
    (0xF800_0140, "GEM0_CLK_CTRL"),
    (0xF800_0144, "GEM1_CLK_CTRL"),
    (0xF800_0148, "SMC_CLK_CTRL"),
    (0xF800_014C, "LQSPI_CLK_CTRL"),
    (0xF800_0150, "SDIO_CLK_CTRL"),
    (0xF800_0154, "UART_CLK_CTRL"),
    (0xF800_0158, "SPI_CLK_CTRL"),
    (0xF800_015C, "CAN_CLK_CTRL"),
    (0xF800_0160, "CAN_MIOCLK_CTRL"),
    (0xF800_0164, "DBG_CLK_CTRL"),
    (0xF800_0168, "PCAP_CLK_CTRL"),
    (0xF800_016C, "TOPSW_CLK_CTRL"),
    (0xF800_0170, "FPGA0_CLK_CTRL"),
    (0xF800_0174, "FPGA0_THR_CTRL"),
    (0xF800_0178, "FPGA0_THR_CNT"),
    (0xF800_017C, "FPGA0_THR_STA"),
    (0xF800_0180, "FPGA1_CLK_CTRL"),
    (0xF800_0184, "FPGA1_THR_CTRL"),
    (0xF800_0188, "FPGA1_THR_CNT"),
    (0xF800_018C, "FPGA1_THR_STA"),
    (0xF800_0190, "FPGA2_CLK_CTRL"),
    (0xF800_0194, "FPGA2_THR_CTRL"),
    (0xF800_0198, "FPGA2_THR_CNT"),
    (0xF800_019C, "FPGA2_THR_STA"),
    (0xF800_01A0, "FPGA3_CLK_CTRL"),
    (0xF800_01A4, "FPGA3_THR_CTRL"),
    (0xF800_01A8, "FPGA3_THR_CNT"),
    (0xF800_01AC, "FPGA3_THR_STA"),
    (0xF800_01C4, "CLK_621_TRUE"),
    // SLCR reset control block (base 0xF8000200, `zynq7000::slcr::reset::ResetControl`).
    (0xF800_0200, "PSS_RST_CTRL"),
    (0xF800_0204, "DDR_RST_CTRL"),
    (0xF800_0208, "TOPSW_RESET_CTRL"),
    (0xF800_020C, "DMAC_RST_CTRL"),
    (0xF800_0210, "USB_RST_CTRL"),
    (0xF800_0214, "GEM_RST_CTRL"),
    (0xF800_0218, "SDIO_RST_CTRL"),
    (0xF800_021C, "SPI_RST_CTRL"),
    (0xF800_0220, "CAN_RST_CTRL"),
    (0xF800_0224, "I2C_RST_CTRL"),
    (0xF800_0228, "UART_RST_CTRL"),
    (0xF800_022C, "GPIO_RST_CTRL"),
    (0xF800_0230, "LQSPI_RST_CTRL"),
    (0xF800_0234, "SMC_RST_CTRL"),
    (0xF800_0238, "OCM_RST_CTRL"),
    (0xF800_0244, "A9_CPU_RST_CTRL"),
    (0xF800_024C, "RS_AWDT_CTRL"),
    // MIO/GPIOB registers surrounding the MIO_PIN_NN block (`zynq7000::slcr::Registers`).
    (0xF800_0804, "MIO_LOOPBACK"),
    (0xF800_080C, "MIO_MST_TRI0"),
    (0xF800_0810, "MIO_MST_TRI1"),
    (0xF800_0830, "SD0_WP_CD_SEL"),
    (0xF800_0834, "SD1_WP_CD_SEL"),
    // GPIOB block (base 0xF8000B00, `zynq7000::slcr::GpiobRegisters`).
    (0xF800_0B00, "GPIOB_CTRL"),
    (0xF800_0B04, "GPIOB_CFG_CMOS18"),
    (0xF800_0B08, "GPIOB_CFG_CMOS25"),
    (0xF800_0B0C, "GPIOB_CFG_CMOS33"),
    (0xF800_0B14, "GPIOB_CFG_HSTL"),
    (0xF800_0B18, "GPIOB_DRVR_BIAS_CTRL"),
];

/// `REGISTER_NAMES` as an actual O(1) lookup table, built once on first use.
static REGISTER_NAME_MAP: std::sync::LazyLock<HashMap<u32, &'static str>> =
    std::sync::LazyLock::new(|| REGISTER_NAMES.iter().copied().collect());

/// Looks up the human-readable name for a register address, if this crate has a name for it.
#[inline]
pub fn register_name(addr: u32) -> Option<&'static str> {
    REGISTER_NAME_MAP.get(&addr).copied()
}

/// Looks up the human-readable name for a register address, same as [`register_name`], but also
/// resolves addresses inside `MIO_PIN_ADDR_RANGE` to `MIO_PIN_NN`. Kept separate from
/// [`register_name`] since those names are computed rather than static, so they can't live in
/// the lookup table.
fn resolve_register_name(addr: u32) -> Option<String> {
    if let Some(name) = register_name(addr) {
        return Some(name.to_string());
    }
    if MIO_PIN_ADDR_RANGE.contains(&addr) && (addr - MIO_PIN_ADDR_RANGE.start()).is_multiple_of(4) {
        let pin = (addr - MIO_PIN_ADDR_RANGE.start()) / 4;
        return Some(format!("MIO_PIN_{pin:02}"));
    }
    None
}

/// Extracts every `0x`/`0X`-prefixed hex literal from a `ps7_init.tcl` line, in the order they
/// appear.
pub fn extract_all_hex(line: &str) -> Vec<u32> {
    let re = regex::Regex::new(r"0[xX]([0-9A-Fa-f]+)").unwrap();

    re.captures_iter(line)
        .filter_map(|cap| u32::from_str_radix(&cap[1], 16).ok())
        .collect()
}

/// Parses a single `ps7_init.tcl` line into a [`RegOp`], based on which command/macro it uses.
/// The resulting op's `name` is looked up immediately, since the address is already at hand here.
pub fn parse_op(line: &str) -> Option<RegOp> {
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
    Some(RegOp::new(kind, resolve_register_name(kind.addr())))
}

/// Which `ps7_*_init_data_3_0` proc body a `ps7_init.tcl` line-by-line scan is currently inside,
/// if any.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ParsingMode {
    DdrRev3,
    MioRev3,
    PllRev3,
    ClockRev3,
    PostConfigRev3,
}

impl ParsingMode {
    /// Returns the mode a line switches into, if it's the opening line of one of the
    /// `ps7_*_init_data_3_0`/`ps7_post_config_3_0` procs.
    pub fn detect(line: &str) -> Option<Self> {
        if line.contains("ps7_ddr_init_data_3_0") {
            Some(Self::DdrRev3)
        } else if line.contains("ps7_mio_init_data_3_0") {
            Some(Self::MioRev3)
        } else if line.contains("ps7_pll_init_data_3_0") {
            Some(Self::PllRev3)
        } else if line.contains("ps7_clock_init_data_3_0") {
            Some(Self::ClockRev3)
        } else if line.contains("ps7_post_config_3_0") {
            Some(Self::PostConfigRev3)
        } else {
            None
        }
    }
}

/// Parses an AMD `ps7_init.tcl` script directly into a [`PsInitOps`].
pub fn parse_ps7_init_tcl(tcl: &str) -> PsInitOps {
    let mut parsing_mode = None;

    let mut ops = PsInitOps::default();

    for line in tcl.lines() {
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

        // Ops are collected in source order and are not deduplicated: PLL bring-up in
        // particular relies on writing the same register several times in sequence (bypass,
        // reset, poll lock, un-bypass).
        let Some(op) = parse_op(line) else {
            continue;
        };
        match mode {
            ParsingMode::PllRev3 => ops.pll_init_ops.push(op),
            ParsingMode::ClockRev3 => ops.clock_init_ops.push(op),
            ParsingMode::DdrRev3 => ops.ddr_init_ops.push(op),
            ParsingMode::PostConfigRev3 => ops.post_config_ops.push(op),
            ParsingMode::MioRev3 => {
                let addr = op.addr();
                if DDRIOB_ADDR_RANGE.contains(&addr)
                    || addr == SLCR_UNLOCK_ADDR
                    || addr == SLCR_LOCK_ADDR
                {
                    ops.ddriob_init_ops.push(op.clone());
                }
                ops.mio_init_ops.push(op);
            }
        }
    }

    ops
}

/// (De)serializes a `u32` as a `0x`-prefixed hex string, so RON/JSON files stay readable instead
/// of showing register addresses/masks/values as plain decimal numbers.
mod hex_u32 {
    use serde::{Deserialize, Deserializer, Serializer, de::Error};

    pub fn serialize<S: Serializer>(val: &u32, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&format!("{val:#010x}"))
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u32, D::Error> {
        let s = String::deserialize(deserializer)?;
        let digits = s
            .strip_prefix("0x")
            .or_else(|| s.strip_prefix("0X"))
            .ok_or_else(|| D::Error::custom(format!("expected 0x-prefixed hex string, got {s}")))?;
        u32::from_str_radix(digits, 16).map_err(D::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_through_ron() {
        let ops = PsInitOps {
            pll_init_ops: vec![RegOp {
                name: Some("SLCR Unlock".to_string()),
                kind: RegOpKind::Write {
                    addr: 0xf800_0008,
                    val: 0x0000_df0d,
                },
            }],
            clock_init_ops: vec![],

            ddr_init_ops: vec![RegOp {
                name: None,
                kind: RegOpKind::MaskPoll {
                    addr: 0xf800_6054,
                    mask: 0x0000_0001,
                },
            }],
            ddriob_init_ops: vec![],
            mio_init_ops: vec![],
            post_config_ops: vec![],
        };
        let ron = ron::ser::to_string_pretty(&ops, ron::ser::PrettyConfig::default()).unwrap();
        let parsed: PsInitOps = ron::from_str(&ron).unwrap();

        assert_eq!(parsed.pll_init_ops[0].name.as_deref(), Some("SLCR Unlock"));
        match &parsed.pll_init_ops[0].kind {
            RegOpKind::Write { addr, val } => {
                assert_eq!(*addr, 0xf800_0008);
                assert_eq!(*val, 0x0000_df0d);
            }
            _ => panic!("unexpected variant"),
        }
    }
}
