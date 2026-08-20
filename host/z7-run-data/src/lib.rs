//! Shared data model for the PS7 PLL/clock/DDR/DDRIOB register init sequences.
//!
//! [`z7-ps7init-extract`](https://github.com/us-irs/zynq7000-rs/tree/main/host/z7-ps7init-extract)
//! produces a [`PsInitOps`] value (serialized as RON, optionally JSON) from an AMD `ps7_init.tcl`
//! script. [`z7-run`](https://github.com/us-irs/zynq7000-rs/tree/main/host/z7-run)
//! deserializes it back and executes the op sequences over JTAG via probe-rs. Sharing this crate
//! between both keeps the two in sync instead of maintaining parallel type definitions.

use serde::{Deserialize, Serialize};

/// A single register operation, in the order it needs to be executed in, plus an optional
/// human-readable register name (e.g. "DDRC Control").
///
/// The name is purely informational: populated by `z7-ps7init-extract` when known, omitted from
/// the serialized output when not (`skip_serializing_if`), and ignored by `z7-run`'s execution
/// logic. It exists so RON/JSON files stay readable/greppable instead of being a wall of bare
/// addresses. Splitting it out into its own field here - rather than repeating a `name` field on
/// every [`RegOpKind`] variant - keeps that "common to every op" property common in the type too.
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
