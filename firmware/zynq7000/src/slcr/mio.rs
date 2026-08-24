//! # SLCR MIO (Multiplexed I/O) configuration registers
//!
//! Writing any of these registers required unlocking the SLCR first.
use arbitrary_int::{u2, u3};

/// Output edge speed for a MIO pin.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// Slow CMOS edge rate.
    SlowCmosEdge = 0b0,
    /// Fast CMOS edge rate.
    FastCmosEdge = 0b1,
}

/// I/O signal standard for a MIO pin.
#[bitbybit::bitenum(u3)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IoType {
    /// LVCMOS 1.8V.
    LvCmos18 = 0b001,
    /// LVCMOS 2.5V.
    LvCmos25 = 0b010,
    /// LVCMOS 3.3V.
    LvCmos33 = 0b011,
    /// HSTL.
    Hstl = 0b100,
}

/// Per-pin MIO configuration, including muxing, I/O standard and tri-state control.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
#[derive(PartialEq, Eq)]
pub struct Config {
    /// Disables the HSTL input receiver.
    #[bit(13, rw)]
    disable_hstl_rcvr: bool,
    /// Enables the internal pull-up.
    #[bit(12, rw)]
    pullup: bool,
    /// I/O signal standard.
    #[bits(9..=11, rw)]
    io_type: Option<IoType>,
    /// Output edge speed.
    #[bit(8, rw)]
    speed: Speed,
    /// L3 mux select, chooses the pin's function at the third mux level.
    #[bits(5..=7, rw)]
    l3_sel: u3,
    /// L2 mux select, chooses the pin's function at the second mux level.
    #[bits(3..=4, rw)]
    l2_sel: u2,
    /// L1 mux select, chooses the pin's function at the first mux level.
    #[bit(2, rw)]
    l1_sel: bool,
    /// L0 mux select, chooses between the L1 path and the GPIO function.
    #[bit(1, rw)]
    l0_sel: bool,
    /// Tri-state enable, disables the output driver when set.
    #[bit(0, rw)]
    tri_enable: bool,
}
