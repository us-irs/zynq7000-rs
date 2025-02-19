//! # SLCR MIO (Multiplexed I/O) configuration registers
//!
//! Writing any of these registers required unlocking the SLCR first.
use arbitrary_int::{u2, u3};

#[bitbybit::bitenum(u1, exhaustive = true)]
pub enum Speed {
    SlowCmosEdge = 0b0,
    FastCmosEdge = 0b1,
}

#[bitbybit::bitenum(u3)]
pub enum IoType {
    LvCmos18 = 0b001,
    LvCmos25 = 0b010,
    LvCmos33 = 0b011,
    Hstl = 0b100,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct Config {
    #[bit(13, rw)]
    disable_hstl_rcvr: bool,
    #[bit(12, rw)]
    pullup: bool,
    #[bits(9..=11, rw)]
    io_type: Option<IoType>,
    #[bit(8, rw)]
    speed: Speed,
    #[bits(5..=7, rw)]
    l3_sel: u3,
    #[bits(3..=4, rw)]
    l2_sel: u2,
    #[bit(2, rw)]
    l1_sel: bool,
    #[bit(1, rw)]
    l0_sel: bool,
    #[bit(0, rw)]
    tri_enable: bool,
}
