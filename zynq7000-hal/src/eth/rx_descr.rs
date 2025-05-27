//! RX buffer descriptor module.
pub use super::shared::Ownership;
use arbitrary_int::{u2, u3, u13, u30};

/// RX buffer descriptor.
///
/// The user should declare an array of this structure inside uncached memory.
///
/// These descriptors are shared between software and hardware and contain information
/// related to frame reception.
#[repr(C)]
pub struct Descriptor {
    /// The first word of the descriptor.
    pub word0: Word0,
    /// The second word of the descriptor.
    pub word1: Word1,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug, PartialEq, Eq)]
pub struct Word0 {
    /// The full reception address with the last two bits cleared.
    #[bits(2..=31, rw)]
    addr_upper_30_bits: u30,
    #[bit(1, rw)]
    wrap: bool,
    #[bit(0, rw)]
    ownership: Ownership,
}

/// This control word contains the status bits.
///
/// If the end of frame bit (15) is not set, the only valid status information is the start of
/// frame bit.
#[bitbybit::bitfield(u32)]
#[derive(Debug, PartialEq, Eq)]
pub struct Word1 {
    #[bit(31, r)]
    broadcast_detect: bool,
    #[bit(30, r)]
    multicast_hash: bool,
    #[bit(29, r)]
    unicast_hash: bool,
    #[bit(27, r)]
    specific_addr_match: bool,
    /// Specifies which of the 4 specific address registers was matched.
    #[bits(25..=26, r)]
    specific_addr_match_info: u2,
    #[bit(24, r)]
    type_id_match_or_snap_info: bool,
    #[bits(22..=23, r)]
    type_id_match_info_or_chksum_status: u2,
    #[bit(21, r)]
    vlan_tag_detected: bool,
    #[bit(20, r)]
    priority_tag_detected: bool,
    #[bits(17..=19, r)]
    vlan_prio: u3,
    #[bit(16, r)]
    cfi_bit: bool,
    /// If this bit is not set, the only other valid status bit is the start of frame bit.
    #[bit(15, r)]
    end_of_frame: bool,
    #[bit(14, r)]
    start_of_frame: bool,
    /// Relevant when FCS errors are not ignored.
    /// 0: Frame has good FCS, 1: Frame has bad FCS, but was copied to memory as the ignore FCS
    /// functionality was enabled.
    #[bit(13, r)]
    fcs_status: bool,
    #[bits(0..=12, r)]
    rx_len: u13,
}

impl Descriptor {
    #[inline]
    pub const fn new() -> Self {
        Self {
            word0: Word0::new_with_raw_value(0),
            word1: Word1::new_with_raw_value(0),
        }
    }

    #[inline]
    pub fn set_ownership(&mut self, ownership: Ownership) {
        self.word0.set_ownership(ownership);
    }

    #[inline]
    pub fn set_wrap_bit(&mut self) {
        self.word0.set_wrap(true);
    }

    #[inline]
    pub fn write_rx_addr(&mut self, addr: u32) {
        self.word0.set_addr_upper_30_bits(u30::new(addr >> 2));
    }
}

impl Default for Descriptor {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

pub struct DescriptorListRef<'a>(&'a mut [Descriptor]);

impl<'a> DescriptorListRef<'a> {
    #[inline]
    pub fn new(descriptor: &'a mut [Descriptor]) -> Self {
        Self(descriptor)
    }
}

impl DescriptorListRef<'_> {
    #[inline]
    pub fn base_ptr(&self) -> *const Descriptor {
        self.0.as_ptr()
    }

    #[inline]
    pub fn base_addr(&self) -> u32 {
        self.base_ptr() as u32
    }

    pub fn init(&mut self) {
        for desc in self.0.iter_mut() {
            desc.set_ownership(Ownership::Hardware);
        }
        self.0.last_mut().unwrap().set_wrap_bit();
    }

    pub fn set_rx_buf_address(&mut self, index: usize, addr: u32) {
        self.0[index].write_rx_addr(addr);
    }
}
