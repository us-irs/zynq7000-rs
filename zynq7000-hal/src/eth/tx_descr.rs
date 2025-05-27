use arbitrary_int::u14;

pub use super::shared::Ownership;

/// TX buffer descriptor.
///
/// The user should declare an array of this structure inside uncached memory.
///
/// These descriptors are shared between software and hardware and contain information
/// related to frame reception.
#[repr(C)]
pub struct Descriptor {
    /// The first word of the descriptor which is the byte address of the buffer.
    pub word0: u32,
    /// The second word of the descriptor.
    pub word1: Word1,
}

#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum TransmitChecksumGenerationStatus {
    NoError = 0b000,
    VlanError = 0b001,
    SnapError = 0b010,
    IpError = 0b011,
    NotVlanOrSnapOrIp = 0b100,
    NonSupportedPacketFragmentation = 0b101,
    PacketNotTcpUdp = 0b110,
    PrematureEndOfFrame = 0b111,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug, PartialEq, Eq)]
pub struct Word1 {
    #[bit(31, rw)]
    ownership: Ownership,
    #[bit(30, rw)]
    wrap: bool,
    #[bit(29, rw)]
    retry_limit_exceeded: bool,
    #[bit(27, rw)]
    transmit_frame_corruption_ahb_error: bool,
    #[bit(26, rw)]
    late_collision: bool,
    #[bits(20..=22, rw)]
    checksum_status: TransmitChecksumGenerationStatus,
    #[bit(16, rw)]
    no_crc_generation: bool,
    #[bit(15, rw)]
    last_buffer: bool,
    #[bits(0..=13, rw)]
    tx_len: u14,
}

impl Descriptor {
    #[inline]
    pub const fn new() -> Self {
        Self {
            word0: 0,
            word1: Word1::new_with_raw_value(0),
        }
    }

    #[inline]
    pub fn set_ownership(&mut self, ownership: Ownership) {
        self.word1.set_ownership(ownership);
    }

    /// Set the wrap bit, which should be done for the last descriptor in the descriptor list.
    #[inline]
    pub fn set_wrap_bit(&mut self) {
        self.word1.set_wrap(true);
    }

    /// Set the information for a transfer.
    pub fn set_tx_transfer_info(
        &mut self,
        tx_len: u14,
        last_buffer: bool,
        no_crc_generation: bool,
    ) {
        self.word1.set_tx_len(tx_len);
        self.word1.set_last_buffer(last_buffer);
        self.word1.set_no_crc_generation(no_crc_generation);
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
}
