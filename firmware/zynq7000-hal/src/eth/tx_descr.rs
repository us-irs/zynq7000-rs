//! TX buffer descriptor module.
use core::{cell::UnsafeCell, mem::MaybeUninit};

use arbitrary_int::u14;

pub use super::shared::Ownership;
use vcell::VolatileCell;

/// TX buffer descriptor.
///
/// The user should declare an array of this structure inside uncached memory.
///
/// These descriptors are shared between software and hardware and contain information
/// related to frame reception.
#[repr(C, align(8))]
pub struct Descriptor {
    /// The first word of the descriptor which is the byte address of the buffer.
    pub word0: VolatileCell<u32>,
    /// The second word of the descriptor.
    pub word1: VolatileCell<Word1>,
}

/// This is a low level wrapper to simplify declaring a global descriptor list.
///
/// It allows placing the descriptor structure statically in memory which might not
/// be zero-initialized.
#[repr(transparent)]
pub struct DescriptorList<const SLOTS: usize>(pub UnsafeCell<MaybeUninit<[Descriptor; SLOTS]>>);

// SAFETY: The descriptors are only accessed through the reference returned by [DescriptorList::take], which can only be called once.
unsafe impl<const SLOTS: usize> Sync for DescriptorList<SLOTS> {}

impl<const SLOTS: usize> DescriptorList<SLOTS> {
    /// Create an uninitialized list.
    #[inline]
    pub const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    /// Initializes the TX descriptors and returns a mutable reference to them.
    ///
    /// # Safety
    ///
    /// This allows creating aliasing mutable references and circumventing ownership and safety
    /// guarantees of the HAL. You MUST call this function only once per descriptor instance.
    pub unsafe fn take(&self) -> &'static mut [Descriptor; SLOTS] {
        // SAFETY: Upheld by the caller, this is only called once.
        let descr = unsafe { &mut *self.0.get() };
        descr.write([const { Descriptor::new() }; SLOTS]);
        // SAFETY: Initialized by the write above.
        unsafe { descr.assume_init_mut() }
    }
}

impl<const SLOTS: usize> Default for DescriptorList<SLOTS> {
    fn default() -> Self {
        Self::new()
    }
}

/// Checksum generation status of a transmitted frame.
#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum TransmitChecksumGenerationStatus {
    /// No error.
    NoError = 0b000,
    /// VLAN header error.
    VlanError = 0b001,
    /// SNAP header error.
    SnapError = 0b010,
    /// IP header error.
    IpError = 0b011,
    /// Not a VLAN, SNAP or IP frame.
    NotVlanOrSnapOrIp = 0b100,
    /// Fragmented packets are not supported.
    NonSupportedPacketFragmentation = 0b101,
    /// Not a TCP or UDP packet.
    PacketNotTcpUdp = 0b110,
    /// Premature end of frame.
    PrematureEndOfFrame = 0b111,
}

/// Second descriptor word with the control and status bits.
#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug, PartialEq, Eq)]
pub struct Word1 {
    /// The ownership bit must be set to [Ownership::Hardware] if a frame should be transmitted.
    ///
    /// The controller will set this to [Ownership::Software] once the frame has been transmitted.
    #[bit(31, rw)]
    ownership: Ownership,
    /// Last descriptor of the list.
    #[bit(30, rw)]
    wrap: bool,
    /// Retry limit exceeded.
    #[bit(29, rw)]
    retry_limit_exceeded: bool,
    /// Frame corrupted because of an AHB error.
    #[bit(27, rw)]
    transmit_frame_corruption_ahb_error: bool,
    /// Late collision.
    #[bit(26, rw)]
    late_collision: bool,
    /// Checksum generation status.
    #[bits(20..=22, rw)]
    checksum_status: TransmitChecksumGenerationStatus,
    /// Do not append a CRC.
    #[bit(16, rw)]
    no_crc_generation: bool,
    /// Last buffer of the frame.
    #[bit(15, rw)]
    last_buffer: bool,
    /// Buffer length in bytes.
    #[bits(0..=13, rw)]
    tx_len: u14,
}

impl Descriptor {
    /// Create a zeroed descriptor.
    #[inline]
    pub const fn new() -> Self {
        Self {
            word0: VolatileCell::new(0),
            word1: VolatileCell::new(Word1::new_with_raw_value(0)),
        }
    }

    /// Ownership of the descriptor.
    #[inline]
    pub fn ownership(&self) -> Ownership {
        self.word1.get().ownership()
    }

    /// Set the buffer address.
    #[inline]
    pub fn set_addr(&self, addr: u32) {
        self.word0.set(addr)
    }

    /// Read the second descriptor word.
    #[inline]
    pub fn read_word_1(&self) -> Word1 {
        self.word1.get()
    }

    /// Set the second descriptor word.
    #[inline]
    pub fn set_word_1(&self, word1: Word1) {
        self.word1.set(word1);
    }

    /// Set the information for a transfer.
    pub fn setup_tx_transfer_unchecked(
        &mut self,
        addr: Option<u32>,
        tx_len: u14,
        last_buffer: bool,
        no_crc_generation: bool,
    ) {
        if let Some(addr) = addr {
            self.set_addr(addr);
        }
        // Perform the read-modify-write sequence manually to ensure a minimum of reads/writes
        // for the uncached memory.
        let mut word1 = self.read_word_1();
        word1.set_tx_len(tx_len);
        word1.set_last_buffer(last_buffer);
        word1.set_no_crc_generation(no_crc_generation);
        word1.set_ownership(Ownership::Hardware);
        self.set_word_1(word1);
    }
}

impl Default for Descriptor {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// TX error bits of a descriptor.
#[derive(Debug, PartialEq, Eq, thiserror::Error)]
#[error("tx error: {self:?}")]
pub struct TxError {
    /// Retry limit exceeded.
    pub retry_limit_exceeded: bool,
    /// Late collision.
    pub late_collisions: bool,
    /// Frame corrupted because of an AHB error.
    pub ahb_error: bool,
    /// Checksum generation error, if any.
    pub checksum_generation: Option<TransmitChecksumGenerationStatus>,
}

impl TxError {
    /// Read the error bits from the second descriptor word.
    pub fn from_word1(word1: &Word1) -> Self {
        Self {
            retry_limit_exceeded: word1.retry_limit_exceeded(),
            late_collisions: word1.late_collision(),
            ahb_error: word1.transmit_frame_corruption_ahb_error(),
            checksum_generation: if word1.checksum_status()
                == TransmitChecksumGenerationStatus::NoError
            {
                None
            } else {
                Some(word1.checksum_status())
            },
        }
    }
}

/// Result of [DescriptorListWrapper::check_and_handle_completed_transfer].
#[derive(Debug, PartialEq, Eq)]
pub enum BusyHandlingResult {
    /// Handled a descriptor slot where a TX error has occured.
    TxError(TxError),
    /// Handled one busy descriptor slot. More calls to this function are required to handle
    /// all busy slots.
    SlotHandled(super::tx_descr::Word1),
    /// The busy index has caught up to the transmission index (all frames have been sent), or
    /// the busy index is at an active transmission index.
    Complete,
}

/// Thin wrapper around a TX descriptor list.
pub struct DescriptorListWrapper<'a> {
    list: &'a mut [Descriptor],
    /// The head index is used to handle the transmission of new frames.
    tx_idx: usize,
    /// The tail index is used to track the progress of active transmissions.
    busy_idx: usize,
}

impl core::fmt::Debug for DescriptorListWrapper<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DescriptorList")
            .field("tx_idx", &self.tx_idx)
            .field("busy_idx", &self.busy_idx)
            .field("list_len", &self.list.len())
            .finish()
    }
}

impl<'a> DescriptorListWrapper<'a> {
    /// Create a wrapper for the given descriptors.
    #[inline]
    pub fn new(descr_list: &'a mut [Descriptor]) -> Self {
        Self {
            list: descr_list,
            tx_idx: 0,
            busy_idx: 0,
        }
    }
}

impl DescriptorListWrapper<'_> {
    /// Number of descriptors.
    #[allow(clippy::len_without_is_empty)]
    #[inline]
    pub fn len(&self) -> usize {
        self.list.len()
    }

    /// Pointer to the first descriptor.
    #[inline]
    pub fn base_ptr(&self) -> *const Descriptor {
        self.list.as_ptr()
    }

    /// Address of the first descriptor.
    #[inline]
    pub fn base_addr(&self) -> u32 {
        self.base_ptr() as u32
    }

    /// Give all descriptors to the software and reset the indexes.
    pub fn init_or_reset(&mut self) {
        self.tx_idx = 0;
        self.busy_idx = 0;
        let mut reset_val = Word1::builder()
            .with_ownership(Ownership::Software)
            .with_wrap(false)
            .with_retry_limit_exceeded(false)
            .with_transmit_frame_corruption_ahb_error(false)
            .with_late_collision(false)
            .with_checksum_status(TransmitChecksumGenerationStatus::NoError)
            .with_no_crc_generation(false)
            .with_last_buffer(false)
            .with_tx_len(u14::new(0))
            .build();
        let list_len = self.list.len();
        for desc in self.list.iter_mut().take(list_len - 1) {
            desc.set_word_1(reset_val);
        }
        reset_val.set_wrap(true);
        self.list.last_mut().unwrap().set_word_1(reset_val);
    }

    /// Increment the TX index.
    #[inline]
    pub fn increment_tx_idx(&mut self) {
        self.tx_idx = (self.tx_idx + 1) % self.list.len();
    }

    /// Increment the busy index without checking whether it has become
    /// larger than the transmission index.
    #[inline]
    pub fn increment_busy_idx_unchecked(&mut self) {
        self.busy_idx = (self.busy_idx + 1) % self.list.len();
    }

    /// The next descriptor is still in use by the hardware.
    #[inline]
    pub fn full(&self) -> bool {
        self.list[self.tx_idx].ownership() == Ownership::Hardware
    }

    /// Check whether a transfer has completed and handles the descriptor accordingly.
    ///
    /// This should be called continuosly when a TX error or a TX completion interrupt has
    /// occured until it returns [BusyHandlingResult::Complete].
    pub fn check_and_handle_completed_transfer(&mut self) -> BusyHandlingResult {
        let word1 = self.list[self.busy_idx].word1.get();
        if word1.ownership() == Ownership::Hardware || self.busy_idx == self.tx_idx {
            return BusyHandlingResult::Complete;
        }
        if word1.transmit_frame_corruption_ahb_error()
            || word1.retry_limit_exceeded()
            || word1.checksum_status() != TransmitChecksumGenerationStatus::NoError
            || word1.late_collision()
        {
            self.increment_busy_idx_unchecked();
            return BusyHandlingResult::TxError(TxError::from_word1(&word1));
        }
        self.list[self.busy_idx].set_word_1(
            Word1::builder()
                .with_ownership(Ownership::Software)
                .with_wrap(word1.wrap())
                .with_retry_limit_exceeded(false)
                .with_transmit_frame_corruption_ahb_error(false)
                .with_late_collision(false)
                .with_checksum_status(TransmitChecksumGenerationStatus::NoError)
                .with_no_crc_generation(false)
                .with_last_buffer(false)
                .with_tx_len(u14::new(0))
                .build(),
        );
        self.increment_busy_idx_unchecked();
        BusyHandlingResult::SlotHandled(word1)
    }

    /// Current TX index.
    #[inline]
    pub fn current_tx_idx(&self) -> usize {
        self.tx_idx
    }

    /// Prepare a transfer without checking whether that particular descriptor slot is used by
    /// the hardware.
    pub fn prepare_transfer_unchecked(
        &mut self,
        addr: Option<u32>,
        tx_len: u14,
        last_buffer: bool,
        no_crc_generation: bool,
    ) {
        self.list[self.tx_idx].setup_tx_transfer_unchecked(
            addr,
            tx_len,
            last_buffer,
            no_crc_generation,
        );
        self.increment_tx_idx();
    }
}
