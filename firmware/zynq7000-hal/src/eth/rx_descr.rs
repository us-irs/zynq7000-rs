//! RX buffer descriptor module.
use core::{cell::UnsafeCell, mem::MaybeUninit};

use crate::{cache::clean_and_invalidate_data_cache_range, eth::AlignedBuffer};

pub use super::shared::Ownership;
use arbitrary_int::{prelude::*, u2, u3, u13, u30};
use vcell::VolatileCell;

/// RX buffer descriptor.
///
/// The user should declare an array of this structure inside uncached memory.
///
/// These descriptors are shared between software and hardware and contain information
/// related to frame reception.
#[repr(C, align(8))]
pub struct Descriptor {
    /// The first word of the descriptor.
    pub word_0: VolatileCell<Word0>,
    /// The second word of the descriptor.
    pub status: VolatileCell<StatusWord>,
}

/// First descriptor word.
#[bitbybit::bitfield(u32)]
#[derive(Debug, PartialEq, Eq)]
pub struct Word0 {
    /// The full reception address with the last two bits cleared.
    #[bits(2..=31, rw)]
    addr_upper_30_bits: u30,
    /// Last descriptor of the list.
    #[bit(1, rw)]
    wrap: bool,
    /// Ownership of the descriptor.
    #[bit(0, rw)]
    ownership: Ownership,
}

/// This control word contains the status bits.
///
/// If the end of frame bit (15) is not set, the only valid status information is the start of
/// frame bit.
#[bitbybit::bitfield(u32)]
#[derive(Debug, PartialEq, Eq)]
pub struct StatusWord {
    /// Broadcast address detected.
    #[bit(31, r)]
    broadcast_detect: bool,
    /// Multicast hash match.
    #[bit(30, r)]
    multicast_hash: bool,
    /// Unicast hash match.
    #[bit(29, r)]
    unicast_hash: bool,
    /// Specific address register match.
    #[bit(27, r)]
    specific_addr_match: bool,
    /// Specifies which of the 4 specific address registers was matched.
    #[bits(25..=26, r)]
    specific_addr_match_info: u2,
    /// Type ID register match. With RX checksum offload: SNAP encoded frame.
    #[bit(24, r)]
    type_id_match_or_snap_info: bool,
    /// Which type ID register matched. With RX checksum offload: checksum status.
    #[bits(22..=23, r)]
    type_id_match_info_or_chksum_status: u2,
    /// VLAN tag detected.
    #[bit(21, r)]
    vlan_tag_detected: bool,
    /// Priority tag detected.
    #[bit(20, r)]
    priority_tag_detected: bool,
    /// VLAN priority.
    #[bits(17..=19, r)]
    vlan_prio: u3,
    /// Canonical format indicator (CFI) bit.
    #[bit(16, r)]
    cfi_bit: bool,
    /// If this bit is not set, the only other valid status bit is the start of frame bit.
    #[bit(15, r)]
    end_of_frame: bool,
    /// Start of frame.
    #[bit(14, r)]
    start_of_frame: bool,
    /// Relevant when FCS errors are not ignored.
    /// 0: Frame has good FCS, 1: Frame has bad FCS, but was copied to memory as the ignore FCS
    /// functionality was enabled.
    #[bit(13, r)]
    fcs_status: bool,
    /// Frame length in bytes.
    #[bits(0..=12, r)]
    rx_len: u13,
}

impl Descriptor {
    /// Create a zeroed descriptor.
    #[inline]
    pub const fn new() -> Self {
        Self {
            word_0: VolatileCell::new(Word0::new_with_raw_value(0)),
            status: VolatileCell::new(StatusWord::new_with_raw_value(0)),
        }
    }

    /// Ownership of the descriptor.
    #[inline]
    pub fn ownership(&self) -> Ownership {
        self.word_0.get().ownership()
    }

    /// Set the first descriptor word.
    #[inline]
    pub fn set_word_0(&mut self, word: Word0) {
        self.word_0.set(word);
    }

    /// Set the status word.
    #[inline]
    pub fn set_word_1(&mut self, word: StatusWord) {
        self.status.set(word);
    }

    /// First descriptor word.
    #[inline]
    pub fn word_0(&mut self) -> Word0 {
        self.word_0.get()
    }

    /// Status word.
    #[inline]
    pub fn status_word(&mut self) -> StatusWord {
        self.status.get()
    }
}

impl Default for Descriptor {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
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

    /// Initializes the RX descriptors and returns a mutable reference to them.
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

/// Result of a frame scan.
pub enum FrameScanResult {
    /// No received frames.
    NoFrames,
    /// A frame in a single slot.
    SingleFrame {
        /// Slot index.
        index: usize,
        /// Frame size in bytes.
        size: usize,
        /// Status word of the slot.
        status: StatusWord,
    },
    /// A frame spanning multiple slots.
    MultiSlotFrame {
        /// Index of the first slot.
        first_slot_index: usize,
        /// Index of the last slot.
        last_slot_index: usize,
    },
    /// Broken frame fragments, which were discarded.
    BrokenFragments {
        /// Index of the first slot.
        first_slot_index: usize,
        /// Index of the last slot.
        last_slot_index: usize,
    },
}

/// This is a thin wrapper around a descriptor list.
///
/// It provides a basic higher-level API to perform tasks like descriptor initialization
/// and handling of received frames.
pub struct DescriptorListWrapper<'a> {
    list: &'a mut [Descriptor],
    current_idx: usize,
}

impl<'a> DescriptorListWrapper<'a> {
    /// Create a wrapper for the given descriptors.
    #[inline]
    pub fn new(descr_list: &'a mut [Descriptor]) -> Self {
        Self {
            list: descr_list,
            current_idx: 0,
        }
    }

    /// Unsafely clone this descriptor list. See safety notes.
    ///
    /// # Safety
    ///
    /// You must not use both the original and the clone at the same time.
    pub unsafe fn clone_unchecked(&mut self) -> Self {
        Self {
            // SAFETY: Upheld by the caller.
            list: unsafe {
                core::slice::from_raw_parts_mut(self.list.as_mut().as_mut_ptr(), self.list.len())
            },
            current_idx: self.current_idx,
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

    /// Resets the descriptor list. This retains the previous configured reception
    /// addresses, but sets the ownership for all descriptors to the hardware again.
    pub fn reset(&mut self) {
        self.current_idx = 0;
        let list_len = self.list.len();
        let mut word_0;
        for desc in self.list.iter_mut().take(list_len - 1) {
            word_0 = desc.word_0();
            word_0.set_ownership(Ownership::Hardware);
            word_0.set_wrap(false);
            desc.set_word_0(word_0);
        }
        let last = self.list.last_mut().unwrap();
        word_0 = last.word_0();
        word_0.set_ownership(Ownership::Hardware);
        word_0.set_wrap(true);
        last.set_word_0(word_0);
    }

    /// Initialize the descriptor list with a provided RX buffer.
    ///
    /// This function performs important initialization routines for the descriptors
    /// and also sets the addresses of the provided buffers. The number of buffers and the length
    /// of the descriptor list need to be the same.
    pub fn init_with_aligned_bufs(&mut self, aligned_bufs: &[AlignedBuffer]) {
        self.current_idx = 0;
        let list_len = self.list.len();
        for (desc, buf) in self.list.iter_mut().take(list_len - 1).zip(aligned_bufs) {
            desc.set_word_0(
                Word0::builder()
                    .with_addr_upper_30_bits(u30::new(buf.0.as_ptr() as u32 >> 2))
                    .with_wrap(false)
                    .with_ownership(Ownership::Hardware)
                    .build(),
            );

            clean_and_invalidate_data_cache_range(
                buf.0.as_ptr() as u32,
                core::mem::size_of::<AlignedBuffer>(),
            )
            .expect("TX buffer or buffer size not aligned to cache line size");
        }
        self.list.last_mut().unwrap().set_word_0(
            Word0::builder()
                .with_addr_upper_30_bits(u30::new(
                    aligned_bufs.last().unwrap().0.as_ptr() as u32 >> 2,
                ))
                .with_wrap(true)
                .with_ownership(Ownership::Hardware)
                .build(),
        );
    }

    /// This function tries to scan for received frames and handles the frame if it finds one.
    pub fn scan_and_handle_next_received_frame(
        &mut self,
        scan_full_descr_list: bool,
    ) -> FrameScanResult {
        let mut handled_slots = 0;
        let mut current_idx = self.current_idx;
        let mut start_found = false;
        let mut start_idx = 0;
        while handled_slots < self.list.len() {
            let word_0 = self.list[current_idx].word_0.get();
            if word_0.ownership() == Ownership::Hardware {
                if scan_full_descr_list {
                    current_idx = (current_idx + 1) % self.list.len();
                    handled_slots += 1;
                    continue;
                }
                // The descriptor is not owned by the hardware, so it is not ready for processing.
                return FrameScanResult::NoFrames;
            }
            let status = self.list[current_idx].status_word();
            match (status.start_of_frame(), status.end_of_frame()) {
                (true, true) => {
                    self.current_idx = (current_idx + 1) % self.list.len();
                    return FrameScanResult::SingleFrame {
                        index: current_idx,
                        size: status.rx_len().as_usize(),
                        status,
                    };
                }
                (true, false) => {
                    // Consecutive start frame.. Which means something went wrong, and we need
                    // to discard all the slots until the second start frame slot.
                    if start_found {
                        self.clear_slots(start_idx, current_idx);
                        self.current_idx = (current_idx + 1) % self.list.len();
                        return FrameScanResult::BrokenFragments {
                            first_slot_index: start_idx,
                            last_slot_index: current_idx,
                        };
                    } else {
                        start_found = true;
                        start_idx = current_idx;
                    }
                }
                (false, true) => {
                    // Detected frame spanning multiple buffers.. which should really not happen
                    // if we only use frames with a certain MTU, but we will handle it.
                    if start_found {
                        self.current_idx = (current_idx + 1) % self.list.len();
                        return FrameScanResult::MultiSlotFrame {
                            first_slot_index: start_idx,
                            last_slot_index: current_idx,
                        };
                    }
                }
                (false, false) => {
                    // If a slot is neither the start nor the end of frame.
                }
            }
            current_idx = (current_idx + 1) % self.list.len();
            handled_slots += 1;
        }
        FrameScanResult::NoFrames
    }

    /// Clear a slot range by setting the ownership bit back to [Ownership::Hardware].
    pub fn clear_slots(&mut self, start: usize, end: usize) {
        if start >= self.list.len() || end >= self.list.len() {
            return;
        }
        let mut current_idx = start;
        while current_idx != end {
            let mut word_0 = self.list[current_idx].word_0.get();
            word_0.set_ownership(Ownership::Hardware);
            self.list[current_idx].set_word_0(word_0);
            current_idx = (current_idx + 1) % self.list.len();
        }
    }

    /// Give the slot back to the hardware after its frame was processed.
    pub fn clear_slot(&mut self, index: usize) {
        if index >= self.list.len() {
            return;
        }
        let mut word_0 = self.list[index].word_0.get();
        word_0.set_ownership(Ownership::Hardware);
        self.list[index].set_word_0(word_0);
    }
}
