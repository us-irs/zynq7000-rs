//! RX buffer descriptor module.
use core::{cell::UnsafeCell, mem::MaybeUninit, sync::atomic::AtomicBool};

use crate::{cache::clean_and_invalidate_data_cache_range, eth::AlignedBuffer};

pub use super::shared::Ownership;
use arbitrary_int::{Number, u2, u3, u13, u30};
use vcell::VolatileCell;

static RX_DESCR_TAKEN: AtomicBool = AtomicBool::new(false);

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
pub struct StatusWord {
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
            word_0: VolatileCell::new(Word0::new_with_raw_value(0)),
            status: VolatileCell::new(StatusWord::new_with_raw_value(0)),
        }
    }

    #[inline]
    pub fn ownership(&self) -> Ownership {
        self.word_0.get().ownership()
    }

    #[inline]
    pub fn set_word_0(&mut self, word: Word0) {
        self.word_0.set(word);
    }

    #[inline]
    pub fn set_word_1(&mut self, word: StatusWord) {
        self.status.set(word);
    }

    #[inline]
    pub fn word_0(&mut self) -> Word0 {
        self.word_0.get()
    }

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

unsafe impl<const SLOTS: usize> Sync for DescriptorList<SLOTS> {}

impl<const SLOTS: usize> DescriptorList<SLOTS> {
    #[inline]
    pub const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    /// Initializes the RX descriptors and returns a mutable reference to them.
    pub fn take(&self) -> Option<&'static mut [Descriptor; SLOTS]> {
        if RX_DESCR_TAKEN.swap(true, core::sync::atomic::Ordering::SeqCst) {
            return None; // Already taken, return None
        }
        let descr = unsafe { &mut *self.0.get() };
        descr.write([const { Descriptor::new() }; SLOTS]);
        Some(unsafe { descr.assume_init_mut() })
    }
}

impl<const SLOTS: usize> Default for DescriptorList<SLOTS> {
    fn default() -> Self {
        Self::new()
    }
}

pub enum FrameScanResult {
    NoFrames,
    SingleFrame {
        index: usize,
        size: usize,
        status: StatusWord,
    },
    MultiSlotFrame {
        first_slot_index: usize,
        last_slot_index: usize,
    },
    BrokenFragments {
        first_slot_index: usize,
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
    #[inline]
    pub fn new(descr_list: &'a mut [Descriptor]) -> Self {
        Self {
            list: descr_list,
            current_idx: 0,
        }
    }

    /// Unsafely clone this descriptor list. See safety notes.
    //
    /// # Safety
    //
    /// You must not use both the original and the clone at the same time.
    pub unsafe fn clone_unchecked(&mut self) -> Self {
        Self {
            list: unsafe {
                core::slice::from_raw_parts_mut(self.list.as_mut().as_mut_ptr(), self.list.len())
            },
            current_idx: self.current_idx,
        }
    }
}

impl DescriptorListWrapper<'_> {
    #[allow(clippy::len_without_is_empty)]
    #[inline]
    pub fn len(&self) -> usize {
        self.list.len()
    }

    #[inline]
    pub fn base_ptr(&self) -> *const Descriptor {
        self.list.as_ptr()
    }

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

    pub fn clear_slot(&mut self, index: usize) {
        if index >= self.list.len() {
            return;
        }
        let mut word_0 = self.list[index].word_0.get();
        word_0.set_ownership(Ownership::Hardware);
        self.list[index].set_word_0(word_0);
    }
}
