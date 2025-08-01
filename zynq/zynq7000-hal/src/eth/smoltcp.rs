//! smoltcp driver for the Zynq 7000 ethernet peripheral.
use arbitrary_int::u14;

pub use crate::eth::{EthernetId, InterruptResult};
use crate::{
    cache::{CACHE_LINE_SIZE, clean_and_invalidate_data_cache_range, invalidate_data_cache_range},
    eth::{rx_descr, tx_descr},
};

/// This interrupt handler should be called when a Gigabit Ethernet interrupt occurs.
pub fn on_interrupt(eth_id: EthernetId) -> InterruptResult {
    super::on_interrupt(eth_id, false, false)
}

pub struct SmoltcpRxToken<'a> {
    pub(crate) descr_list: &'a mut super::rx_descr::DescriptorListWrapper<'static>,
    pub(crate) slot_index: usize,
    pub(crate) rx_buf: &'a mut super::AlignedBuffer,
    pub(crate) rx_size: usize,
}

impl embassy_net_driver::RxToken for SmoltcpRxToken<'_> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.consume(f)
    }
}

impl smoltcp::phy::RxToken for SmoltcpRxToken<'_> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        // Convert the mutable slice to an immutable slice for the closure
        self.consume(|buf| f(&buf[..]))
    }
}

impl SmoltcpRxToken<'_> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // The DMA will write the received frame into DDR. The L1 and L2 cache lines for the
        // particular reception address need to be invalidated, to avoid fetching stale data from
        // the cache instead of the DDR.
        let clean_invalidate_len = (self.rx_size + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);
        invalidate_data_cache_range(self.rx_buf.0.as_ptr() as u32, clean_invalidate_len)
            .expect("RX buffer or buffer size not aligned to cache line size");
        log::debug!("eth rx {} bytes", self.rx_size);
        log::trace!("rx data: {:x?}", &self.rx_buf.0[0..self.rx_size]);
        let result = f(&mut self.rx_buf.0[0..self.rx_size]);
        self.descr_list.clear_slot(self.slot_index);
        // Okay, this is weird, but we have to do this. I encountered this bug where ICMP replies
        // were duplicated after the descriptor rings wrapped. My theory is that there is
        // some data in the cache after the embassy reception function which needs to be cleaned.
        clean_and_invalidate_data_cache_range(self.rx_buf.0.as_ptr() as u32, clean_invalidate_len)
            .expect("RX buffer or buffer size not aligned to cache line size");
        result
    }
}

pub struct SmoltcpTxToken<'a> {
    pub(crate) eth_id: super::EthernetId,
    pub(crate) descr_list: &'a mut super::tx_descr::DescriptorListWrapper<'static>,
    pub(crate) tx_bufs: &'a mut [super::AlignedBuffer],
}

impl smoltcp::phy::TxToken for SmoltcpTxToken<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.consume(len, f)
    }
}

impl embassy_net_driver::TxToken for SmoltcpTxToken<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.consume(len, f)
    }
}

impl SmoltcpTxToken<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        assert!(len <= super::MTU, "packet length exceeds MTU");
        // In the transmit call, it was checked that the buffer queue actually is not full.
        let tx_idx = self.descr_list.current_tx_idx();
        let buffer = self.tx_bufs.get_mut(tx_idx).unwrap();
        let addr = buffer.0.as_ptr() as u32;
        let result = f(&mut buffer.0[0..len]);
        let clean_invalidate_len = (len + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);
        // DMA accesses the DDR memory directly, so we need to flush everything that might
        // still be in the L1 or L2 cache to the DDR.
        clean_and_invalidate_data_cache_range(buffer.0.as_ptr() as u32, clean_invalidate_len)
            .expect("TX buffer or buffer size not aligned to cache line size");
        log::debug!("eth tx {len} bytes");
        log::trace!("tx data: {:x?}", &buffer.0[0..len]);
        self.descr_list
            .prepare_transfer_unchecked(Some(addr), u14::new(len as u16), true, false);
        let mut regs = unsafe { self.eth_id.steal_regs() };
        regs.modify_net_ctrl(|mut val| {
            val.set_start_tx(true);
            val
        });
        result
    }
}

pub struct Driver(CommonSmoltcpDriver);

impl smoltcp::phy::Device for Driver {
    type RxToken<'a>
        = SmoltcpRxToken<'a>
    where
        Self: 'a;

    type TxToken<'a>
        = SmoltcpTxToken<'a>
    where
        Self: 'a;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.0.receive()
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        self.0.transmit()
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut capabilities = smoltcp::phy::DeviceCapabilities::default();
        capabilities.medium = smoltcp::phy::Medium::Ethernet;
        capabilities.max_transmission_unit = super::MTU;
        capabilities.max_burst_size = Some(self.0.burst_size);
        capabilities.checksum.ipv4 = smoltcp::phy::Checksum::Both;
        capabilities.checksum.udp = smoltcp::phy::Checksum::Both;
        capabilities.checksum.tcp = smoltcp::phy::Checksum::Both;
        capabilities
    }
}

pub struct DescriptorsAndBuffers {
    pub rx_descr: super::rx_descr::DescriptorListWrapper<'static>,
    pub rx_bufs: &'static mut [super::AlignedBuffer],
    pub tx_descr: super::tx_descr::DescriptorListWrapper<'static>,
    pub tx_bufs: &'static mut [super::AlignedBuffer],
}

impl DescriptorsAndBuffers {
    pub fn new(
        rx_descr: super::rx_descr::DescriptorListWrapper<'static>,
        rx_bufs: &'static mut [super::AlignedBuffer],
        tx_descr: super::tx_descr::DescriptorListWrapper<'static>,
        tx_bufs: &'static mut [super::AlignedBuffer],
    ) -> Option<Self> {
        if rx_descr.len() != rx_bufs.len() || tx_descr.len() != tx_bufs.len() {
            return None;
        }
        Some(Self {
            rx_descr,
            rx_bufs,
            tx_descr,
            tx_bufs,
        })
    }

    #[inline]
    pub fn tx_burst_len(&self) -> usize {
        self.tx_descr.len()
    }
}

pub(crate) struct CommonSmoltcpDriver {
    pub eth_id: super::EthernetId,
    pub mac_addr: [u8; 6],
    pub bufs: DescriptorsAndBuffers,
    pub burst_size: usize,
}

impl CommonSmoltcpDriver {
    pub fn new(
        eth_id: super::EthernetId,
        mac_addr: [u8; 6],
        bufs: DescriptorsAndBuffers,
        burst_size: usize,
    ) -> Self {
        Self {
            burst_size,
            eth_id,
            mac_addr,
            bufs,
        }
    }

    pub fn handle_completed_tx_transfers(&mut self) {
        loop {
            let result = self.bufs.tx_descr.check_and_handle_completed_transfer();
            match result {
                tx_descr::BusyHandlingResult::TxError(tx_error) => {
                    // TODO: An ideal error reporting system would send a message to the user,
                    // but we would need to introduce an abstraction for that..
                    // For now, we log unexpected checksum errors and other errors.
                    if let Some(cksum_error) = tx_error.checksum_generation {
                        // These can occur for ICMP packets.
                        if cksum_error
                            != tx_descr::TransmitChecksumGenerationStatus::PacketNotTcpUdp
                            && cksum_error
                                != tx_descr::TransmitChecksumGenerationStatus::NotVlanOrSnapOrIp
                        {
                            log::warn!("TX checksum generation error: {tx_error:?}");
                        }
                    } else {
                        log::warn!("TX error: {tx_error:?}");
                    }
                }
                tx_descr::BusyHandlingResult::SlotHandled(_) => (),
                tx_descr::BusyHandlingResult::Complete => break,
            }
        }
    }

    pub fn receive(&mut self) -> Option<(SmoltcpRxToken<'_>, SmoltcpTxToken<'_>)> {
        self.handle_completed_tx_transfers();
        if self.bufs.tx_descr.full() {
            // TODO: When introducing an abstraction for notifying the user, send a message
            // for this case.
            log::warn!("TX descriptor queue is full");
            return None;
        }
        match self.bufs.rx_descr.scan_and_handle_next_received_frame(true) {
            // Nothing to do.
            rx_descr::FrameScanResult::NoFrames => None,
            rx_descr::FrameScanResult::SingleFrame {
                index,
                size,
                status,
            } => {
                log::trace!(
                    "eth rx frame, fsc status {:?}, cksum status {:?}",
                    status.fcs_status(),
                    status.type_id_match_info_or_chksum_status()
                );

                let rx_buf = self.bufs.rx_bufs.get_mut(index).unwrap();
                Some((
                    SmoltcpRxToken {
                        descr_list: &mut self.bufs.rx_descr,
                        slot_index: index,
                        rx_buf,
                        rx_size: size,
                    },
                    SmoltcpTxToken {
                        eth_id: self.eth_id,
                        descr_list: &mut self.bufs.tx_descr,
                        tx_bufs: self.bufs.tx_bufs,
                    },
                ))
            }
            rx_descr::FrameScanResult::MultiSlotFrame {
                first_slot_index: _,
                last_slot_index: _,
            } => {
                // We can not really handle multi-slot frames.. this should never happen.
                None
            }
            rx_descr::FrameScanResult::BrokenFragments {
                first_slot_index: _,
                last_slot_index: _,
            } => None,
        }
    }

    pub fn transmit(&mut self) -> Option<SmoltcpTxToken<'_>> {
        // Handle any completed frames first.
        self.handle_completed_tx_transfers();
        if self.bufs.tx_descr.full() {
            // TODO: When introducing an abstraction for notifying the user, send a message
            // for this case.
            log::warn!("TX descriptor queue is full");
            return None;
        }
        Some(SmoltcpTxToken {
            eth_id: self.eth_id,
            descr_list: &mut self.bufs.tx_descr,
            tx_bufs: self.bufs.tx_bufs,
        })
    }
}
