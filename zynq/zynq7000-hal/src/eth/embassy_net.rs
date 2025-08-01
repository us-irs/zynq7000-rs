//! Embassy-net driver for the Zynq 7000 ethernet peripheral.
use core::sync::atomic::AtomicBool;

pub use crate::eth::smoltcp::DescriptorsAndBuffers;
use crate::eth::smoltcp::{SmoltcpRxToken, SmoltcpTxToken};
pub use crate::eth::{EthernetId, InterruptResult};
use embassy_sync::waitqueue::AtomicWaker;

pub(crate) static TX_WAKER: AtomicWaker = AtomicWaker::new();
pub(crate) static RX_WAKER: AtomicWaker = AtomicWaker::new();

static LINK_WAKER: AtomicWaker = AtomicWaker::new();
static LINK_STATE: AtomicBool = AtomicBool::new(false);

/// This interrupt handler should be called when a Gigabit Ethernet interrupt occurs.
///
/// It also handles embassy-net related waking up of tasks via  wakers.
pub fn on_interrupt(eth_id: EthernetId) -> InterruptResult {
    super::on_interrupt(eth_id, true, true)
}

#[inline]
pub fn link_state() -> embassy_net_driver::LinkState {
    match LINK_STATE.load(core::sync::atomic::Ordering::Relaxed) {
        true => embassy_net_driver::LinkState::Up,
        false => embassy_net_driver::LinkState::Down,
    }
}

pub fn update_link_state(new_state: embassy_net_driver::LinkState) {
    let new_value = match new_state {
        embassy_net_driver::LinkState::Up => true,
        embassy_net_driver::LinkState::Down => false,
    };
    if LINK_STATE.swap(new_value, core::sync::atomic::Ordering::Relaxed) != new_value {
        LINK_WAKER.wake();
    }
}

pub struct Driver(super::smoltcp::CommonSmoltcpDriver);

impl Driver {
    pub fn new(eth: &super::Ethernet, mac_addr: [u8; 6], bufs: DescriptorsAndBuffers) -> Self {
        let tx_burst_len = bufs.tx_burst_len();
        Self(super::smoltcp::CommonSmoltcpDriver::new(
            eth.id(),
            mac_addr,
            bufs,
            tx_burst_len,
        ))
    }
}

impl embassy_net_driver::Driver for Driver {
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
        cx: &mut core::task::Context,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        RX_WAKER.register(cx.waker());
        TX_WAKER.register(cx.waker());
        self.0.receive()
    }

    fn transmit(&mut self, cx: &mut core::task::Context) -> Option<Self::TxToken<'_>> {
        TX_WAKER.register(cx.waker());
        self.0.transmit()
    }

    fn link_state(&mut self, cx: &mut core::task::Context) -> embassy_net_driver::LinkState {
        LINK_WAKER.register(cx.waker());
        link_state()
    }

    fn capabilities(&self) -> embassy_net_driver::Capabilities {
        let mut capabilities = embassy_net_driver::Capabilities::default();
        capabilities.max_transmission_unit = super::MTU;
        capabilities.max_burst_size = Some(self.0.burst_size);
        capabilities.checksum.ipv4 = embassy_net_driver::Checksum::Both;
        capabilities.checksum.udp = embassy_net_driver::Checksum::Both;
        capabilities.checksum.tcp = embassy_net_driver::Checksum::Both;
        capabilities.checksum.icmpv4 = embassy_net_driver::Checksum::None;
        capabilities.checksum.icmpv6 = embassy_net_driver::Checksum::None;
        capabilities
    }

    fn hardware_address(&self) -> embassy_net_driver::HardwareAddress {
        embassy_net_driver::HardwareAddress::Ethernet(self.0.mac_addr)
    }
}
