//! # Ethernet module
//!
//! ## Examples
//!
//! - [Zedboard Ethernet](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq/examples/zedboard/src/bin/ethernet.rs)
use arbitrary_int::{u2, u3};
pub use zynq7000::eth::MdcClockDivisor;
use zynq7000::eth::{
    BurstLength, DmaRxBufSize, GEM_0_BASE_ADDR, GEM_1_BASE_ADDR, InterruptControl, InterruptStatus,
    MmioEthernet, RxStatus, TxStatus,
};

pub use ll::{ClockConfig, ClockDivSet, Duplex, EthernetLowLevel, Speed};

pub mod embassy_net;
pub mod ll;
pub mod mdio;
pub mod rx_descr;
pub mod smoltcp;
pub mod tx_descr;

pub const MTU: usize = 1536;
pub const MAX_MDC_SPEED: Hertz = Hertz::from_raw(2_500_000);

#[repr(C, align(32))]
#[derive(Debug, Clone, Copy)]
pub struct AlignedBuffer(pub [u8; MTU]);

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27,
};
use crate::{
    clocks::ArmClocks,
    eth::ll::ClockDivisors,
    gpio::{
        IoPeriphPin,
        mio::{
            Mio28, Mio29, Mio30, Mio31, Mio32, Mio33, Mio34, Mio35, Mio36, Mio37, Mio38, Mio39,
            Mio52, Mio53, MioPin, MuxConfig, Pin,
        },
    },
    time::Hertz,
};

pub const MUX_CONF_PHY: MuxConfig = MuxConfig::new_with_l0();
pub const MUX_CONF_MDIO: MuxConfig = MuxConfig::new_with_l3(u3::new(0b100));

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EthernetId {
    Eth0 = 0,
    Eth1 = 1,
}

impl EthernetId {
    /// Steal the ethernet register block for the given ethernet ID.
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees of the HAL.
    pub const unsafe fn steal_regs(&self) -> zynq7000::eth::MmioEthernet<'static> {
        unsafe {
            match self {
                EthernetId::Eth0 => zynq7000::eth::Ethernet::new_mmio_fixed_0(),
                EthernetId::Eth1 => zynq7000::eth::Ethernet::new_mmio_fixed_1(),
            }
        }
    }

    pub fn clk_config_regs(
        &self,
        slcr: &mut zynq7000::slcr::MmioSlcr<'static>,
    ) -> (
        *mut zynq7000::slcr::clocks::GigEthClockControl,
        *mut zynq7000::slcr::clocks::GigEthRclkControl,
    ) {
        match self {
            EthernetId::Eth0 => (
                slcr.clk_ctrl().pointer_to_gem_0_clk_ctrl(),
                slcr.clk_ctrl().pointer_to_gem_0_rclk_ctrl(),
            ),
            EthernetId::Eth1 => (
                slcr.clk_ctrl().pointer_to_gem_1_clk_ctrl(),
                slcr.clk_ctrl().pointer_to_gem_1_rclk_ctrl(),
            ),
        }
    }
}

pub trait PsEthernet {
    fn reg_block(&self) -> MmioEthernet<'static>;
    fn id(&self) -> Option<EthernetId>;
}

impl PsEthernet for MmioEthernet<'static> {
    #[inline]
    fn reg_block(&self) -> MmioEthernet<'static> {
        unsafe { self.clone() }
    }

    #[inline]
    fn id(&self) -> Option<EthernetId> {
        let base_addr = unsafe { self.ptr() } as usize;
        if base_addr == GEM_0_BASE_ADDR {
            return Some(EthernetId::Eth0);
        } else if base_addr == GEM_1_BASE_ADDR {
            return Some(EthernetId::Eth1);
        }
        None
    }
}

pub trait TxClockPin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait TxControlPin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait TxData0Pin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait TxData1Pin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait TxData2Pin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait TxData3Pin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait RxClockPin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait RxControlPin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait RxData0Pin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait RxData1Pin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait RxData2Pin: MioPin {
    const ETH_ID: EthernetId;
}
pub trait RxData3Pin: MioPin {
    const ETH_ID: EthernetId;
}

pub trait MdClockPin: MioPin {}
pub trait MdIoPin: MioPin {}

impl MdClockPin for Pin<Mio52> {}
impl MdIoPin for Pin<Mio53> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxClockPin for Pin<Mio16> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxControlPin for Pin<Mio21> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData0Pin for Pin<Mio17> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData1Pin for Pin<Mio18> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData2Pin for Pin<Mio19> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData3Pin for Pin<Mio20> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxClockPin for Pin<Mio22> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxControlPin for Pin<Mio27> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData0Pin for Pin<Mio23> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData1Pin for Pin<Mio24> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData2Pin for Pin<Mio25> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData3Pin for Pin<Mio26> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}

impl TxClockPin for Pin<Mio28> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxControlPin for Pin<Mio33> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData0Pin for Pin<Mio29> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData1Pin for Pin<Mio30> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData2Pin for Pin<Mio31> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData3Pin for Pin<Mio32> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxClockPin for Pin<Mio34> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxControlPin for Pin<Mio39> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData0Pin for Pin<Mio35> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData1Pin for Pin<Mio36> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData2Pin for Pin<Mio37> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData3Pin for Pin<Mio38> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}

/// Calculate the CPU 1x clock divisor required to achieve a clock speed which is below
/// 2.5 MHz, as specified by the 802.3 standard.
pub fn calculate_mdc_clk_div(arm_clks: &ArmClocks) -> Option<MdcClockDivisor> {
    let div = arm_clks.cpu_1x_clk().raw().div_ceil(MAX_MDC_SPEED.raw());
    match div {
        0..8 => Some(MdcClockDivisor::Div8),
        8..16 => Some(MdcClockDivisor::Div16),
        16..32 => Some(MdcClockDivisor::Div32),
        32..48 => Some(MdcClockDivisor::Div48),
        48..64 => Some(MdcClockDivisor::Div64),
        64..96 => Some(MdcClockDivisor::Div96),
        96..128 => Some(MdcClockDivisor::Div128),
        128..224 => Some(MdcClockDivisor::Div224),
        // MDC clock divisor is too high for the maximum speed.
        // This is not a valid configuration.
        _ => None,
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EthernetConfig {
    pub clk_config_1000_mbps: ClockConfig,
    pub mdc_clk_div: MdcClockDivisor,
    pub mac_address: [u8; 6],
}

impl EthernetConfig {
    /// Creates a new Ethernet configuration.
    pub fn new(
        clk_config_1000_mbps: ClockConfig,
        mdc_clk_div: MdcClockDivisor,
        mac_address: [u8; 6],
    ) -> Self {
        Self {
            clk_config_1000_mbps,
            mdc_clk_div,
            mac_address,
        }
    }
}

/// Higher-level ethernet abstraction.
pub struct Ethernet {
    ll: ll::EthernetLowLevel,
    mdio: mdio::Mdio,
    current_speed: Speed,
    current_duplex: Duplex,
    current_clk_divs: ClockDivisors,
}

const IRQ_CONTROL: InterruptControl = InterruptControl::builder()
    .with_tsu_sec_incr(false)
    .with_partner_pg_rx(false)
    .with_auto_negotiation_complete(false)
    .with_external_interrupt(false)
    .with_pause_transmitted(false)
    .with_pause_time_zero(false)
    .with_pause_with_non_zero_quantum(false)
    .with_hresp_not_ok(true)
    .with_rx_overrun(true)
    .with_link_changed(false)
    .with_frame_sent(true)
    .with_tx_frame_corruption_ahb_error(true)
    .with_tx_retry_limit_reached_or_late_collision(true)
    .with_tx_descr_read_when_used(true)
    .with_rx_descr_read_when_used(true)
    .with_frame_received(true)
    .with_mgmt_frame_sent(false)
    .build();

const IRQ_CLEAR_ALL: InterruptStatus = InterruptStatus::builder()
    .with_tsu_sec_incr(true)
    .with_partner_pg_rx(true)
    .with_auto_negotiation_complete(true)
    .with_external_interrupt(true)
    .with_pause_transmitted(true)
    .with_pause_time_zero(true)
    .with_pause_with_non_zero_quantum(true)
    .with_hresp_not_ok(true)
    .with_rx_overrun(true)
    .with_link_changed(true)
    .with_frame_sent(true)
    .with_tx_retry_limit_reached_or_late_collision(true)
    .with_tx_descr_read_when_used(true)
    .with_rx_descr_read_when_used(true)
    .with_frame_received(true)
    .with_mgmt_frame_sent(true)
    .build();

impl Ethernet {
    /// Creates a new Ethernet instance with the given configuration while also
    /// configuring all the necessary MIO pins.
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_mio<
        TxClock: TxClockPin,
        TxControl: TxControlPin,
        TxData0: TxData0Pin,
        TxData1: TxData1Pin,
        TxData2: TxData2Pin,
        TxData3: TxData3Pin,
        RxClock: RxClockPin,
        RxControl: RxControlPin,
        RxData0: RxData0Pin,
        RxData1: RxData1Pin,
        RxData2: RxData2Pin,
        RxData3: RxData3Pin,
        MdClock: MdClockPin,
        MdIo: MdIoPin,
    >(
        mut ll: ll::EthernetLowLevel,
        config: EthernetConfig,
        tx_clk: TxClock,
        tx_ctrl: TxControl,
        tx_data: (TxData0, TxData1, TxData2, TxData3),
        rx_clk: RxClock,
        rx_ctrl: RxControl,
        rx_data: (RxData0, RxData1, RxData2, RxData3),
        md_pins: Option<(MdClock, MdIo)>,
    ) -> Self {
        Self::common_init(&mut ll, config.mac_address);
        let tx_mio_config = zynq7000::slcr::mio::Config::builder()
            .with_disable_hstl_rcvr(true)
            .with_pullup(true)
            .with_io_type(zynq7000::slcr::mio::IoType::Hstl)
            .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
            .with_l3_sel(MUX_CONF_PHY.l3_sel())
            .with_l2_sel(MUX_CONF_PHY.l2_sel())
            .with_l1_sel(MUX_CONF_PHY.l1_sel())
            .with_l0_sel(MUX_CONF_PHY.l0_sel())
            .with_tri_enable(false)
            .build();
        let rx_mio_config = zynq7000::slcr::mio::Config::builder()
            .with_disable_hstl_rcvr(false)
            .with_pullup(true)
            .with_io_type(zynq7000::slcr::mio::IoType::Hstl)
            .with_speed(zynq7000::slcr::mio::Speed::FastCmosEdge)
            .with_l3_sel(MUX_CONF_PHY.l3_sel())
            .with_l2_sel(MUX_CONF_PHY.l2_sel())
            .with_l1_sel(MUX_CONF_PHY.l1_sel())
            .with_l0_sel(MUX_CONF_PHY.l0_sel())
            // TODO: What is correct now?
            // Disable output driver.
            .with_tri_enable(true)
            //.with_tri_enable(false)
            .build();
        unsafe {
            crate::slcr::Slcr::with(|slcr_mut| {
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    tx_clk,
                    slcr_mut,
                    tx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    tx_ctrl,
                    slcr_mut,
                    tx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    tx_data.0,
                    slcr_mut,
                    tx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    tx_data.1,
                    slcr_mut,
                    tx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    tx_data.2,
                    slcr_mut,
                    tx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    tx_data.3,
                    slcr_mut,
                    tx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    rx_clk,
                    slcr_mut,
                    rx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    rx_ctrl,
                    slcr_mut,
                    rx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    rx_data.0,
                    slcr_mut,
                    rx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    rx_data.1,
                    slcr_mut,
                    rx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    rx_data.2,
                    slcr_mut,
                    rx_mio_config,
                );
                IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                    rx_data.3,
                    slcr_mut,
                    rx_mio_config,
                );
                if let Some((md_clk, md_io)) = md_pins {
                    let md_mio_config = zynq7000::slcr::mio::Config::builder()
                        .with_disable_hstl_rcvr(false)
                        .with_pullup(true)
                        .with_io_type(zynq7000::slcr::mio::IoType::LvCmos18)
                        .with_speed(zynq7000::slcr::mio::Speed::SlowCmosEdge)
                        .with_l3_sel(MUX_CONF_MDIO.l3_sel())
                        .with_l2_sel(MUX_CONF_MDIO.l2_sel())
                        .with_l1_sel(MUX_CONF_MDIO.l1_sel())
                        .with_l0_sel(MUX_CONF_MDIO.l0_sel())
                        .with_tri_enable(false)
                        .build();
                    IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                        md_clk,
                        slcr_mut,
                        md_mio_config,
                    );
                    IoPeriphPin::new_with_full_config_and_unlocked_slcr(
                        md_io,
                        slcr_mut,
                        md_mio_config,
                    );
                }
                // Enable VREF internal generator, which is required for HSTL pin mode.
                slcr_mut.gpiob().modify_ctrl(|mut ctrl| {
                    ctrl.set_vref_en(true);
                    ctrl
                });
            });
        }
        ll.configure_clock(config.clk_config_1000_mbps, true);
        let mut mdio = mdio::Mdio::new(&ll, true);
        mdio.configure_clock_div(config.mdc_clk_div);
        ll.regs.modify_net_ctrl(|mut val| {
            val.set_management_port_enable(true);
            val
        });
        let mut eth = Ethernet {
            ll,
            mdio,
            current_speed: Speed::Mbps1000,
            current_duplex: Duplex::Full,
            current_clk_divs: config.clk_config_1000_mbps.divs,
        };
        eth.set_rx_buf_descriptor_base_address(0);
        eth.set_tx_buf_descriptor_base_address(0);
        eth
    }

    pub fn new(mut ll: EthernetLowLevel, config: EthernetConfig) -> Self {
        Self::common_init(&mut ll, config.mac_address);
        ll.configure_clock(config.clk_config_1000_mbps, true);
        let mut mdio = mdio::Mdio::new(&ll, true);
        mdio.configure_clock_div(config.mdc_clk_div);
        Ethernet {
            ll,
            mdio,
            current_speed: Speed::Mbps1000,
            current_duplex: Duplex::Full,
            current_clk_divs: config.clk_config_1000_mbps.divs,
        }
    }

    /// Starts the peripheral by enabling relevant interrupts as well as the TX and RX blocks.
    ///
    /// The user should set valid TX and RX buffer queue addresses before calling this function.
    pub fn start(&mut self) {
        self.clear_interrupts();
        self.enable_interrupts();
        self.ll.regs.modify_net_ctrl(|mut val| {
            val.set_tx_enable(true);
            val.set_rx_enable(true);
            val
        });
    }

    /// Current speed settings.
    #[inline]
    pub fn speed(&self) -> Speed {
        self.current_speed
    }

    /// Current duplex settings.
    #[inline]
    pub fn duplex(&self) -> Duplex {
        self.current_duplex
    }

    pub fn reset(&mut self) {
        self.ll.reset(3);
        // Do not set RX and TX queue base address.
        self.ll.initialize(false);
    }

    fn common_init(ll: &mut EthernetLowLevel, mac_address: [u8; 6]) {
        ll.enable_peripheral_clock();
        ll.reset(3);
        ll.initialize(true);
        // Set speed and duplex to sensible values, but these probably need to be set again after
        // auto-negotiation has completed.
        ll.set_speed_and_duplex(Speed::Mbps1000, Duplex::Full);
        let macaddr_msbs = (u32::from(mac_address[5]) << 8) | u32::from(mac_address[4]);
        let macaddr_lsbs = (u32::from(mac_address[3]) << 24)
            | (u32::from(mac_address[2]) << 16)
            | (u32::from(mac_address[1]) << 8)
            | u32::from(mac_address[0]);
        // Writing to the lower address portion disables the address match, writing to the higher
        // portion enables it again. Address matching is disabled on reset, so we do not need
        // to disable the other addresses here.
        ll.regs.write_addr1_low(macaddr_lsbs);
        ll.regs.write_addr1_high(macaddr_msbs);
        ll.regs.modify_net_cfg(|mut val| {
            val.set_rx_enable_1536(true);
            val.set_rx_checksum_enable(true);
            val.set_no_broadcast(false);
            // val.set_pause_enable(true);
            val
        });
        ll.regs.modify_dma_cfg(|mut val| {
            val.set_rx_packet_buf_size_sel(u2::new(0b11));
            val.set_tx_packet_buf_size_sel(true);
            val.set_chksum_offload_enable(true);
            val.set_burst_length(BurstLength::Incr16.reg_value());
            // Configure 1536 bytes receive buffer size. This is sufficient for regular Ethernet
            // frames.
            val.set_dma_rx_ahb_buf_size_sel(DmaRxBufSize::new((MTU >> 6) as u8).unwrap());
            val.set_endian_swap_mgmt_descriptor(zynq7000::eth::AhbEndianess::Little);
            val.set_endian_swap_packet_data(zynq7000::eth::AhbEndianess::Little);
            val
        });
    }

    #[inline]
    pub fn clear_interrupts(&mut self) {
        self.ll.regs.write_interrupt_status(IRQ_CLEAR_ALL);
    }

    #[inline]
    pub fn enable_interrupts(&mut self) {
        self.ll.regs.write_interrupt_enable(IRQ_CONTROL);
    }

    #[inline]
    pub fn disable_interrupts(&mut self) {
        self.ll.regs.write_interrupt_disable(IRQ_CONTROL);
    }

    #[inline]
    pub fn ll(&mut self) -> &mut EthernetLowLevel {
        &mut self.ll
    }

    #[inline]
    pub fn regs(&mut self) -> &MmioEthernet<'static> {
        &self.ll.regs
    }

    #[inline]
    pub fn mdio(&mut self) -> &mdio::Mdio {
        &self.mdio
    }

    pub fn mdio_mut(&mut self) -> &mut mdio::Mdio {
        &mut self.mdio
    }

    #[inline]
    pub fn regs_mut(&mut self) -> &mut MmioEthernet<'static> {
        &mut self.ll.regs
    }

    /// This function checks whether new auto-negotiated settings require driver settings
    /// updates and perform them if necessary.
    pub fn configure_clock_and_speed_duplex(
        &mut self,
        speed: Speed,
        duplex: Duplex,
        clk_collection: &ClockDivSet,
    ) {
        if speed == self.current_speed
            && duplex == self.current_duplex
            && *clk_collection.clk_divs_for_speed(speed) == self.current_clk_divs
        {
            // No change, do nothing.
            return;
        }
        self.ll.set_tx_rx_enable(false, false);
        self.ll
            .configure_clock_and_speed_duplex(speed, duplex, clk_collection);
        self.ll.set_tx_rx_enable(true, true);
    }

    delegate::delegate! {
        to self.ll {
            #[inline]
            pub const fn id(&self) -> EthernetId;

            #[inline]
            pub fn set_rx_buf_descriptor_base_address(&mut self, addr: u32);

            #[inline]
            pub fn set_tx_buf_descriptor_base_address(&mut self, addr: u32);

            #[inline]
            pub fn set_tx_rx_enable(&mut self, tx_enable: bool, rx_enable: bool);
        }
    }
}

mod shared {
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    pub enum Ownership {
        Hardware = 0,
        Software = 1,
    }
}

/// Possibly critical errors.
#[derive(Debug, Clone, Copy, Default)]
pub struct EthErrors {
    /// According to the TMR, p.551, this condition implies a packet is dropped because the
    /// packet buffer is full. It occurs occasionally when the controller is unable to process
    /// the packets if they arrive very fast. No special action for error recovery needs to
    /// be taken, but we still report it.
    pub rx_overrun: bool,
    /// The TMR recommends re-initializing the controller and the buffer descriptors for
    /// receive and transmit paths.
    pub hresp_error: bool,
    /// The TMR recommends disabling the ethernet transmitter, re-initializing the buffer
    /// descriptors on the transmit side and re-enabling the transmitter.
    pub tx_frame_corruption_ahb_error: bool,
    /// Only set in Gigabit mode, for 10/100 mode, late collision and collisions are treated
    /// the same.
    pub tx_late_collision: bool,
    /// In 10/100 mode, this is set when the retry limit was reached.
    ///
    /// According to the TMR, p.551, this implies there are a series of collisions for which
    /// an Ethernet frame could not be sent out even with a number of retries in half-duplex mode.
    /// No drastic measures need to be taken, but this could also be an indicator for a duplex
    /// missmatch.
    pub tx_retry_limit_reached: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct InterruptResult {
    pub frame_received: bool,
    pub frame_sent: bool,
    /// Possibly indicator for high traffic, not enough descriptors available or not handled
    /// quick enough.
    pub rx_descr_read_when_used: bool,
    /// Indicator that driver is done sending all frames because it found a descriptor slot owned
    /// by the software.
    pub tx_descr_read_when_used: bool,
    /// These are full errors.
    pub errors: EthErrors,
}

impl InterruptResult {
    #[inline]
    pub fn has_errors(&self) -> bool {
        self.errors.rx_overrun
            || self.errors.hresp_error
            || self.errors.tx_frame_corruption_ahb_error
            || self.errors.tx_late_collision
            || self.errors.tx_retry_limit_reached
    }
}

/// Interrupt handler which also has dedicated handling for embassy-net.
pub(crate) fn on_interrupt(
    eth_id: EthernetId,
    wake_embassy_tx: bool,
    wake_embassy_rx: bool,
) -> InterruptResult {
    let mut eth_regs = unsafe { eth_id.steal_regs() };
    let status = eth_regs.read_interrupt_status();
    let mut clear = InterruptStatus::new_with_raw_value(0);
    let mut tx_status_clear = TxStatus::new_with_raw_value(0);
    let mut rx_status_clear = RxStatus::new_with_raw_value(0);
    let mut disable = InterruptControl::new_with_raw_value(0);
    let mut result = InterruptResult::default();

    if status.frame_sent() {
        if wake_embassy_tx {
            embassy_net::TX_WAKER.wake();
        }
        result.frame_sent = true;
        tx_status_clear.set_complete(true);
        clear.set_frame_sent(true);
    }
    if status.frame_received() {
        if wake_embassy_rx {
            embassy_net::RX_WAKER.wake();
        }
        result.frame_received = true;
        clear.set_frame_received(true);
    }
    if status.hresp_not_ok() {
        result.errors.hresp_error = true;
        clear.set_hresp_not_ok(true);
        tx_status_clear.set_hresp_not_ok(true);
        rx_status_clear.set_hresp_not_ok(true);
    }
    if status.tx_retry_limit_reached_or_late_collision() {
        let tx_status = eth_regs.read_tx_status();
        if tx_status.late_collision() {
            result.errors.tx_late_collision = true;
            tx_status_clear.set_late_collision(true);
        } else {
            result.errors.tx_retry_limit_reached = true;
            tx_status_clear.set_retry_limit_reached(true);
        }
        // Clear this in any case.
        tx_status_clear.set_collision(true);
        clear.set_tx_retry_limit_reached_or_late_collision(true);
    }
    if status.tx_descr_read_when_used() {
        result.tx_descr_read_when_used = true;
        // The interrupt status bit is cleared on a read.
        clear.set_tx_descr_read_when_used(true);
        tx_status_clear.set_read_when_used(true);
    }
    if status.tx_frame_corruption_ahb_error() {
        result.errors.tx_frame_corruption_ahb_error = true;
        // The interrupt status bit is cleared on a read.
        tx_status_clear.set_frame_corruption_ahb_error(true);
    }
    if status.rx_descr_read_when_used() {
        result.rx_descr_read_when_used = true;
        // I am guessing that those are related.
        rx_status_clear.set_buf_not_available(true);
        clear.set_rx_descr_read_when_used(true);
    }
    if status.rx_overrun() {
        if wake_embassy_rx {
            embassy_net::RX_WAKER.wake();
        }
        result.errors.rx_overrun = true;
        rx_status_clear.set_overrun(true);
        disable.set_rx_overrun(true);
        clear.set_rx_overrun(true);
    }
    eth_regs.write_interrupt_status(clear);
    eth_regs.write_interrupt_disable(disable);
    eth_regs.write_tx_status(tx_status_clear);
    eth_regs.write_rx_status(rx_status_clear);
    result
}
