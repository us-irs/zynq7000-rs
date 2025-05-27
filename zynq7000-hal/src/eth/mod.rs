use arbitrary_int::{u2, u3};
pub use zynq7000::eth::MdcClkDiv;
use zynq7000::eth::{
    BurstLength, DmaRxBufSize, GEM_0_BASE_ADDR, GEM_1_BASE_ADDR, MmioEthernet, NetworkConfig,
    SpeedMode,
};

pub use ll::{ClkConfig, EthernetLowLevel};

pub mod ll;
pub mod mdio;
pub mod rx_descr;
pub mod tx_descr;

pub const MTU: usize = 1536;

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27,
};
use crate::gpio::{
    IoPeriphPin,
    mio::{
        Mio28, Mio29, Mio30, Mio31, Mio32, Mio33, Mio34, Mio35, Mio36, Mio37, Mio38, Mio39, Mio52,
        Mio53, MioPinMarker, MuxConf, Pin,
    },
};

pub const MUX_CONF_PHY: MuxConf = MuxConf::new_with_l0();
pub const MUX_CONF_MDIO: MuxConf = MuxConf::new_with_l3(u3::new(0b100));

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

pub trait TxClk: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait TxCtrl: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait TxData0: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait TxData1: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait TxData2: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait TxData3: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait RxClk: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait RxCtrl: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait RxData0: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait RxData1: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait RxData2: MioPinMarker {
    const ETH_ID: EthernetId;
}
pub trait RxData3: MioPinMarker {
    const ETH_ID: EthernetId;
}

pub trait MdClk: MioPinMarker {}
pub trait MdIo: MioPinMarker {}

impl MdClk for Pin<Mio52> {}
impl MdIo for Pin<Mio53> {}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxClk for Pin<Mio16> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxCtrl for Pin<Mio21> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData0 for Pin<Mio17> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData1 for Pin<Mio18> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData2 for Pin<Mio19> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl TxData3 for Pin<Mio20> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxClk for Pin<Mio22> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxCtrl for Pin<Mio27> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData0 for Pin<Mio23> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData1 for Pin<Mio24> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData2 for Pin<Mio25> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl RxData3 for Pin<Mio26> {
    const ETH_ID: EthernetId = EthernetId::Eth0;
}

impl TxClk for Pin<Mio28> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxCtrl for Pin<Mio33> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData0 for Pin<Mio29> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData1 for Pin<Mio30> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData2 for Pin<Mio31> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl TxData3 for Pin<Mio32> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxClk for Pin<Mio34> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxCtrl for Pin<Mio39> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData0 for Pin<Mio35> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData1 for Pin<Mio36> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData2 for Pin<Mio37> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}
impl RxData3 for Pin<Mio38> {
    const ETH_ID: EthernetId = EthernetId::Eth1;
}

#[derive(Debug, Clone, Copy)]
pub struct EthernetConfig {
    pub clk_config: ClkConfig,
    pub mdc_clk_div: MdcClkDiv,
    pub mac_address: [u8; 6],
}

impl EthernetConfig {
    pub fn new(clk_config: ClkConfig, mdc_clk_div: MdcClkDiv, mac_address: [u8; 6]) -> Self {
        Self {
            clk_config,
            mdc_clk_div,
            mac_address,
        }
    }
}

pub struct Ethernet {
    ll: ll::EthernetLowLevel,
    mdio: mdio::Mdio,
}

impl Ethernet {
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_mio<
        TxClkPin: TxClk,
        TxCtrlPin: TxCtrl,
        TxData0Pin: TxData0,
        TxData1Pin: TxData1,
        TxData2Pin: TxData2,
        TxData3Pin: TxData3,
        RxClkPin: RxClk,
        RxCtrlPin: RxCtrl,
        RxData0Pin: RxData0,
        RxData1Pin: RxData1,
        RxData2Pin: RxData2,
        RxData3Pin: RxData3,
        MdClkPin: MdClk,
        MdIoPin: MdIo,
    >(
        mut ll: ll::EthernetLowLevel,
        config: EthernetConfig,
        tx_clk: TxClkPin,
        tx_ctrl: TxCtrlPin,
        tx_data: (TxData0Pin, TxData1Pin, TxData2Pin, TxData3Pin),
        rx_clk: RxClkPin,
        rx_ctrl: RxCtrlPin,
        rx_data: (RxData0Pin, RxData1Pin, RxData2Pin, RxData3Pin),
        md_pins: Option<(MdClkPin, MdIoPin)>,
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
            // Disable output driver.
            .with_tri_enable(true)
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
        ll.configure_clock(config.clk_config);
        let mut mdio = mdio::Mdio::new(&ll, true);
        mdio.configure_clock_div(config.mdc_clk_div);
        Ethernet { ll, mdio }
    }

    pub fn new(mut ll: EthernetLowLevel, config: EthernetConfig) -> Self {
        Self::common_init(&mut ll, config.mac_address);
        ll.configure_clock(config.clk_config);
        let mut mdio = mdio::Mdio::new(&ll, true);
        mdio.configure_clock_div(config.mdc_clk_div);
        Ethernet { ll, mdio }
    }

    fn common_init(ll: &mut EthernetLowLevel, mac_address: [u8; 6]) {
        ll.enable_peripheral_clock();
        ll.reset(3);
        ll.initialize();
        // By default, only modify critical network control bits to retain user configuration
        // like the MDC clock divisor.
        ll.regs.modify_net_cfg(|mut net_cfg| {
            net_cfg.set_full_duplex(true);
            net_cfg.set_gigabit_enable(true);
            net_cfg.set_speed_mode(SpeedMode::High100Mbps);
            net_cfg
        });
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
        // TODO
        ll.regs.modify_dma_cfg(|mut val| {
            val.set_rx_packet_buf_size_sel(u2::new(0b11));
            val.set_tx_packet_buf_size_sel(true);
            val.set_burst_length(BurstLength::Incr16.reg_value());
            // Configure 1536 bytes receive buffer size. This is sufficient for regular Ethernet
            // frames.
            val.set_dma_rx_ahb_buf_size_sel(DmaRxBufSize::new((MTU >> 6) as u8).unwrap());
            val.set_endian_swap_mgmt_descriptor(zynq7000::eth::AhbEndianess::Little);
            val
        });
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
    pub fn regs_mut(&mut self) -> &mut MmioEthernet<'static> {
        &mut self.ll.regs
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
