use arbitrary_int::{u2, u4, u5};

#[derive(Clone, Debug)]
pub struct PhyIdentifier {
    pub oui: u32,
    pub model: u8,
    pub rev: u8,
}

// Organizational Unique Identifier for Marvell 88E1518 PHY
pub const MARVELL_88E1518_OUI: u32 = 0x005043;
pub const MARVELL_88E1518_MODELL_NUMBER: u8 = 0b011101;

#[bitbybit::bitenum(u5, exhaustive = false)]
pub enum MarvellRegistersPage0 {
    CopperControl = 0,
    CopperStatus = 1,
    IdReg1 = 2,
    IdReg2 = 3,
    CopperSpecificStatus = 17,
    PageSel = 22,
}

#[bitbybit::bitfield(u16)]
pub struct CopperControlRegister {
    #[bit(15, rw)]
    copper_reset: bool,
    #[bit(14, rw)]
    loopback: bool,
    #[bit(12, rw)]
    auto_negotiation_enable: bool,
    #[bit(11, rw)]
    power_down: bool,
    #[bit(10, rw)]
    isolate: bool,
    #[bit(9, rw)]
    restart_auto_negotiation: bool,
    /// 1: Full-duplex, 0: Half-duplex
    #[bit(8, rw)]
    copper_duplex_mode: bool,
    #[bits([13, 6], rw)]
    speed_selection: u2,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum LatchingLinkStatus {
    Up = 1,
    DownSinceLastRead = 0,
}

#[bitbybit::bitfield(u16)]
pub struct CopperStatusRegister {
    /// Always 0, the 100BASE-T4 protocol is not available on Marvell 88E15XX.
    #[bit(15, r)]
    p_100_base_t4: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(14, r)]
    p_100_base_x_full_duplex: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(13, r)]
    p_100_base_x_half_duplex: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(12, r)]
    p_10_base_t_full_duplex: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(11, r)]
    p_10_base_t_half_duplex: bool,
    /// Always 0 for Marvell 88E15XX
    #[bit(10, r)]
    p_100_base_t2_full_duplex: bool,
    /// Always 0 for Marvell 88E15XX
    #[bit(9, r)]
    p_100_base_t2_half_duplex: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(8, r)]
    extended_status: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(6, r)]
    mf_preamble_suppression: bool,
    #[bit(5, r)]
    auto_negotiation_complete: bool,
    // Latching high register bit.
    #[bit(4, r)]
    copper_remote_fault: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(3, r)]
    auto_negotation_ability: bool,
    // Latching low register bit. For the current link status, this register should be read back
    // to back, or the link real time register (17_0.10) should be read
    #[bit(2, r)]
    copper_link_status: LatchingLinkStatus,
    // Latching high register bit.
    #[bit(1, r)]
    jabber_detect: bool,
    /// Always 1 for Marvell 88E15XX
    #[bit(0, r)]
    extended_capabilities: bool,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum PhySpeedBits {
    Reserved = 0b11,
    Mbps1000 = 0b10,
    Mbps100 = 0b01,
    Mbps10 = 0b00,
}

impl PhySpeedBits {
    #[inline]
    pub fn as_zynq7000_eth_speed(&self) -> Option<zynq7000_hal::eth::Speed> {
        match self {
            PhySpeedBits::Reserved => None,
            PhySpeedBits::Mbps1000 => Some(zynq7000_hal::eth::Speed::Mbps1000),
            PhySpeedBits::Mbps100 => Some(zynq7000_hal::eth::Speed::Mbps100),
            PhySpeedBits::Mbps10 => Some(zynq7000_hal::eth::Speed::Mbps10),
        }
    }
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum PhyDuplexBit {
    Full = 1,
    Half = 0,
}

impl PhyDuplexBit {
    #[inline]
    pub fn as_zynq7000_eth_duplex(&self) -> zynq7000_hal::eth::Duplex {
        match self {
            PhyDuplexBit::Full => zynq7000_hal::eth::Duplex::Full,
            PhyDuplexBit::Half => zynq7000_hal::eth::Duplex::Half,
        }
    }
}

#[bitbybit::bitfield(u16)]
pub struct CopperSpecificStatusRegister {
    #[bits(14..=15, r)]
    speed: PhySpeedBits,
    #[bit(13, r)]
    duplex: PhyDuplexBit,
    /// Latching high register bit.
    #[bit(12, r)]
    page_received: bool,
    /// This is 1 when auto-negotiation is not enabled.
    #[bit(11, r)]
    speed_and_duplex_resolved: bool,
    /// This is the real-time link status.
    #[bit(10, r)]
    copper_link: bool,
    #[bit(9, r)]
    transmit_pause_enabled: bool,
    #[bit(8, r)]
    received_pause_enabled: bool,
    #[bit(6, r)]
    mdi_crossover_status: bool,
    #[bit(4, r)]
    copper_energy_detect_status: bool,
    #[bit(3, r)]
    global_link_status: bool,
    #[bit(1, r)]
    polarity: bool,
    #[bit(0, r)]
    jabber: bool,
}

pub struct Marvell88E1518Phy {
    mdio: zynq7000_hal::eth::mdio::Mdio,
    addr: u5,
}

impl Marvell88E1518Phy {
    pub fn new_autoprobe_addr(mdio: &mut zynq7000_hal::eth::mdio::Mdio) -> Option<(Self, u4)> {
        for addr in 0..32 {
            let phy_id_1 =
                mdio.read_blocking(u5::new(addr), MarvellRegistersPage0::IdReg1.raw_value());
            let phy_id_2 =
                mdio.read_blocking(u5::new(addr), MarvellRegistersPage0::IdReg2.raw_value());
            // PHY ID 1 contains bits 3 to 18 of the OUI in the goofy IEEE ordering scheme,
            // which corresponds to bit \[21:6\] of the OUI.
            // PHY ID 2 contains bits 19 to 24 which correspond to bits \[5:0\] of the OUI.
            let oui = ((phy_id_1 as u32) << 6) | ((phy_id_2 >> 10) & 0b111111) as u32;
            let model_number = ((phy_id_2 >> 4) & 0b111111) as u8;
            let revision_number = u4::new((phy_id_2 & 0b1111) as u8);
            if oui == MARVELL_88E1518_OUI && model_number == MARVELL_88E1518_MODELL_NUMBER {
                return Some((
                    Self {
                        mdio: unsafe { mdio.clone() },
                        addr: u5::new(addr),
                    },
                    revision_number,
                ));
            }
        }
        None
    }

    pub fn new(mdio: zynq7000_hal::eth::mdio::Mdio, addr: u5) -> Self {
        Self { mdio, addr }
    }

    pub fn reset(&mut self) {
        let mut ctrl = CopperControlRegister::new_with_raw_value(
            self.mdio
                .read_blocking(self.addr, MarvellRegistersPage0::CopperControl.raw_value()),
        );
        ctrl.set_copper_reset(true);
        self.mdio.write_blocking(
            self.addr,
            MarvellRegistersPage0::CopperControl.raw_value(),
            ctrl.raw_value(),
        );
    }

    pub fn restart_auto_negotiation(&mut self) {
        let mut ctrl = CopperControlRegister::new_with_raw_value(
            self.mdio
                .read_blocking(self.addr, MarvellRegistersPage0::CopperControl.raw_value()),
        );
        ctrl.set_auto_negotiation_enable(true);
        ctrl.set_restart_auto_negotiation(true);
        self.mdio.write_blocking(
            self.addr,
            MarvellRegistersPage0::CopperControl.raw_value(),
            ctrl.raw_value(),
        );
    }

    pub fn read_copper_status(&mut self) -> CopperStatusRegister {
        let raw_value = self
            .mdio
            .read_blocking(self.addr, MarvellRegistersPage0::CopperStatus.raw_value());
        CopperStatusRegister::new_with_raw_value(raw_value)
    }

    pub fn read_copper_specific_status_register_1(&mut self) -> CopperSpecificStatusRegister {
        let raw_value = self.mdio.read_blocking(
            self.addr,
            MarvellRegistersPage0::CopperSpecificStatus.raw_value(),
        );
        CopperSpecificStatusRegister::new_with_raw_value(raw_value)
    }
}
