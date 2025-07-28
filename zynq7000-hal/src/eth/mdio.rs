use arbitrary_int::{u2, u5};
use zynq7000::eth::{MdcClkDiv, PhyMaintenance};

use super::{EthernetId, ll::EthernetLowLevel};

pub struct Mdio {
    regs: zynq7000::eth::MmioEthernet<'static>,
    clause22: bool,
}

impl Mdio {
    pub fn new(ll: &EthernetLowLevel, clause22: bool) -> Self {
        Self {
            regs: unsafe { ll.regs.clone() },
            clause22,
        }
    }

    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees of the HAL.
    pub unsafe fn steal(eth_id: EthernetId, clause22: bool) -> Self {
        Self {
            regs: unsafe { eth_id.steal_regs() },
            clause22,
        }
    }

    /// Steals the MDIO handle from the given Ethernet low-level interface.
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees of the HAL.
    pub unsafe fn clone(&self) -> Self {
        Self {
            regs: unsafe { self.regs.clone() },
            clause22: self.clause22,
        }
    }

    #[inline]
    pub fn configure_clock_div(&mut self, clk_div: MdcClkDiv) {
        self.regs.modify_net_cfg(|mut val| {
            val.set_mdc_clk_div(clk_div);
            val
        });
    }

    pub fn read_blocking(&mut self, phy_addr: u5, reg_addr: u5) -> u16 {
        while !self.regs.read_net_status().phy_mgmt_idle() {}
        self.regs.write_phy_maintenance(
            PhyMaintenance::builder()
                .with_clause_22(self.clause22)
                .with_op(zynq7000::eth::PhyOperation::Read)
                .with_phy_addr(phy_addr)
                .with_reg_addr(reg_addr)
                .with_must_be_0b10(u2::new(0b10))
                .with_data(0x0000)
                .build(),
        );
        while !self.regs.read_net_status().phy_mgmt_idle() {}
        self.regs.read_phy_maintenance().data()
    }

    pub fn write_blocking(&mut self, phy_addr: u5, reg_addr: u5, data: u16) {
        while !self.regs.read_net_status().phy_mgmt_idle() {}
        self.regs.write_phy_maintenance(
            PhyMaintenance::builder()
                .with_clause_22(self.clause22)
                .with_op(zynq7000::eth::PhyOperation::Write)
                .with_phy_addr(phy_addr)
                .with_reg_addr(reg_addr)
                .with_must_be_0b10(u2::new(0b10))
                .with_data(data)
                .build(),
        );
        while !self.regs.read_net_status().phy_mgmt_idle() {}
    }
}
