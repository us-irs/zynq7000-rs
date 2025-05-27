use arbitrary_int::{Number, u6};
use zynq7000::{
    eth::{InterruptControl, NetworkControl, RxStatus, TxStatus},
    slcr::reset::EthernetReset,
};

use crate::{enable_amba_peripheral_clock, slcr::Slcr, time::Hertz};

use super::{EthernetId, PsEthernet as _};

pub struct EthernetLowLevel {
    id: EthernetId,
    pub regs: zynq7000::eth::MmioEthernet<'static>,
}

#[derive(Debug, Clone, Copy)]
pub struct ClkConfig {
    pub src_sel: zynq7000::slcr::clocks::SrcSelIo,
    pub use_emio_tx_clk: bool,
    pub divisor_0: u6,
    pub divisor_1: u6,
    /// Enable the clock.
    pub enable: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    Mbps10,
    Mbps100,
    Mbps1000,
}

impl Speed {
    pub const fn rgmii_clk_rate(&self) -> Hertz {
        match self {
            Speed::Mbps10 => Hertz::from_raw(2_500_000),
            Speed::Mbps100 => Hertz::from_raw(25_000_000),
            Speed::Mbps1000 => Hertz::from_raw(125_000_000),
        }
    }
}

impl ClkConfig {
    pub const fn new(divisor_0: u6, divisor_1: u6) -> Self {
        Self {
            src_sel: zynq7000::slcr::clocks::SrcSelIo::IoPll,
            use_emio_tx_clk: false,
            divisor_0,
            divisor_1,
            enable: true,
        }
    }

    /// Calculate the best clock configuration (divisors) for the given reference clock
    /// and desired target speed when using a RGMII interface.
    ///
    /// Usually, the reference clock will be the IO clock.
    ///
    /// Returns a tuple where the first entry is the calcualted clock configuration
    /// and the second entry is the difference between calculated clock speed for the divisors
    /// and the target speed. Ideally, this difference should be 0.
    pub fn calculate_for_rgmii(ref_clk: Hertz, target_speed: Speed) -> (Self, u32) {
        let mut smallest_diff = u32::MAX;
        let target_speed = target_speed.rgmii_clk_rate();
        let mut best_div_0 = u6::new(0);
        let mut best_div_1 = u6::new(0);
        for div_0 in 0..=u6::MAX.as_usize() {
            for div_1 in 0..=u6::MAX.as_usize() {
                let clk_rate = ref_clk.raw() / div_0 as u32 / div_1 as u32;
                let diff = (target_speed.raw() as i64 - clk_rate as i64).unsigned_abs() as u32;
                if diff < smallest_diff {
                    smallest_diff = diff;
                    best_div_0 = u6::new(div_0 as u8);
                    best_div_1 = u6::new(div_1 as u8);
                }
                // We found a perfect match. No need to continue.
                if diff == 0 {
                    break;
                }
            }
        }
        (Self::new(best_div_0, best_div_1), smallest_diff)
    }
}

/// Ethernet low-level interface.
impl EthernetLowLevel {
    /// Creates a new instance of the Ethernet low-level interface.
    #[inline]
    pub fn new(regs: zynq7000::eth::MmioEthernet<'static>) -> Option<Self> {
        regs.id()?;
        Some(EthernetLowLevel {
            id: regs.id().unwrap(),
            regs,
        })
    }

    /// Create a low-level instance for the given [EthernetId].
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees of the HAL.
    #[inline]
    pub const unsafe fn steal(id: EthernetId) -> Self {
        Self {
            id,
            regs: unsafe {
                match id {
                    EthernetId::Eth0 => zynq7000::eth::Ethernet::new_mmio_fixed_0(),
                    EthernetId::Eth1 => zynq7000::eth::Ethernet::new_mmio_fixed_1(),
                }
            },
        }
    }

    pub fn reset(&mut self, cycles: usize) {
        let assert_reset = match self.id {
            EthernetId::Eth0 => EthernetReset::builder()
                .with_gem1_ref_rst(false)
                .with_gem0_ref_rst(true)
                .with_gem1_rx_rst(false)
                .with_gem0_rx_rst(true)
                .with_gem1_cpu1x_rst(false)
                .with_gem0_cpu1x_rst(true)
                .build(),
            EthernetId::Eth1 => EthernetReset::builder()
                .with_gem1_ref_rst(true)
                .with_gem0_ref_rst(false)
                .with_gem1_rx_rst(true)
                .with_gem0_rx_rst(false)
                .with_gem1_cpu1x_rst(true)
                .with_gem0_cpu1x_rst(false)
                .build(),
        };
        unsafe {
            Slcr::with(|regs| {
                regs.reset_ctrl().write_eth(assert_reset);
                for _ in 0..cycles {
                    cortex_ar::asm::nop();
                }
                regs.reset_ctrl().write_eth(EthernetReset::DEFAULT);
            });
        }
    }

    #[inline]
    pub fn enable_peripheral_clock(&mut self) {
        let periph_sel = match self.id {
            EthernetId::Eth0 => crate::PeripheralSelect::Gem0,
            EthernetId::Eth1 => crate::PeripheralSelect::Gem1,
        };
        enable_amba_peripheral_clock(periph_sel);
    }

    #[inline]
    pub fn configure_clock(&mut self, cfg: ClkConfig) {
        unsafe {
            Slcr::with(|regs| {
                regs.clk_ctrl().modify_gem_0_clk_ctrl(|mut val| {
                    val.set_srcsel(cfg.src_sel);
                    val.set_divisor_0(cfg.divisor_0);
                    val.set_divisor_1(cfg.divisor_1);
                    val.set_use_emio_tx_clk(cfg.use_emio_tx_clk);
                    val.set_clk_act(cfg.enable);
                    val
                });
            })
        }
    }

    #[inline]
    pub fn set_promiscous_mode(&mut self, enable: bool) {
        self.regs.modify_net_cfg(|mut val| {
            val.set_copy_all_frames(enable);
            val
        });
    }

    #[inline]
    pub fn set_rx_buf_descriptor_base_address(&mut self, addr: u32) {
        self.regs.write_rx_buf_queue_base_addr(addr);
    }

    #[inline]
    pub fn set_tx_buf_descriptor_base_address(&mut self, addr: u32) {
        self.regs.write_tx_buf_queue_base_addr(addr);
    }

    /// Performs initialization according to TRM p.541.
    ///
    /// These steps do not include any resets or clock configuration.
    pub fn initialize(&mut self) {
        let mut ctrl_val = NetworkControl::new_with_raw_value(0);
        self.regs.write_net_ctrl(ctrl_val);
        // Now clear statistics.
        ctrl_val.set_clear_statistics(true);
        self.regs.write_net_ctrl(ctrl_val);
        self.regs.write_tx_status(TxStatus::new_clear_all());
        self.regs.write_rx_status(RxStatus::new_clear_all());
        self.regs
            .write_interrupt_disable(InterruptControl::new_clear_all());
        self.regs.write_rx_buf_queue_base_addr(0);
        self.regs.write_tx_buf_queue_base_addr(0);
    }

    #[inline]
    pub const fn id(&self) -> EthernetId {
        self.id
    }
}
