use arbitrary_int::{prelude::*, u6};
use zynq7000::{
    eth::{InterruptControl, NetworkControl, RxStatus, TxStatus},
    slcr::reset::EthernetReset,
};

use crate::{clocks::IoClocks, enable_amba_peripheral_clock, slcr::Slcr, time::Hertz};

use super::{EthernetId, PsEthernet as _};

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Duplex {
    Half,
    Full,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockDivisors {
    pub divisor_0: u6,
    pub divisor_1: u6,
}

impl ClockDivisors {
    pub const fn new(divisor_0: u6, divisor_1: u6) -> Self {
        Self {
            divisor_0,
            divisor_1,
        }
    }

    /// Calls [Self::calculate_for_rgmii], assuming that the IO clock is the reference clock,
    /// which is the default clock for the Ethernet module.
    pub fn calculate_for_rgmii_and_io_clock(
        io_clks: &IoClocks,
        target_speed: Speed,
    ) -> (Self, u32) {
        Self::calculate_for_rgmii(io_clks.ref_clk(), target_speed)
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
        for div_1 in 1..=u6::MAX.as_usize() {
            for div_0 in 1..=u6::MAX.as_usize() {
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

/// Full clock configuration for the ethernet peripheral.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    pub src_sel: zynq7000::slcr::clocks::SrcSelIo,
    pub use_emio_tx_clk: bool,
    pub divs: ClockDivisors,
    /// Enable the clock.
    pub enable: bool,
}

impl ClockConfig {
    pub const fn new(divs: ClockDivisors) -> Self {
        Self {
            src_sel: zynq7000::slcr::clocks::SrcSelIo::IoPll,
            use_emio_tx_clk: false,
            divs,
            enable: true,
        }
    }
}

/// This is a set of clock configuration for all relevant speed settings.
///
/// Generally, the clock need to be re-configured each time the speed settings change, for example
/// after a completed auto-negotiation process. The necessary clock configurations for each speed
/// setting can be pre-calculated and stored using this data structure.
#[derive(Debug, Clone, Copy)]
pub struct ClockDivSet {
    pub cfg_10_mbps: ClockDivisors,
    pub cfg_100_mbps: ClockDivisors,
    pub cfg_1000_mbps: ClockDivisors,
}

impl ClockDivSet {
    pub const fn new(
        cfg_10_mbps: ClockDivisors,
        cfg_100_mbps: ClockDivisors,
        cfg_1000_mbps: ClockDivisors,
    ) -> Self {
        Self {
            cfg_10_mbps,
            cfg_100_mbps,
            cfg_1000_mbps,
        }
    }

    #[inline]
    pub fn clk_divs_for_speed(&self, speed: Speed) -> &ClockDivisors {
        match speed {
            Speed::Mbps10 => &self.cfg_10_mbps,
            Speed::Mbps100 => &self.cfg_100_mbps,
            Speed::Mbps1000 => &self.cfg_1000_mbps,
        }
    }

    /// Calls [Self::calculate_for_rgmii], assuming that the IO clock is the reference clock,
    /// which is the default clock for the Ethernet module.
    pub fn calculate_for_rgmii_and_io_clock(io_clks: &IoClocks) -> (Self, [u32; 3]) {
        Self::calculate_for_rgmii(io_clks.ref_clk())
    }

    /// Calculate the best clock configuration (divisors) for the given reference clock
    /// and desired target speed when using a RGMII interface.
    ///
    /// Usually, the reference clock will be the IO clock.
    ///
    /// Returns a tuple where the first entry is the calcualted clock configuration
    /// and the second entry is the difference between calculated clock speed for the divisors
    /// and the target speed. Ideally, this difference should be 0.
    pub fn calculate_for_rgmii(ref_clk: Hertz) -> (Self, [u32; 3]) {
        let (cfg_10_mbps, error_10_mbps) =
            ClockDivisors::calculate_for_rgmii(ref_clk, Speed::Mbps10);
        let (cfg_100_mbps, error_100_mbps) =
            ClockDivisors::calculate_for_rgmii(ref_clk, Speed::Mbps100);
        let (cfg_1000_mbps, error_1000_mbps) =
            ClockDivisors::calculate_for_rgmii(ref_clk, Speed::Mbps1000);
        (
            Self::new(cfg_10_mbps, cfg_100_mbps, cfg_1000_mbps),
            [error_10_mbps, error_100_mbps, error_1000_mbps],
        )
    }
}

/// Ethernet low-level interface.
///
/// Basic building block for higher-level abstraction.
pub struct EthernetLowLevel {
    id: EthernetId,
    /// Register block. Direct public access is allowed to allow low-level operations.
    pub regs: zynq7000::eth::MmioRegisters<'static>,
}

impl EthernetLowLevel {
    /// Creates a new instance of the Ethernet low-level interface.
    ///
    /// Returns [None] if the given registers block base address does not correspond to a valid
    /// Ethernet peripheral.
    #[inline]
    pub fn new(regs: zynq7000::eth::MmioRegisters<'static>) -> Option<Self> {
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
                    EthernetId::Eth0 => zynq7000::eth::Registers::new_mmio_fixed_0(),
                    EthernetId::Eth1 => zynq7000::eth::Registers::new_mmio_fixed_1(),
                }
            },
        }
    }

    pub fn reset(&mut self, cycles: usize) {
        reset(self.id, cycles);
    }

    #[inline]
    pub fn enable_peripheral_clock(&mut self) {
        let periph_sel = match self.id {
            EthernetId::Eth0 => crate::PeriphSelect::Gem0,
            EthernetId::Eth1 => crate::PeriphSelect::Gem1,
        };
        enable_amba_peripheral_clock(periph_sel);
    }

    /// Completely configures the clock based on the provided [ClockConfig].
    ///
    /// This should be called once when initializing the peripheral.
    pub fn configure_clock(&mut self, cfg: ClockConfig, enable_rx_clock: bool) {
        unsafe {
            Slcr::with(|regs| {
                let (ptr_gig_eth_clk_ctrl, ptr_gig_eth_rclk_ctrl) = self.id().clk_config_regs(regs);
                let mut gig_eth_clk_ctrl_val = core::ptr::read_volatile(ptr_gig_eth_clk_ctrl);
                gig_eth_clk_ctrl_val.set_srcsel(cfg.src_sel);
                gig_eth_clk_ctrl_val.set_divisor_0(cfg.divs.divisor_0);
                gig_eth_clk_ctrl_val.set_divisor_1(cfg.divs.divisor_1);
                gig_eth_clk_ctrl_val.set_use_emio_tx_clk(cfg.use_emio_tx_clk);
                gig_eth_clk_ctrl_val.set_clk_act(cfg.enable);
                core::ptr::write_volatile(ptr_gig_eth_clk_ctrl, gig_eth_clk_ctrl_val);

                if enable_rx_clock {
                    let mut gig_eth_rclk_ctrl_val = core::ptr::read_volatile(ptr_gig_eth_rclk_ctrl);
                    gig_eth_rclk_ctrl_val.set_clk_enable(true);
                    core::ptr::write_volatile(ptr_gig_eth_rclk_ctrl, gig_eth_rclk_ctrl_val);
                }
            })
        }
    }

    /// Re-configures the clock divisors for the Ethernet peripheral.
    ///
    /// This might be necessary after auto-negotiation of speed settings.
    pub fn configure_clock_divs(&mut self, cfg: ClockDivisors) {
        unsafe {
            Slcr::with(|regs| {
                let (ptr_gig_eth_clk_ctrl, _ptr_gig_eth_rclk_ctrl) =
                    self.id().clk_config_regs(regs);
                let mut gig_eth_clk_ctrl_val = core::ptr::read_volatile(ptr_gig_eth_clk_ctrl);
                gig_eth_clk_ctrl_val.set_divisor_0(cfg.divisor_0);
                gig_eth_clk_ctrl_val.set_divisor_1(cfg.divisor_1);
                core::ptr::write_volatile(ptr_gig_eth_clk_ctrl, gig_eth_clk_ctrl_val);
            })
        }
    }

    /// Can be used after auto-negotiation to update all relevant speed and duplex
    /// parameter of the ethernet peripheral.
    ///
    /// It is probably a good idea to disable the receiver and transmitter while doing this.
    /// This function calls [Self::configure_clock_for_speed] and [Self::set_speed_and_duplex].
    pub fn configure_clock_and_speed_duplex(
        &mut self,
        speed: Speed,
        duplex: Duplex,
        clk_collection: &ClockDivSet,
    ) {
        self.configure_clock_for_speed(speed, clk_collection);
        self.set_speed_and_duplex(speed, duplex);
    }

    pub fn configure_clock_for_speed(&mut self, speed: Speed, clk_collection: &ClockDivSet) {
        match speed {
            Speed::Mbps10 => self.configure_clock_divs(clk_collection.cfg_10_mbps),
            Speed::Mbps100 => self.configure_clock_divs(clk_collection.cfg_100_mbps),
            Speed::Mbps1000 => self.configure_clock_divs(clk_collection.cfg_1000_mbps),
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

    /// This function sets the speed and duplex mode of the Ethernet interface.
    ///
    /// This should be called after a completed auto-negotiation process with the negotiated
    /// settings.
    pub fn set_speed_and_duplex(&mut self, speed: Speed, duplex: Duplex) {
        self.regs.modify_net_cfg(|mut val| {
            val.set_full_duplex(duplex == Duplex::Full);
            match speed {
                Speed::Mbps10 => {
                    val.set_speed_mode(zynq7000::eth::SpeedMode::Low10Mbps);
                    val.set_gigabit_enable(false);
                    val
                }
                Speed::Mbps100 => {
                    val.set_speed_mode(zynq7000::eth::SpeedMode::High100Mbps);
                    val.set_gigabit_enable(false);
                    val
                }
                Speed::Mbps1000 => {
                    val.set_gigabit_enable(true);
                    val
                }
            }
        });
    }

    /// Allows enabling/disabling ethernet receiver and transmitter respectively.
    #[inline]
    pub fn set_tx_rx_enable(&mut self, tx_enable: bool, rx_enable: bool) {
        self.regs.modify_net_ctrl(|mut val| {
            val.set_rx_enable(rx_enable);
            val.set_tx_enable(tx_enable);
            val
        });
    }

    /// Performs initialization according to TRM p.541.
    ///
    /// These steps do not include any resets or clock configuration.
    pub fn initialize(&mut self, reset_rx_tx_queue_base_addr: bool) {
        let mut ctrl_val = NetworkControl::new_with_raw_value(0);
        self.regs.write_net_ctrl(ctrl_val);
        // Now clear statistics.
        ctrl_val.set_clear_statistics(true);
        self.regs.write_net_ctrl(ctrl_val);
        self.regs.write_tx_status(TxStatus::new_clear_all());
        self.regs.write_rx_status(RxStatus::new_clear_all());
        self.regs
            .write_interrupt_disable(InterruptControl::new_clear_all());
        if reset_rx_tx_queue_base_addr {
            self.regs.write_rx_buf_queue_base_addr(0);
            self.regs.write_tx_buf_queue_base_addr(0);
        }
    }

    #[inline]
    pub const fn id(&self) -> EthernetId {
        self.id
    }
}

/// Resets the Ethernet peripheral with the given ID.
pub fn reset(id: EthernetId, cycles: usize) {
    let assert_reset = match id {
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
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_eth(EthernetReset::DEFAULT);
        });
    }
}
