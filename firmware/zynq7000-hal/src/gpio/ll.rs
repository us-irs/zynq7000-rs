//! Low-level GPIO access module.
use embedded_hal::digital::PinState;
use zynq7000::gpio::{
    InterruptTypeRegister, MaskedOutput, MmioRegisters, PolarityRegister, Registers,
};

use crate::{gpio::asynch::Sensitivity, slcr::Slcr};

use super::{PinIsOutputOnly, mio::MuxConfig};

/// MIO or EMIO pin offset.
#[derive(Debug, Clone, Copy)]
pub enum PinOffset {
    /// MIO pin offset, 0 to 53.
    Mio(usize),
    /// EMIO pin offset, 0 to 63.
    Emio(usize),
}

impl PinOffset {
    /// Returs [None] if offset is larger than 53.
    pub const fn new_for_mio(offset: usize) -> Option<Self> {
        if offset > 53 {
            return None;
        }
        Some(PinOffset::Mio(offset))
    }

    /// Returs [None] if offset is larger than 63.
    pub const fn new_for_emio(offset: usize) -> Option<Self> {
        if offset > 63 {
            return None;
        }
        Some(PinOffset::Emio(offset))
    }

    /// Pin is a MIO pin.
    pub fn is_mio(&self) -> bool {
        match self {
            PinOffset::Mio(_) => true,
            PinOffset::Emio(_) => false,
        }
    }
}

impl PinOffset {
    /// Raw offset without the MIO or EMIO information.
    pub fn offset(&self) -> usize {
        match self {
            PinOffset::Mio(offset) => *offset,
            PinOffset::Emio(offset) => *offset,
        }
    }
}

/// Interrupt configuration registers of the bank of a pin.
struct InterruptRegs {
    /// Pin offset inside the bank.
    local_offset: usize,
    enable: *mut u32,
    status: *mut u32,
    interrupt_type: *mut InterruptTypeRegister,
    polarity: *mut PolarityRegister,
    any_edge: *mut u32,
}

impl InterruptRegs {
    #[inline(always)]
    fn new(bank: &zynq7000::gpio::MmioBankControlRegisters<'_>, local_offset: usize) -> Self {
        Self {
            local_offset,
            enable: bank.pointer_to_enable_interrupt(),
            status: bank.pointer_to_interrupt_status(),
            interrupt_type: bank.pointer_to_interrupt_type(),
            polarity: bank.pointer_to_interrupt_polarity(),
            any_edge: bank.pointer_to_interrupt_any_sensitivity(),
        }
    }
}

/// Low-level access to a single GPIO pin.
#[derive(Debug)]
pub struct LowLevelGpio {
    offset: PinOffset,
    regs: MmioRegisters<'static>,
}

impl LowLevelGpio {
    /// Create a low-level pin. This does not configure the pin.
    pub fn new(offset: PinOffset) -> Self {
        Self {
            offset,
            // SAFETY: Each instance only changes the bits of its own pin.
            regs: unsafe { Registers::new_mmio_fixed() },
        }
    }

    /// Pin offset.
    pub fn offset(&self) -> PinOffset {
        self.offset
    }

    /// Convert the pin into an output pin.
    pub fn configure_as_output_push_pull(&mut self, init_level: PinState) {
        let (offset, dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        if self.offset.is_mio() {
            // Tri-state bit must be 0 for the output driver to work.
            self.reconfigure_slcr_mio_cfg(false, None, Some(MuxConfig::new_for_gpio()));
        }
        // The registers are shared by all pins of a bank.
        critical_section::with(|_| {
            // SAFETY: The pointers come from the GPIO register block.
            unsafe {
                let curr_dirm = core::ptr::read_volatile(dirm);
                core::ptr::write_volatile(dirm, curr_dirm | (1 << offset));
                let curr_outen = core::ptr::read_volatile(outen);
                core::ptr::write_volatile(outen, curr_outen | (1 << offset));
            }
        });
        self.write_state(init_level);
    }

    /// Enable the interrupt with the given sensitivity.
    pub fn enable_interrupt(&mut self, sensitivity: Sensitivity) {
        let regs = self.get_interrupt_regs();
        let offset = regs.local_offset;
        // The configuration registers are shared by all pins of a bank.
        critical_section::with(|_| match sensitivity {
            // SAFETY: The pointers come from the GPIO register block.
            Sensitivity::Level(level) => unsafe {
                let interrupt_type = core::ptr::read_volatile(regs.interrupt_type)
                    .with_bit(offset, zynq7000::gpio::InterruptType::Level);
                core::ptr::write_volatile(regs.interrupt_type, interrupt_type);
                let polarity =
                    core::ptr::read_volatile(regs.polarity).with_bit(offset, level.polarity_bit());
                core::ptr::write_volatile(regs.polarity, polarity);
            },
            // SAFETY: The pointers come from the GPIO register block.
            Sensitivity::Edge(edge) => unsafe {
                let interrupt_type = core::ptr::read_volatile(regs.interrupt_type)
                    .with_bit(offset, zynq7000::gpio::InterruptType::Edge);
                core::ptr::write_volatile(regs.interrupt_type, interrupt_type);
                let polarity =
                    core::ptr::read_volatile(regs.polarity).with_bit(offset, edge.polarity_bit());
                core::ptr::write_volatile(regs.polarity, polarity);
                let mut any_edge = core::ptr::read_volatile(regs.any_edge);
                if edge == super::asynch::EdgeSensitivity::Any {
                    any_edge |= 1 << offset;
                } else {
                    any_edge &= !(1 << offset);
                }
                core::ptr::write_volatile(regs.any_edge, any_edge);
            },
        });
        let enable_and_clear = 1 << offset;
        // SAFETY: The pointers come from the GPIO register block. Both registers are write-1, so
        // other pins are not affected.
        unsafe {
            // Clear the interrupt.
            core::ptr::write_volatile(regs.status, enable_and_clear);
            // Enable the interrupt.
            core::ptr::write_volatile(regs.enable, enable_and_clear);
        }
    }

    /// Disable the interrupt for the pin.
    pub fn disable_interrupt(&mut self) {
        let (offset, disable_reg_ptr) = self.get_interrupt_disable_regs();
        let disable = 1 << offset;
        // SAFETY: The pointer comes from the GPIO register block. The register is write-1, so
        // other pins are not affected.
        unsafe { core::ptr::write_volatile(disable_reg_ptr, disable) };
    }

    /// Convert the pin into an output pin with open drain emulation.
    ///
    /// This works by only enabling the output driver when the pin is driven low and letting
    /// the pin float when it is driven high. A pin pull-up is used for MIO pins as well which
    /// pulls the pin to a defined state if it is not driven. This allows something like 1-wire bus
    /// operation because other devices can pull the pin low as well.
    ///
    /// For EMIO pins, the pull-up and the IO buffer necessary for open-drain usage must be
    /// provided by the FPGA design.
    pub fn configure_as_output_open_drain(
        &mut self,
        init_level: PinState,
        with_internal_pullup: bool,
    ) {
        let (offset, dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        if self.offset.is_mio() {
            // Tri-state bit must be 0 for the output driver to work. Enable the pullup pin.
            self.reconfigure_slcr_mio_cfg(
                false,
                Some(with_internal_pullup),
                Some(MuxConfig::new_for_gpio()),
            );
        }
        if init_level == PinState::Low {
            self.write_state(init_level);
        }
        // The registers are shared by all pins of a bank.
        critical_section::with(|_| {
            // SAFETY: The pointers come from the GPIO register block.
            unsafe {
                let curr_dirm = core::ptr::read_volatile(dirm);
                core::ptr::write_volatile(dirm, curr_dirm | (1 << offset));
                // Disable the output driver depending on initial level.
                let mut curr_outen = core::ptr::read_volatile(outen);
                if init_level == PinState::High {
                    curr_outen &= !(1 << offset);
                } else {
                    curr_outen |= 1 << offset;
                }
                core::ptr::write_volatile(outen, curr_outen);
            }
        });
    }

    /// Convert the pin into a floating input pin.
    pub fn configure_as_input_floating(&mut self) -> Result<(), PinIsOutputOnly> {
        if self.offset.is_mio() {
            let offset_raw = self.offset.offset();
            if offset_raw == 7 || offset_raw == 8 {
                return Err(PinIsOutputOnly);
            }
            self.reconfigure_slcr_mio_cfg(true, Some(false), Some(MuxConfig::new_for_gpio()));
        }
        self.configure_input_pin();
        Ok(())
    }

    /// Convert the pin into an input pin with a pull up.
    pub fn configure_as_input_with_pull_up(&mut self) -> Result<(), PinIsOutputOnly> {
        if self.offset.is_mio() {
            let offset_raw = self.offset.offset();
            if offset_raw == 7 || offset_raw == 8 {
                return Err(PinIsOutputOnly);
            }
            self.reconfigure_slcr_mio_cfg(true, Some(true), Some(MuxConfig::new_for_gpio()));
        }
        self.configure_input_pin();
        Ok(())
    }

    /// Convert the pin into an IO peripheral pin.
    ///
    /// `pullup` is `Some(true)` to enable the pin's internal pull-up, `Some(false)` to disable
    /// it, or `None` to leave the current pull-up configuration untouched. MIO pins only support
    /// a pull-up, there is no pull-down option, so `Some(false)` leaves the pin floating rather
    /// than pulling it low.
    pub fn configure_as_io_periph_pin(&mut self, mux_conf: MuxConfig, pullup: Option<bool>) {
        self.reconfigure_slcr_mio_cfg(false, pullup, Some(mux_conf));
    }

    /// Set the MIO pin configuration.
    pub fn set_mio_pin_config(&mut self, config: zynq7000::slcr::mio::Config) {
        let raw_offset = self.offset.offset();
        // SAFETY: We only modify the MIO config of the pin.
        let mut slcr_wrapper = unsafe { Slcr::steal() };
        slcr_wrapper.modify(|mut_slcr| mut_slcr.write_mio_pins(raw_offset, config).unwrap());
    }

    /// Set the MIO pin configuration with an unlocked SLCR.
    pub fn set_mio_pin_config_with_unlocked_slcr(
        &mut self,
        slcr: &mut zynq7000::slcr::MmioRegisters<'static>,
        config: zynq7000::slcr::mio::Config,
    ) {
        let raw_offset = self.offset.offset();
        slcr.write_mio_pins(raw_offset, config).unwrap();
    }

    /// Input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        let (offset, in_reg) = self.get_data_in_reg_and_local_offset();
        // SAFETY: The pointer comes from the GPIO register block.
        let in_val = unsafe { core::ptr::read_volatile(in_reg) };
        ((in_val >> offset) & 0b1) == 0
    }

    /// Input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    /// Pin is driven low.
    #[inline]
    pub fn is_set_low(&self) -> bool {
        let (offset, out_reg) = self.get_data_out_reg_and_local_offset();
        // SAFETY: The pointer comes from the GPIO register block.
        let out_val = unsafe { core::ptr::read_volatile(out_reg) };
        ((out_val >> offset) & 0b1) == 0
    }

    /// Pin is driven high.
    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Enable the output driver.
    #[inline]
    pub fn enable_output_driver(&mut self) {
        let (offset, _dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        // SAFETY: The pointer comes from the GPIO register block.
        unsafe {
            let mut outen_reg = core::ptr::read_volatile(outen);
            outen_reg |= 1 << offset;
            core::ptr::write_volatile(outen, outen_reg);
        }
    }

    /// Disable the output driver.
    #[inline]
    pub fn disable_output_driver(&mut self) {
        let (offset, _dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        // SAFETY: The pointer comes from the GPIO register block.
        unsafe {
            let mut outen_reg = core::ptr::read_volatile(outen);
            outen_reg &= !(1 << offset);
            core::ptr::write_volatile(outen, outen_reg);
        }
    }

    /// Drive the pin low.
    #[inline]
    pub fn set_low(&mut self) {
        self.write_state(PinState::Low)
    }

    /// Drive the pin high.
    #[inline]
    pub fn set_high(&mut self) {
        self.write_state(PinState::High)
    }

    #[inline]
    fn write_state(&mut self, level: PinState) {
        let (offset_in_reg, masked_out_ptr) = self.get_masked_out_reg_and_local_offset();
        // SAFETY: The pointer comes from the GPIO register block. The masked write only changes
        // this pin.
        unsafe {
            core::ptr::write_volatile(
                masked_out_ptr,
                MaskedOutput::builder()
                    .with_mask(!(1 << offset_in_reg))
                    .with_output((level as u16) << offset_in_reg)
                    .build(),
            );
        }
    }

    fn reconfigure_slcr_mio_cfg(
        &mut self,
        tristate: bool,
        pullup: Option<bool>,
        mux_conf: Option<MuxConfig>,
    ) {
        let raw_offset = self.offset.offset();
        // SAFETY: We only modify the MIO config of the pin.
        let mut slcr_wrapper = unsafe { Slcr::steal() };
        // We read first, because writing also required unlocking the SLCR.
        // This allows the user to configure the SLCR themselves to avoid unnecessary
        // re-configuration which might also be potentially unsafe at run-time.
        let mio_cfg = slcr_wrapper.regs().read_mio_pins(raw_offset).unwrap();
        if (pullup.is_some() && mio_cfg.pullup() != pullup.unwrap())
            || (mux_conf.is_some() && MuxConfig::from(mio_cfg) != mux_conf.unwrap())
            || tristate != mio_cfg.tri_enable()
        {
            slcr_wrapper.modify(|mut_slcr| {
                mut_slcr
                    .modify_mio_pins(raw_offset, |mut val| {
                        if let Some(pullup) = pullup {
                            val.set_pullup(pullup);
                        }
                        if let Some(mux_conf) = mux_conf {
                            val.set_l0_sel(mux_conf.l0_sel());
                            val.set_l1_sel(mux_conf.l1_sel());
                            val.set_l2_sel(mux_conf.l2_sel());
                            val.set_l3_sel(mux_conf.l3_sel());
                        }
                        val.set_tri_enable(tristate);
                        val
                    })
                    .unwrap();
            });
        }
    }

    fn configure_input_pin(&mut self) {
        let (offset, dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        // The registers are shared by all pins of a bank.
        critical_section::with(|_| {
            // SAFETY: The pointers come from the GPIO register block.
            unsafe {
                let curr_dirm = core::ptr::read_volatile(dirm);
                core::ptr::write_volatile(dirm, curr_dirm & !(1 << offset));
                let curr_outen = core::ptr::read_volatile(outen);
                core::ptr::write_volatile(outen, curr_outen & !(1 << offset));
            }
        });
    }

    #[inline(always)]
    fn get_data_in_reg_and_local_offset(&self) -> (usize, *mut u32) {
        match self.offset {
            PinOffset::Mio(offset) => match offset {
                0..=31 => (offset, self.regs.pointer_to_in_0()),
                32..=53 => (offset - 32, self.regs.pointer_to_in_1()),
                _ => panic!("invalid MIO pin offset"),
            },
            PinOffset::Emio(offset) => match offset {
                0..=31 => (offset, self.regs.pointer_to_in_2()),
                32..=63 => (offset - 32, self.regs.pointer_to_in_3()),
                _ => panic!("invalid EMIO pin offset"),
            },
        }
    }

    #[inline(always)]
    fn get_data_out_reg_and_local_offset(&self) -> (usize, *mut u32) {
        match self.offset {
            PinOffset::Mio(offset) => match offset {
                0..=31 => (offset, self.regs.pointer_to_out_0()),
                32..=53 => (offset - 32, self.regs.pointer_to_out_1()),
                _ => panic!("invalid MIO pin offset"),
            },
            PinOffset::Emio(offset) => match offset {
                0..=31 => (offset, self.regs.pointer_to_out_2()),
                32..=63 => (offset - 32, self.regs.pointer_to_out_3()),
                _ => panic!("invalid EMIO pin offset"),
            },
        }
    }

    #[inline(always)]
    fn get_dirm_outen_regs_and_local_offset(&self) -> (usize, *mut u32, *mut u32) {
        match self.offset {
            PinOffset::Mio(offset) => match offset {
                0..=31 => (
                    offset,
                    self.regs.bank_0_shared().pointer_to_dirm(),
                    self.regs.bank_0_shared().pointer_to_out_en(),
                ),
                32..=53 => (
                    offset - 32,
                    self.regs.bank_1_shared().pointer_to_dirm(),
                    self.regs.bank_1_shared().pointer_to_out_en(),
                ),
                _ => panic!("invalid MIO pin offset"),
            },
            PinOffset::Emio(offset) => match offset {
                0..=31 => (
                    offset,
                    self.regs.bank_2_shared().pointer_to_dirm(),
                    self.regs.bank_2_shared().pointer_to_out_en(),
                ),
                32..=63 => (
                    offset - 32,
                    self.regs.bank_3_shared().pointer_to_dirm(),
                    self.regs.bank_3_shared().pointer_to_out_en(),
                ),
                _ => panic!("invalid EMIO pin offset"),
            },
        }
    }

    #[inline(always)]
    fn get_interrupt_regs(&self) -> InterruptRegs {
        match self.offset {
            PinOffset::Mio(offset) => match offset {
                0..=31 => InterruptRegs::new(&self.regs.bank_0_shared(), offset),
                32..=53 => InterruptRegs::new(&self.regs.bank_1_shared(), offset - 32),
                _ => panic!("invalid MIO pin offset"),
            },
            PinOffset::Emio(offset) => match offset {
                0..=31 => InterruptRegs::new(&self.regs.bank_2_shared(), offset),
                32..=63 => InterruptRegs::new(&self.regs.bank_3_shared(), offset - 32),
                _ => panic!("invalid EMIO pin offset"),
            },
        }
    }

    #[inline(always)]
    fn get_interrupt_disable_regs(&self) -> (usize, *mut u32) {
        match self.offset {
            PinOffset::Mio(offset) => match offset {
                0..=31 => (
                    offset,
                    self.regs.bank_0_shared().pointer_to_disable_interrupt(),
                ),
                32..=53 => (
                    offset - 32,
                    self.regs.bank_1_shared().pointer_to_disable_interrupt(),
                ),
                _ => panic!("invalid MIO pin offset"),
            },
            PinOffset::Emio(offset) => match offset {
                0..=31 => (
                    offset,
                    self.regs.bank_2_shared().pointer_to_disable_interrupt(),
                ),
                32..=63 => (
                    offset - 32,
                    self.regs.bank_3_shared().pointer_to_disable_interrupt(),
                ),
                _ => panic!("invalid EMIO pin offset"),
            },
        }
    }

    #[inline(always)]
    fn get_masked_out_reg_and_local_offset(&mut self) -> (usize, *mut MaskedOutput) {
        match self.offset {
            PinOffset::Mio(offset) => match offset {
                0..=15 => (offset, self.regs.pointer_to_masked_out_0_lsw()),
                16..=31 => (offset - 16, self.regs.pointer_to_masked_out_0_msw()),
                32..=47 => (offset - 32, self.regs.pointer_to_masked_out_1_lsw()),
                48..=53 => (offset - 48, self.regs.pointer_to_masked_out_1_msw()),
                _ => panic!("invalid MIO pin offset"),
            },
            PinOffset::Emio(offset) => match offset {
                0..=15 => (offset, self.regs.pointer_to_masked_out_2_lsw()),
                16..=31 => (offset - 16, self.regs.pointer_to_masked_out_2_msw()),
                32..=47 => (offset - 32, self.regs.pointer_to_masked_out_3_lsw()),
                48..=63 => (offset - 48, self.regs.pointer_to_masked_out_3_msw()),
                _ => panic!("invalid EMIO pin offset"),
            },
        }
    }
}
