//! Low-level GPIO access module.
use embedded_hal::digital::PinState;
use zynq7000::gpio::{Gpio, MaskedOutput, MmioGpio};

use crate::slcr::Slcr;

use super::{mio::MuxConf, PinIsOutputOnly};

#[derive(Debug, Clone, Copy)]
pub enum PinOffset {
    Mio(usize),
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

    pub fn is_mio(&self) -> bool {
        match self {
            PinOffset::Mio(_) => true,
            PinOffset::Emio(_) => false,
        }
    }
}

impl PinOffset {
    pub fn offset(&self) -> usize {
        match self {
            PinOffset::Mio(offset) => *offset,
            PinOffset::Emio(offset) => *offset,
        }
    }
}

pub struct LowLevelGpio {
    offset: PinOffset,
    regs: MmioGpio<'static>,
}

impl LowLevelGpio {
    pub fn new(offset: PinOffset) -> Self {
        Self {
            offset,
            regs: unsafe { Gpio::new_mmio_fixed() },
        }
    }

    pub fn offset(&self) -> PinOffset {
        self.offset
    }

    /// Convert the pin into an output pin.
    pub fn configure_as_output_push_pull(&mut self, init_level: PinState) {
        let (offset, dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        if self.offset.is_mio() {
            // Tri-state bit must be 0 for the output driver to work.
            self.reconfigure_slcr_mio_cfg(false, None, Some(MuxConf::new_for_gpio()));
        }
        let mut curr_dirm = unsafe { core::ptr::read_volatile(dirm) };
        curr_dirm |= 1 << offset;
        unsafe { core::ptr::write_volatile(dirm, curr_dirm) };
        let mut curr_outen = unsafe { core::ptr::read_volatile(outen) };
        curr_outen |= 1 << offset;
        unsafe { core::ptr::write_volatile(outen, curr_outen) };
        // Unwrap okay, just set mode.
        self.write_state(init_level);
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
                Some(MuxConf::new_for_gpio()),
            );
        }
        let mut curr_dirm = unsafe { core::ptr::read_volatile(dirm) };
        curr_dirm |= 1 << offset;
        unsafe { core::ptr::write_volatile(dirm, curr_dirm) };
        // Disable the output driver depending on initial level.
        let mut curr_outen = unsafe { core::ptr::read_volatile(outen) };
        if init_level == PinState::High {
            curr_outen &= !(1 << offset);
        } else {
            curr_outen |= 1 << offset;
            self.write_state(init_level);
        }
        unsafe { core::ptr::write_volatile(outen, curr_outen) };
    }

    /// Convert the pin into a floating input pin.
    pub fn configure_as_input_floating(&mut self) -> Result<(), PinIsOutputOnly> {
        if self.offset.is_mio() {
            let offset_raw = self.offset.offset();
            if offset_raw == 7 || offset_raw == 8 {
                return Err(PinIsOutputOnly);
            }
            self.reconfigure_slcr_mio_cfg(true, Some(false), Some(MuxConf::new_for_gpio()));
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
            self.reconfigure_slcr_mio_cfg(true, Some(true), Some(MuxConf::new_for_gpio()));
        }
        self.configure_input_pin();
        Ok(())
    }

    /// Convert the pin into an IO peripheral pin.
    pub fn configure_as_io_periph_pin(&mut self, mux_conf: MuxConf, pullup: Option<bool>) {
        self.reconfigure_slcr_mio_cfg(false, pullup, Some(mux_conf));
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        let (offset, in_reg) = self.get_data_in_reg_and_local_offset();
        let in_val = unsafe { core::ptr::read_volatile(in_reg) };
        ((in_val >> offset) & 0b1) == 0
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        let (offset, out_reg) = self.get_data_out_reg_and_local_offset();
        let out_val = unsafe { core::ptr::read_volatile(out_reg) };
        ((out_val >> offset) & 0b1) == 0
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    #[inline]
    pub fn enable_output_driver(&mut self) {
        let (offset, _dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        let mut outen_reg = unsafe { core::ptr::read_volatile(outen) };
        outen_reg |= 1 << offset;
        unsafe { core::ptr::write_volatile(outen, outen_reg) };
    }

    #[inline]
    pub fn disable_output_driver(&mut self) {
        let (offset, _dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        let mut outen_reg = unsafe { core::ptr::read_volatile(outen) };
        outen_reg &= !(1 << offset);
        unsafe { core::ptr::write_volatile(outen, outen_reg) };
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.write_state(PinState::Low)
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.write_state(PinState::High)
    }

    #[inline]
    fn write_state(&mut self, level: PinState) {
        let (offset_in_reg, masked_out_ptr) = self.get_masked_out_reg_and_local_offset();
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
        mux_conf: Option<MuxConf>,
    ) {
        let raw_offset = self.offset.offset();
        // Safety: We only modify the MIO config of the pin.
        let mut slcr_wrapper = unsafe { Slcr::steal() };
        // We read first, because writing also required unlocking the SLCR.
        // This allows the user to configure the SLCR themselves to avoid unnecessary
        // re-configuration which might also be potentially unsafe at run-time.
        let mio_cfg = slcr_wrapper.regs().read_mio_pins(raw_offset).unwrap();
        if (pullup.is_some() && mio_cfg.pullup() != pullup.unwrap())
            || (mux_conf.is_some() && MuxConf::from(mio_cfg) != mux_conf.unwrap())
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
        let mut curr_dirm = unsafe { core::ptr::read_volatile(dirm) };
        curr_dirm &= !(1 << offset);
        unsafe { core::ptr::write_volatile(dirm, curr_dirm) };
        let mut curr_outen = unsafe { core::ptr::read_volatile(outen) };
        curr_outen &= !(1 << offset);
        unsafe { core::ptr::write_volatile(outen, curr_outen) };
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
