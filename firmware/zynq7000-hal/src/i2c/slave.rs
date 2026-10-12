//! Low-level I2C slave driver.
//!
//! This only sets up the peripheral and gives access to the FIFO, the status and the interrupts.
//! The protocol on top, for example a register pointer, is up to the user and usually lives in an
//! interrupt handler.
use arbitrary_int::{u2, u6, u7, u10};
use zynq7000::i2c::{
    Address, Control, Direction, Fifo, InterruptControl, InterruptStatus, MmioRegisters, Mode,
    Status,
};

use super::{
    ClockConfig, ConstructionError, I2cId, InvalidPsI2cError, PsI2c, SckPin, SdaPin,
    configure_mio_pins,
};
use crate::enable_amba_peripheral_clock;

/// I2C slave configuration.
pub struct Config {
    /// 7 bit slave address.
    pub address: u7,
    /// Must be at least as fast as the fastest SCL frequency on the bus, see TRM p.643.
    pub clk_cfg: ClockConfig,
}

/// I2C slave driver.
pub struct I2cSlave {
    id: I2cId,
    regs: MmioRegisters<'static>,
}

impl I2cSlave {
    /// Create a driver which uses MIO pins.
    pub fn new_with_mio<Sck: SckPin, Sda: SdaPin>(
        i2c: impl PsI2c,
        config: Config,
        i2c_pins: (Sck, Sda),
    ) -> Result<Self, ConstructionError> {
        let id = configure_mio_pins(&i2c, i2c_pins)?;
        Ok(Self::new_generic(id, i2c.reg_block(), config))
    }

    /// Create a driver which uses EMIO pins.
    pub fn new_with_emio(i2c: impl PsI2c, config: Config) -> Result<Self, InvalidPsI2cError> {
        let id = i2c.id().ok_or(InvalidPsI2cError)?;
        Ok(Self::new_generic(id, i2c.reg_block(), config))
    }

    /// Create a driver without configuring any pins.
    pub fn new_generic(id: I2cId, mut regs: MmioRegisters<'static>, config: Config) -> Self {
        let periph_sel = match id {
            I2cId::I2c0 => crate::PeriphSelect::I2c0,
            I2cId::I2c1 => crate::PeriphSelect::I2c1,
        };
        enable_amba_peripheral_clock(periph_sel);
        regs.write_control(
            Control::builder()
                .with_div_a(u2::new(config.clk_cfg.div_a()))
                .with_div_b(u6::new(config.clk_cfg.div_b()))
                .with_clear_fifo(true)
                .with_slv_mon(false)
                .with_hold_bus(false)
                .with_acken(true)
                .with_addressing(true)
                .with_mode(Mode::Slave)
                .with_dir(Direction::Receiver)
                .build(),
        );
        // The slave responds as soon as the address is written.
        regs.write_addr(Address::new_with_raw_value(0).with_addr(u10::from(config.address)));
        Self { id, regs }
    }

    /// Steals the slave, for example to use it in an interrupt handler.
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees by the HAL. The peripheral must already be
    /// configured as a slave.
    #[inline]
    pub unsafe fn steal(id: I2cId) -> Self {
        Self {
            id,
            // SAFETY: Upheld by the caller.
            regs: unsafe { id.steal_regs() },
        }
    }

    /// I2C ID.
    #[inline]
    pub const fn id(&self) -> I2cId {
        self.id
    }

    /// GIC interrupt of the peripheral.
    #[inline]
    pub const fn interrupt_id(&self) -> crate::Interrupt {
        self.id.interrupt_id()
    }

    /// Enable the given interrupts.
    #[inline]
    pub fn enable_interrupts(&mut self, interrupts: InterruptControl) {
        self.regs.write_interrupt_enable(interrupts);
    }

    /// Disable the given interrupts.
    #[inline]
    pub fn disable_interrupts(&mut self, interrupts: InterruptControl) {
        self.regs.write_interrupt_disable(interrupts);
    }

    /// Read the interrupt status.
    #[inline]
    pub fn read_interrupt_status(&self) -> InterruptStatus {
        self.regs.read_interrupt_status()
    }

    /// Clears all interrupts which are set in `status`.
    #[inline]
    pub fn clear_interrupts(&mut self, status: InterruptStatus) {
        self.regs.write_interrupt_status(status);
    }

    /// [Status::slave_rw] shows the direction requested by the master.
    #[inline]
    pub fn read_status(&self) -> Status {
        self.regs.read_status()
    }

    /// In slave receiver mode, the number of received bytes in the FIFO.
    #[inline]
    pub fn read_transfer_size(&self) -> u8 {
        self.regs.read_transfer_size().size()
    }

    /// Read one byte from the RX FIFO.
    #[inline]
    pub fn read_data(&mut self) -> u8 {
        self.regs.read_data().data()
    }

    /// Write one byte to the TX FIFO.
    #[inline]
    pub fn write_data(&mut self, data: u8) {
        self.regs.write_data(Fifo::new_with_raw_value(data as u32));
    }

    /// Clear the FIFO.
    #[inline]
    pub fn clear_fifo(&mut self) {
        self.regs.modify_control(|cr| cr.with_clear_fifo(true));
    }

    /// With HOLD set, the slave keeps SCL low while the receive FIFO is full, instead of
    /// overflowing. See TRM p.645.
    #[inline]
    pub fn set_hold(&mut self, hold: bool) {
        self.regs.modify_control(|cr| cr.with_hold_bus(hold));
    }

    /// Direct register access for everything not covered by this driver.
    #[inline]
    pub fn regs(&mut self) -> &mut MmioRegisters<'static> {
        &mut self.regs
    }
}
