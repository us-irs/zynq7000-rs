use zynq7000::spi::{Config, FifoWrite, MmioRegisters};

use crate::{
    enable_amba_peripheral_clock,
    spi::{SpiId, SpiLowLevel},
    spi_mode_const_to_cpol_cpha,
};

pub struct SpiSlave(pub SpiLowLevel);

pub struct SlaveConfig {
    pub mode: embedded_hal::spi::Mode,
    pub enable_modefail: bool,
}

impl SpiSlave {
    /// Creates a very simple SPI slave driver.
    ///
    /// The SPI slave will not start in the enabled state to allow pre-loading the FIFO.
    pub fn new(id: SpiId, regs: MmioRegisters<'static>, config: SlaveConfig) -> Self {
        let periph_sel = match id {
            SpiId::Spi0 => crate::PeriphSelect::Spi0,
            SpiId::Spi1 => crate::PeriphSelect::Spi1,
        };
        let mut ll = SpiLowLevel { id, regs };
        ll.enable_ref_clock();
        enable_amba_peripheral_clock(periph_sel);

        let (cpol, cpha) = spi_mode_const_to_cpol_cpha(config.mode);
        ll.write_config(
            Config::ZERO
                .with_modefail_gen_en(config.enable_modefail)
                .with_cpha(cpha)
                .with_cpol(cpol),
        );
        Self(ll)
    }

    #[inline]
    pub const fn id(&self) -> SpiId {
        self.0.id
    }

    #[inline]
    pub const fn interrupt_id(&self) -> crate::Interrupt {
        self.id().interrupt_id()
    }

    #[inline]
    pub fn enable_interrupts(&mut self, mode_error: bool) {
        self.0.enable_interrupts_slave_mode(mode_error);
    }

    /// Enable the SPI slave. This will allow it to respond to transfers initiated by the master.
    ///
    /// Please refer to the word detection in the TRM p.568 for more details.
    #[inline]
    pub fn enable(&mut self) {
        self.0.enable();
    }

    /// This register is used for IDLE state detection on the SCLK line.
    ///
    /// Please refer to the word detection in the TRM p.568 for more details how this is useful.
    #[inline]
    pub fn write_slave_idle_counter(&mut self, count: u8) {
        self.0.write_sicr(count as u32);
    }

    /// Write data to the TX FIFO, which allows pre-loading data which will be sent via the
    /// MISO line when the master initiates a transfer.
    #[inline]
    pub fn write_tx_data(&mut self, data: u8) {
        self.0.write_tx_data(FifoWrite::new(data));
    }
}
