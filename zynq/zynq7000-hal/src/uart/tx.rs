use core::convert::Infallible;

use zynq7000::uart::{Fifo, InterruptControl, InterruptStatus, MmioUart};

use super::UartId;

pub struct Tx {
    pub(crate) regs: MmioUart<'static>,
    pub(crate) idx: UartId,
}

impl Tx {
    /// Steal the TX side of the UART for a given UART index.
    ///
    /// # Safety
    ///
    /// Circumvents safety guarantees provided by the compiler.
    #[inline]
    pub const unsafe fn steal(idx: UartId) -> Self {
        Tx {
            regs: unsafe { idx.regs() },
            idx,
        }
    }

    #[inline]
    pub const fn uart_idx(&self) -> UartId {
        self.idx
    }

    #[inline]
    pub const fn regs(&mut self) -> &mut MmioUart<'static> {
        &mut self.regs
    }

    #[inline]
    pub fn write_fifo(&mut self, word: u8) -> nb::Result<(), Infallible> {
        if self.regs.read_sr().tx_full() {
            return Err(nb::Error::WouldBlock);
        }
        self.write_fifo_unchecked(word);
        Ok(())
    }

    /// Enables TX side of the UART.
    #[inline]
    pub fn enable(&mut self, with_reset: bool) {
        if with_reset {
            self.soft_reset();
        }
        self.regs.modify_cr(|mut val| {
            val.set_tx_en(true);
            val.set_tx_dis(false);
            val
        });
    }

    /// Disables TX side of the UART.
    #[inline]
    pub fn disable(&mut self) {
        self.regs.modify_cr(|mut val| {
            val.set_tx_en(false);
            val.set_tx_dis(true);
            val
        });
    }

    #[inline]
    pub fn soft_reset(&mut self) {
        self.regs.modify_cr(|mut val| {
            val.set_tx_rst(true);
            val
        });
        loop {
            if !self.regs.read_cr().tx_rst() {
                break;
            }
        }
    }

    pub fn flush(&mut self) {
        while !self.regs.read_sr().tx_empty() {}
    }

    #[inline]
    pub fn write_fifo_unchecked(&mut self, word: u8) {
        self.regs.write_fifo(Fifo::new_with_raw_value(word as u32));
    }

    /// Enables interrupts relevant for the TX side of the UART except the TX trigger interrupt.
    #[inline]
    pub fn enable_interrupts(&mut self) {
        self.regs.write_ier(
            InterruptControl::builder()
                .with_tx_over(true)
                .with_tx_near_full(true)
                .with_tx_trig(false)
                .with_rx_dms(false)
                .with_rx_timeout(false)
                .with_rx_parity(false)
                .with_rx_framing(false)
                .with_rx_over(false)
                .with_tx_full(true)
                .with_tx_empty(true)
                .with_rx_full(false)
                .with_rx_empty(false)
                .with_rx_trg(false)
                .build(),
        );
    }

    /// Disable interrupts relevant for the TX side of the UART except the TX trigger interrupt.
    #[inline]
    pub fn disable_interrupts(&mut self) {
        self.regs.write_idr(
            InterruptControl::builder()
                .with_tx_over(true)
                .with_tx_near_full(true)
                .with_tx_trig(false)
                .with_rx_dms(false)
                .with_rx_timeout(false)
                .with_rx_parity(false)
                .with_rx_framing(false)
                .with_rx_over(false)
                .with_tx_full(true)
                .with_tx_empty(true)
                .with_rx_full(false)
                .with_rx_empty(false)
                .with_rx_trg(false)
                .build(),
        );
    }

    /// Clears interrupts relevant for the TX side of the UART except the TX trigger interrupt.
    #[inline]
    pub fn clear_interrupts(&mut self) {
        self.regs.write_isr(
            InterruptStatus::builder()
                .with_tx_over(true)
                .with_tx_near_full(true)
                .with_tx_trig(false)
                .with_rx_dms(false)
                .with_rx_timeout(false)
                .with_rx_parity(false)
                .with_rx_framing(false)
                .with_rx_over(false)
                .with_tx_full(true)
                .with_tx_empty(true)
                .with_rx_full(false)
                .with_rx_empty(false)
                .with_rx_trg(false)
                .build(),
        );
    }
}

impl embedded_hal_nb::serial::ErrorType for Tx {
    type Error = Infallible;
}

impl embedded_hal_nb::serial::Write for Tx {
    #[inline]
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_fifo(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if self.regs.read_sr().tx_empty() {
            return Ok(());
        }
        Err(nb::Error::WouldBlock)
    }
}

impl embedded_io::ErrorType for Tx {
    type Error = Infallible;
}

impl embedded_io::Write for Tx {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut written = 0;
        loop {
            if !self.regs.read_sr().tx_full() {
                break;
            }
        }
        for byte in buf.iter() {
            match self.write_fifo(*byte) {
                Ok(_) => written += 1,
                Err(nb::Error::WouldBlock) => return Ok(written),
            }
        }

        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush();
        Ok(())
    }
}
