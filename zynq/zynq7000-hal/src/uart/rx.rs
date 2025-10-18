use core::convert::Infallible;

use arbitrary_int::prelude::*;
use zynq7000::uart::{InterruptControl, InterruptStatus, MmioRegisters};

use super::FIFO_DEPTH;

pub struct Rx {
    pub(crate) regs: MmioRegisters<'static>,
}
// TODO: Remove once this is impelemnted for MmioUart
unsafe impl Send for Rx {}

#[derive(Debug, Default, Clone, Copy)]
pub struct RxErrors {
    framing: bool,
    overrun: bool,
    parity: bool,
}

impl RxErrors {
    #[inline]
    pub const fn framing(&self) -> bool {
        self.framing
    }
    #[inline]
    pub const fn overrun(&self) -> bool {
        self.overrun
    }
    #[inline]
    pub const fn parity(&self) -> bool {
        self.parity
    }
}

#[derive(Debug, Default)]
pub struct RxInterruptResult {
    read_bytes: usize,
    errors: Option<RxErrors>,
}

impl RxInterruptResult {
    pub fn read_bytes(&self) -> usize {
        self.read_bytes
    }

    pub fn errors(&self) -> Option<RxErrors> {
        self.errors
    }
}

impl Rx {
    #[inline]
    pub fn read_fifo(&mut self) -> nb::Result<u8, Infallible> {
        if self.regs.read_sr().rx_empty() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(self.regs.read_fifo().fifo())
    }

    #[inline(always)]
    pub fn read_fifo_unchecked(&mut self) -> u8 {
        self.regs.read_fifo().fifo()
    }

    /// Write the receiver timeout value.
    ///
    /// A value of 0 will disable the receiver timeout.
    /// Otherwise, the 10-bit counter used by the receiver timeout mechanism of the UART will
    /// load this value for the upper 8 bits on a reload. The counter is clocked by the UART
    /// bit clock, so this value times 4 is the number of UART clock ticks until a timeout occurs.
    #[inline]
    pub fn set_rx_timeout_value(&mut self, rto: u8) {
        self.regs.write_rx_tout(rto as u32);
    }

    #[inline]
    pub fn soft_reset(&mut self) {
        self.regs.modify_cr(|mut cr| {
            cr.set_rx_rst(true);
            cr
        });
        while self.regs.read_cr().rx_rst() {}
    }

    /// Helper function to start the interrupt driven reception of data.
    ///
    /// This function will perform a soft-reset, clear RX related interrupts and then enable
    /// all relevant interrupts for the RX side of the UART. These steps are recommended to have
    /// a glitch-free start of the interrupt driven reception.
    ///
    /// This should be called once at system start-up. After that, you only need to call
    /// [Self::on_interrupt] in the interrupt handler for the UART peripheral.
    pub fn start_interrupt_driven_reception(&mut self) {
        self.soft_reset();
        self.clear_interrupts();
        self.enable_interrupts();
    }

    /// Enables all interrupts relevant for the RX side of the UART.
    ///
    /// It is recommended to also clear all interrupts immediately after enabling them.
    #[inline]
    pub fn enable_interrupts(&mut self) {
        self.regs.write_ier(
            InterruptControl::builder()
                .with_tx_over(false)
                .with_tx_near_full(false)
                .with_tx_trig(false)
                .with_rx_dms(false)
                .with_rx_timeout(true)
                .with_rx_parity(true)
                .with_rx_framing(true)
                .with_rx_over(true)
                .with_tx_full(false)
                .with_tx_empty(false)
                .with_rx_full(true)
                .with_rx_empty(false)
                .with_rx_trg(true)
                .build(),
        );
    }

    pub fn on_interrupt(
        &mut self,
        buf: &mut [u8; FIFO_DEPTH],
        reset_rx_timeout: bool,
    ) -> RxInterruptResult {
        let mut result = RxInterruptResult::default();
        let imr = self.regs.read_imr();
        if !imr.rx_full()
            && !imr.rx_trg()
            && !imr.rx_parity()
            && !imr.rx_framing()
            && !imr.rx_over()
            && !imr.rx_timeout()
        {
            return result;
        }
        let isr = self.regs.read_isr();
        if isr.rx_full() {
            // Read all bytes in the full RX fifo.
            for byte in buf.iter_mut() {
                *byte = self.read_fifo_unchecked();
            }
            result.read_bytes = FIFO_DEPTH;
        } else if isr.rx_trg() {
            // It is guaranteed that we can read the FIFO level amount of data
            let fifo_trigger = self.regs.read_rx_fifo_trigger().trig().as_usize();
            (0..fifo_trigger).for_each(|i| {
                buf[i] = self.read_fifo_unchecked();
            });
            result.read_bytes = fifo_trigger;
        }
        // Read everything else that is available, as long as there is space left.
        while result.read_bytes < buf.len() {
            if let Ok(byte) = self.read_fifo() {
                buf[result.read_bytes] = byte;
                result.read_bytes += 1;
            } else {
                break;
            }
        }
        // Handle error events.
        if isr.rx_parity() || isr.rx_framing() || isr.rx_over() {
            result.errors = Some(RxErrors {
                framing: isr.rx_framing(),
                overrun: isr.rx_over(),
                parity: isr.rx_parity(),
            });
        }
        // Handle timeout event.
        if isr.rx_timeout() && reset_rx_timeout {
            self.regs.modify_cr(|mut cr| {
                cr.set_rstto(true);
                cr
            });
        }
        self.clear_interrupts();
        result
    }

    // This clears all RX related interrupts.
    #[inline]
    pub fn clear_interrupts(&mut self) {
        self.regs.write_isr(
            InterruptStatus::builder()
                .with_tx_over(false)
                .with_tx_near_full(false)
                .with_tx_trig(false)
                .with_rx_dms(true)
                .with_rx_timeout(true)
                .with_rx_parity(true)
                .with_rx_framing(true)
                .with_rx_over(true)
                .with_tx_full(false)
                .with_tx_empty(false)
                .with_rx_full(true)
                .with_rx_empty(true)
                .with_rx_trg(true)
                .build(),
        );
    }
}

impl embedded_hal_nb::serial::ErrorType for Rx {
    type Error = Infallible;
}

impl embedded_hal_nb::serial::Read for Rx {
    /// Read one byte from the FIFO.
    ///
    /// This operation is infallible because pulling an available byte from the FIFO
    /// always succeeds. If you want to be informed about RX errors, you should look at the
    /// non-blocking API using interrupts, which also tracks the RX error bits.
    #[inline]
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_fifo()
    }
}

impl embedded_io::ErrorType for Rx {
    type Error = Infallible;
}

impl embedded_io::Read for Rx {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut read = 0;
        loop {
            if !self.regs.read_sr().rx_empty() {
                break;
            }
        }
        for byte in buf.iter_mut() {
            match <Self as embedded_hal_nb::serial::Read<u8>>::read(self) {
                Ok(w) => {
                    *byte = w;
                    read += 1;
                }
                Err(nb::Error::WouldBlock) => break,
            }
        }

        Ok(read)
    }
}
