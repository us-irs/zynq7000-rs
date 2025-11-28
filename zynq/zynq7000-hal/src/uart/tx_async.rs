//! Asynchronous UART transmitter (TX) implementation.
use core::{cell::RefCell, convert::Infallible, sync::atomic::AtomicBool};

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use raw_slice::RawBufSlice;

use crate::uart::{FIFO_DEPTH, Tx, UartId};

static UART_TX_WAKERS: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];
static TX_CONTEXTS: [Mutex<RefCell<TxContext>>; 2] =
    [const { Mutex::new(RefCell::new(TxContext::new())) }; 2];
// Completion flag. Kept outside of the context structure as an atomic to avoid
// critical section.
static TX_DONE: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

/// This is a generic interrupt handler to handle asynchronous UART TX operations for a given
/// UART peripheral.
///
/// The user has to call this once in the interrupt handler responsible for the TX interrupts on
/// the given UART bank.
pub fn on_interrupt_tx(peripheral: UartId) {
    let mut tx_with_irq = unsafe { Tx::steal(peripheral) };
    let idx = peripheral as usize;
    let imr = tx_with_irq.regs().read_imr();
    // IRQ is not related to TX.
    if !imr.tx_over() && !imr.tx_near_full() && !imr.tx_full() && !imr.tx_empty() && !imr.tx_full()
    {
        return;
    }

    let isr = tx_with_irq.regs().read_isr();
    let unexpected_overrun = isr.tx_over();
    let mut context = critical_section::with(|cs| {
        let context_ref = TX_CONTEXTS[idx].borrow(cs);
        *context_ref.borrow()
    });
    // No transfer active.
    if context.slice.is_null() {
        return;
    }
    let slice_len = context.slice.len().unwrap();
    context.tx_overrun = unexpected_overrun;
    if (context.progress >= slice_len && isr.tx_empty()) || slice_len == 0 {
        // Write back updated context structure.
        critical_section::with(|cs| {
            let context_ref = TX_CONTEXTS[idx].borrow(cs);
            *context_ref.borrow_mut() = context;
        });
        // Transfer is done.
        TX_DONE[idx].store(true, core::sync::atomic::Ordering::Relaxed);
        tx_with_irq.disable_interrupts();
        tx_with_irq.clear_interrupts();
        UART_TX_WAKERS[idx].wake();
        return;
    }
    // Safety: We documented that the user provided slice must outlive the future, so we convert
    // the raw pointer back to the slice here.
    let slice = unsafe { context.slice.get() }.expect("slice is invalid");
    while context.progress < slice_len {
        if tx_with_irq.regs().read_sr().tx_full() {
            break;
        }
        // Safety: TX structure is owned by the future which does not write into the the data
        // register, so we can assume we are the only one writing to the data register.
        tx_with_irq.write_fifo_unchecked(slice[context.progress]);
        context.progress += 1;
    }

    // Write back updated context structure.
    critical_section::with(|cs| {
        let context_ref = TX_CONTEXTS[idx].borrow(cs);
        *context_ref.borrow_mut() = context;
    });
    // Clear interrupts.
    tx_with_irq.clear_interrupts();
}

#[derive(Debug, Copy, Clone)]
struct TxContext {
    progress: usize,
    tx_overrun: bool,
    slice: RawBufSlice,
}

#[allow(clippy::new_without_default)]
impl TxContext {
    pub const fn new() -> Self {
        Self {
            progress: 0,
            tx_overrun: false,
            slice: RawBufSlice::new_nulled(),
        }
    }
}

/// Transmission future for UART TX.
pub struct TxFuture {
    id: UartId,
}

impl TxFuture {
    /// # Safety
    ///
    /// This function stores the raw pointer of the passed data slice. The user MUST ensure
    /// that the slice outlives the data structure.
    pub unsafe fn new(tx_with_irq: &mut Tx, data: &[u8]) -> Self {
        let idx = tx_with_irq.uart_idx() as usize;
        TX_DONE[idx].store(false, core::sync::atomic::Ordering::Relaxed);
        tx_with_irq.disable_interrupts();
        tx_with_irq.disable();

        let init_fill_count = core::cmp::min(data.len(), FIFO_DEPTH);
        critical_section::with(|cs| {
            let context_ref = TX_CONTEXTS[idx].borrow(cs);
            let mut context = context_ref.borrow_mut();
            unsafe {
                context.slice.set(data);
            }
            context.progress = init_fill_count; // We fill the FIFO.
        });
        tx_with_irq.enable(true);
        for data in data.iter().take(init_fill_count) {
            tx_with_irq.write_fifo_unchecked(*data);
        }
        tx_with_irq.enable_interrupts();

        Self {
            id: tx_with_irq.uart_idx(),
        }
    }
}

impl Future for TxFuture {
    type Output = usize;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        UART_TX_WAKERS[self.id as usize].register(cx.waker());
        if TX_DONE[self.id as usize].swap(false, core::sync::atomic::Ordering::Relaxed) {
            let progress = critical_section::with(|cs| {
                let mut ctx = TX_CONTEXTS[self.id as usize].borrow(cs).borrow_mut();
                ctx.slice.set_null();
                ctx.progress
            });
            return core::task::Poll::Ready(progress);
        }
        core::task::Poll::Pending
    }
}

impl Drop for TxFuture {
    fn drop(&mut self) {
        let mut tx = unsafe { Tx::steal(self.id) };
        tx.disable_interrupts();
    }
}

/// Asynchronous UART transmitter (TX) driver.
pub struct TxAsync {
    tx: Tx,
}

impl TxAsync {
    /// Constructor.
    pub fn new(tx: Tx) -> Self {
        Self { tx }
    }

    /// Write a buffer asynchronously.
    ///
    /// This implementation is not side effect free, and a started future might have already
    /// written part of the passed buffer.
    pub async fn write(&mut self, buf: &[u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }
        let fut = unsafe { TxFuture::new(&mut self.tx, buf) };
        fut.await
    }

    /// Release the underlying blocking TX driver.
    pub fn release(self) -> Tx {
        self.tx
    }
}

impl embedded_io::ErrorType for TxAsync {
    type Error = Infallible;
}

impl embedded_io_async::Write for TxAsync {
    /// Write a buffer asynchronously.
    ///
    /// This implementation is not side effect free, and a started future might have already
    /// written part of the passed buffer.
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(self.write(buf).await)
    }

    /// This implementation does not do anything.
    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
