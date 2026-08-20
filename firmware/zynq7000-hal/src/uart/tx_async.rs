//! Asynchronous UART transmitter (TX) implementation.
use core::{
    convert::Infallible,
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicUsize, Ordering},
};

use arbitrary_int::u6;
use embassy_sync::waitqueue::AtomicWaker;
use zynq7000::uart::FifoTrigger;

use crate::uart::{FIFO_DEPTH, Tx, UartId};

static UART_TX_WAKERS: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];
static TX_CONTEXTS: [TxContext; 2] = [const { TxContext::new() }; 2];
// Completion flag. Kept outside of the context structure as an atomic to avoid
// critical section.
static TX_DONE: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

/// TX context structure. Plain atomics rather than a `critical_section::Mutex<RefCell<_>>` so it
/// can live in a `static` array directly. `raw_data` doubles as the "transfer active" flag: it
/// is always published last (`Release`) after `transfer_len`/`progress`, and read first
/// (`Acquire`) before them, so a reader that observes it non-null is guaranteed to see the
/// matching, not stale, `transfer_len`/`progress`.
struct TxContext {
    progress: AtomicUsize,
    raw_data: AtomicPtr<u8>,
    transfer_len: AtomicUsize,
}

impl TxContext {
    const fn new() -> Self {
        Self {
            progress: AtomicUsize::new(0),
            raw_data: AtomicPtr::new(core::ptr::null_mut()),
            transfer_len: AtomicUsize::new(0),
        }
    }
}

/// This is a generic interrupt handler to handle asynchronous UART TX operations for a given
/// UART peripheral.
///
/// # Safety
///
/// The user has to call this once in the interrupt handler responsible for the TX interrupts on
/// the given UART bank.
pub unsafe fn on_interrupt_tx(peripheral: UartId) {
    let mut tx_with_irq = unsafe { Tx::steal(peripheral) };
    let idx = peripheral as usize;
    let enabled_irqs = tx_with_irq.regs().read_enabled_interrupts();
    // IRQ is not related to TX.
    if !enabled_irqs.tx_over()
        && !enabled_irqs.tx_near_full()
        && !enabled_irqs.tx_full()
        && !enabled_irqs.tx_empty()
    {
        return;
    }

    let interrupt_status = tx_with_irq.regs().read_interrupt_status();
    // Disable interrupts, re-enable them later.
    tx_with_irq.disable_interrupts();
    // Clear interrupts.
    tx_with_irq.clear_interrupts();

    let context = &TX_CONTEXTS[idx];
    // `Acquire` pairs with the `Release` store in `TxFuture::new`/`poll`/`Drop`: seeing a
    // non-null pointer here guarantees `transfer_len`/`progress` below are the values published
    // together with it, not stale ones from a previous transfer.
    let raw_data_ptr = context.raw_data.load(Ordering::Acquire);
    // No transfer active.
    if raw_data_ptr.is_null() {
        return;
    }
    let slice_len = context.transfer_len.load(Ordering::Relaxed);
    let mut progress = context.progress.load(Ordering::Relaxed);
    // Safety: We documented that the user provided slice must outlive the future, so we convert
    // the raw pointer back to the slice here.
    let slice = unsafe { core::slice::from_raw_parts(raw_data_ptr as *const u8, slice_len) };

    if (progress >= slice_len && interrupt_status.tx_empty()) || slice_len == 0 {
        // Transfer is done. `Release` publishes the final `progress` value to whichever context
        // observes `TX_DONE` via the `Acquire` swap in `poll`.
        TX_DONE[idx].store(true, Ordering::Release);
        tx_with_irq.disable_interrupts();
        tx_with_irq.clear_interrupts();
        UART_TX_WAKERS[idx].wake();
        return;
    }

    // Pump the FIFO.
    while progress < slice_len {
        match tx_with_irq.write_fifo(slice[progress]) {
            Ok(_) => progress += 1,
            Err(nb::Error::WouldBlock) => break,
        }
    }
    let remaining = slice_len - progress;
    if remaining > FIFO_DEPTH {
        tx_with_irq.regs.write_tx_fifo_trigger(
            FifoTrigger::builder()
                .with_trigger(u6::new((FIFO_DEPTH / 2) as u8))
                .build(),
        );
    }
    context.progress.store(progress, Ordering::Relaxed);

    tx_with_irq.enable_interrupts(remaining > FIFO_DEPTH);
}

/// Transmission future for UART TX.
pub struct TxFuture<'uart, 'buf> {
    id: UartId,
    buffer_empty: bool,
    // Set once `poll` observes completion. `TX_DONE` itself is not enough to tell completion
    // and cancellation apart in `Drop`, because `poll` already swaps it back to `false` as
    // part of observing it.
    completed: bool,
    phantom: core::marker::PhantomData<(&'uart (), &'buf ())>,
}

impl<'uart, 'buf> TxFuture<'uart, 'buf> {
    /// Constructor for TX future.
    pub fn new(tx_with_irq: &'uart mut Tx, data: &'buf [u8]) -> TxFuture<'uart, 'buf> {
        if data.is_empty() {
            // Nothing to transfer, return a future which is immediately ready.
            return TxFuture {
                id: tx_with_irq.uart_id(),
                buffer_empty: true,
                completed: false,
                phantom: PhantomData,
            };
        }
        let idx = tx_with_irq.uart_id() as usize;
        TX_DONE[idx].store(false, Ordering::Relaxed);
        tx_with_irq.disable_interrupts();
        tx_with_irq.disable();

        let init_fill_count = core::cmp::min(data.len(), FIFO_DEPTH);
        let context = &TX_CONTEXTS[idx];
        // Publish the guarded fields before opening the gate (`raw_data`) with `Release`, so a
        // reader that observes `raw_data` non-null via the `Acquire` load in `on_interrupt_tx`
        // is guaranteed to see these too, rather than stale values from a previous transfer.
        context.transfer_len.store(data.len(), Ordering::Relaxed);
        context.progress.store(init_fill_count, Ordering::Relaxed);
        context
            .raw_data
            .store(data.as_ptr() as *mut u8, Ordering::Release);
        // Apparently, we need to enable the UART before we are able to write something into
        // the FIFO.
        tx_with_irq.enable(false);
        if data.len() > FIFO_DEPTH {
            tx_with_irq.regs.write_tx_fifo_trigger(
                FifoTrigger::builder()
                    .with_trigger(u6::new((FIFO_DEPTH / 2) as u8))
                    .build(),
            );
        }
        for data in data.iter().take(init_fill_count) {
            tx_with_irq.write_fifo_unchecked(*data);
        }
        tx_with_irq.enable_interrupts(data.len() > FIFO_DEPTH);

        Self {
            id: tx_with_irq.uart_id(),
            buffer_empty: false,
            completed: false,
            phantom: PhantomData,
        }
    }
}

impl Future for TxFuture<'_, '_> {
    type Output = usize;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        if self.buffer_empty {
            return core::task::Poll::Ready(0);
        }
        UART_TX_WAKERS[self.id as usize].register(cx.waker());
        if TX_DONE[self.id as usize].swap(false, Ordering::Acquire) {
            let context = &TX_CONTEXTS[self.id as usize];
            // Release: pairs with the Acquire load in `on_interrupt_tx`, so a spurious interrupt
            // for this slot after completion can never see a dangling pointer.
            context
                .raw_data
                .store(core::ptr::null_mut(), Ordering::Release);
            let progress = context.progress.load(Ordering::Relaxed);
            self.completed = true;
            return core::task::Poll::Ready(progress);
        }
        core::task::Poll::Pending
    }
}

impl Drop for TxFuture<'_, '_> {
    fn drop(&mut self) {
        let mut tx = unsafe { Tx::steal(self.id) };
        tx.disable_interrupts();
        // On cancellation, clear the stale buffer pointer so a spurious or future interrupt for
        // this UART can never dereference it. `self.completed` (set inside `poll`'s `Ready` arm)
        // is what actually distinguishes cancellation from normal completion here, since
        // `TX_DONE` itself is already swapped back to `false` by the time a completed future is
        // dropped.
        if !self.buffer_empty && !self.completed {
            let context = &TX_CONTEXTS[self.id as usize];
            context.progress.store(0, Ordering::Relaxed);
            context
                .raw_data
                .store(core::ptr::null_mut(), Ordering::Release);
        }
    }
}

/// Asynchronous UART transmitter (TX) driver.
pub struct TxAsync {
    tx: Tx,
}

impl TxAsync {
    /// Constructor.
    ///
    /// The second argument specifies whether the [on_interrupt_tx] function will be registered
    /// in the HAL interrupt map. You might need to skip this in case you have your own
    /// interrupt handler which also handles RX interrupts.
    ///
    /// # Safety
    ///
    /// This function stores the raw pointer of the passed data slice. The user MUST ensure
    /// that the slice outlives the data structure.
    /// This case was considered exotic enough to not justify an `unsafe` API.
    pub fn new(tx: Tx, register_interrupt_handler: bool) -> Self {
        if register_interrupt_handler {
            match tx.uart_id() {
                UartId::Uart0 => {
                    unsafe fn uart0_interrupt_handler() {
                        unsafe {
                            on_interrupt_tx(UartId::Uart0);
                        }
                    }
                    crate::register_interrupt(
                        crate::gic::Interrupt::Spi(crate::gic::SpiInterrupt::Uart0),
                        uart0_interrupt_handler,
                    )
                }
                UartId::Uart1 => {
                    unsafe fn uart1_interrupt_handler() {
                        unsafe {
                            on_interrupt_tx(UartId::Uart1);
                        }
                    }
                    crate::register_interrupt(
                        crate::gic::Interrupt::Spi(crate::gic::SpiInterrupt::Uart1),
                        uart1_interrupt_handler,
                    )
                }
            }
        }

        Self { tx }
    }

    /// Write a buffer asynchronously.
    ///
    /// This implementation is not side effect free, and a started future might have already
    /// written part of the passed buffer.
    pub fn write<'buf>(&mut self, buf: &'buf [u8]) -> TxFuture<'_, 'buf> {
        TxFuture::new(&mut self.tx, buf)
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
