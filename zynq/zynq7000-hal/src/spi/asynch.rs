//! Asynchronous PS SPI driver.
use core::{cell::RefCell, convert::Infallible, sync::atomic::AtomicBool};

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal_async::spi::SpiBus;
use raw_slice::{RawBufSlice, RawBufSliceMut};
use zynq7000::spi::InterruptStatus;

use super::{ChipSelect, FIFO_DEPTH, Spi, SpiId, SpiLowLevel};

static WAKERS: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];
static TRANSFER_CONTEXTS: [Mutex<RefCell<TransferContext>>; 2] =
    [const { Mutex::new(RefCell::new(TransferContext::new())) }; 2];
// Completion flag. Kept outside of the context structure as an atomic to avoid
// critical section.
static DONE: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

/// This is a generic interrupt handler to handle asynchronous SPI  operations for a given
/// SPI peripheral.
///
/// The user has to call this once in the interrupt handler responsible for the SPI interrupts on
/// the given SPI bank.
pub fn on_interrupt(peripheral: SpiId) {
    let mut spi = unsafe { SpiLowLevel::steal(peripheral) };
    let idx = peripheral as usize;
    let imr = spi.read_imr();
    // IRQ is not related.
    if !imr.tx_trig() && !imr.tx_full() && !imr.tx_underflow() && !imr.rx_ovr() && !imr.rx_full() {
        return;
    }
    // Prevent spurious interrupts from messing with out logic here.
    spi.disable_interrupts();
    let isr = spi.read_isr();
    spi.clear_interrupts();
    let mut context = critical_section::with(|cs| {
        let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
        *context_ref.borrow()
    });
    // No transfer active.
    if context.transfer_type.is_none() {
        return;
    }
    let transfer_type = context.transfer_type.unwrap();
    match transfer_type {
        TransferType::Read => on_interrupt_read(idx, &mut context, &mut spi, isr),
        TransferType::Write => on_interrupt_write(idx, &mut context, &mut spi, isr),
        TransferType::Transfer => on_interrupt_transfer(idx, &mut context, &mut spi, isr),
        TransferType::TransferInPlace => {
            on_interrupt_transfer_in_place(idx, &mut context, &mut spi, isr)
        }
    };
}

fn on_interrupt_read(
    idx: usize,
    context: &mut TransferContext,
    spi: &mut SpiLowLevel,
    mut isr: InterruptStatus,
) {
    let read_slice = unsafe { context.rx_slice.get_mut().unwrap() };
    let transfer_len = read_slice.len();

    // Read data from RX FIFO first.
    let read_len = calculate_read_len(spi, isr, transfer_len, context.rx_progress);
    (0..read_len).for_each(|_| {
        read_slice[context.rx_progress] = spi.read_fifo_unchecked();
        context.rx_progress += 1;
    });

    // The FIFO still needs to be pumped.
    while context.tx_progress < read_slice.len() && !isr.tx_full() {
        spi.write_fifo_unchecked(0);
        context.tx_progress += 1;
        isr = spi.read_isr();
    }

    isr_finish_handler(idx, spi, context, transfer_len)
}

fn on_interrupt_write(
    idx: usize,
    context: &mut TransferContext,
    spi: &mut SpiLowLevel,
    mut isr: InterruptStatus,
) {
    let write_slice = unsafe { context.tx_slice.get().unwrap() };
    let transfer_len = write_slice.len();

    // Read data from RX FIFO first.
    let read_len = calculate_read_len(spi, isr, transfer_len, context.rx_progress);
    (0..read_len).for_each(|_| {
        spi.read_fifo_unchecked();
        context.rx_progress += 1;
    });

    // Data still needs to be sent
    while context.tx_progress < transfer_len && !isr.tx_full() {
        spi.write_fifo_unchecked(write_slice[context.tx_progress]);
        context.tx_progress += 1;
        isr = spi.read_isr();
    }

    isr_finish_handler(idx, spi, context, transfer_len)
}

fn on_interrupt_transfer(
    idx: usize,
    context: &mut TransferContext,
    spi: &mut SpiLowLevel,
    mut isr: InterruptStatus,
) {
    let read_slice = unsafe { context.rx_slice.get_mut().unwrap() };
    let read_len = read_slice.len();
    let write_slice = unsafe { context.tx_slice.get().unwrap() };
    let write_len = write_slice.len();
    let transfer_len = core::cmp::max(read_len, write_len);

    // Read data from RX FIFO first.
    let read_len = calculate_read_len(spi, isr, transfer_len, context.rx_progress);
    (0..read_len).for_each(|_| {
        if context.rx_progress < read_len {
            read_slice[context.rx_progress] = spi.read_fifo_unchecked();
        } else {
            spi.read_fifo_unchecked();
        }
        context.rx_progress += 1;
    });

    // Data still needs to be sent
    while context.tx_progress < transfer_len && !isr.tx_full() {
        if context.tx_progress < write_len {
            spi.write_fifo_unchecked(write_slice[context.tx_progress]);
        } else {
            // Dummy write.
            spi.write_fifo_unchecked(0);
        }
        context.tx_progress += 1;
        isr = spi.read_isr();
    }

    isr_finish_handler(idx, spi, context, transfer_len)
}

fn on_interrupt_transfer_in_place(
    idx: usize,
    context: &mut TransferContext,
    spi: &mut SpiLowLevel,
    mut isr: InterruptStatus,
) {
    let transfer_slice = unsafe { context.rx_slice.get_mut().unwrap() };
    let transfer_len = transfer_slice.len();
    // Read data from RX FIFO first.
    let read_len = calculate_read_len(spi, isr, transfer_len, context.rx_progress);
    (0..read_len).for_each(|_| {
        transfer_slice[context.rx_progress] = spi.read_fifo_unchecked();
        context.rx_progress += 1;
    });

    // Data still needs to be sent
    while context.tx_progress < transfer_len && !isr.tx_full() {
        spi.write_fifo_unchecked(transfer_slice[context.tx_progress]);
        context.tx_progress += 1;
        isr = spi.read_isr();
    }

    isr_finish_handler(idx, spi, context, transfer_len)
}

fn calculate_read_len(
    spi: &mut SpiLowLevel,
    isr: InterruptStatus,
    total_read_len: usize,
    rx_progress: usize,
) -> usize {
    if isr.rx_full() {
        core::cmp::min(FIFO_DEPTH, total_read_len - rx_progress)
    } else if isr.rx_not_empty() {
        let trigger = spi.read_rx_not_empty_threshold();
        core::cmp::min(total_read_len - rx_progress, trigger as usize)
    } else {
        0
    }
}

/// Generic handler after RX FIFO and TX FIFO were handled. Checks and handles finished
/// and unfinished conditions.
fn isr_finish_handler(
    idx: usize,
    spi: &mut SpiLowLevel,
    context: &mut TransferContext,
    transfer_len: usize,
) {
    // Transfer finish condition.
    if context.rx_progress == context.tx_progress && context.rx_progress == transfer_len {
        finish_transfer(idx, context, spi);
        return;
    }

    unfinished_transfer(spi, transfer_len, context.rx_progress);

    // If the transfer is done, the context structure was already written back.
    // Write back updated context structure.
    critical_section::with(|cs| {
        let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
        *context_ref.borrow_mut() = *context;
    });
}

fn finish_transfer(idx: usize, context: &mut TransferContext, spi: &mut SpiLowLevel) {
    // Write back updated context structure.
    critical_section::with(|cs| {
        let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
        *context_ref.borrow_mut() = *context;
    });
    spi.set_rx_fifo_trigger(1).unwrap();
    spi.set_tx_fifo_trigger(1).unwrap();
    // Interrupts were already disabled and cleared.
    DONE[idx].store(true, core::sync::atomic::Ordering::Relaxed);
    WAKERS[idx].wake();
}

fn unfinished_transfer(spi: &mut SpiLowLevel, transfer_len: usize, rx_progress: usize) {
    let new_trig_level = core::cmp::min(FIFO_DEPTH, transfer_len - rx_progress);
    spi.set_rx_fifo_trigger(new_trig_level as u32).unwrap();
    // Re-enable interrupts with the new RX FIFO trigger level.
    spi.enable_interrupts();
}

#[derive(Debug, Clone, Copy)]
pub enum TransferType {
    Read,
    Write,
    Transfer,
    TransferInPlace,
}

#[derive(Default, Debug, Copy, Clone)]
pub struct TransferContext {
    transfer_type: Option<TransferType>,
    tx_progress: usize,
    rx_progress: usize,
    tx_slice: RawBufSlice,
    rx_slice: RawBufSliceMut,
}

#[allow(clippy::new_without_default)]
impl TransferContext {
    pub const fn new() -> Self {
        Self {
            transfer_type: None,
            tx_progress: 0,
            rx_progress: 0,
            tx_slice: RawBufSlice::new_nulled(),
            rx_slice: RawBufSliceMut::new_nulled(),
        }
    }
}

pub struct SpiFuture {
    id: super::SpiId,
    spi: super::SpiLowLevel,
    config: super::Config,
    finished_regularly: core::cell::Cell<bool>,
}

impl SpiFuture {
    fn new_for_read(spi: &mut Spi, spi_id: SpiId, words: &mut [u8]) -> Self {
        if words.is_empty() {
            panic!("words length unexpectedly 0");
        }
        let idx = spi_id as usize;
        DONE[idx].store(false, core::sync::atomic::Ordering::Relaxed);
        spi.inner.disable_interrupts();

        let write_idx = core::cmp::min(super::FIFO_DEPTH, words.len());
        // Send dummy bytes.
        (0..write_idx).for_each(|_| {
            spi.inner.write_fifo_unchecked(0);
        });

        Self::set_triggers(spi, write_idx, words.len());
        // We assume that the slave select configuration was already performed, but we take
        // care of issuing a start if necessary.
        spi.issue_manual_start_for_manual_cfg();

        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::Read);
            unsafe {
                context.rx_slice.set(words);
            }
            context.tx_slice.set_null();
            context.tx_progress = write_idx;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts();
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: unsafe { spi.inner.clone() },
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn new_for_write(spi: &mut Spi, spi_id: SpiId, words: &[u8]) -> Self {
        if words.is_empty() {
            panic!("words length unexpectedly 0");
        }
        let (idx, write_idx) = Self::generic_init_transfer(spi, spi_id, words);
        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::Write);
            unsafe {
                context.tx_slice.set(words);
            }
            context.rx_slice.set_null();
            context.tx_progress = write_idx;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts();
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: unsafe { spi.inner.clone() },
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn new_for_transfer(spi: &mut Spi, spi_id: SpiId, read: &mut [u8], write: &[u8]) -> Self {
        if read.is_empty() || write.is_empty() {
            panic!("read or write buffer unexpectedly empty");
        }
        let (idx, write_idx) = Self::generic_init_transfer(spi, spi_id, write);
        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::Transfer);
            unsafe {
                context.tx_slice.set(write);
                context.rx_slice.set(read);
            }
            context.tx_progress = write_idx;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts();
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: unsafe { spi.inner.clone() },
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn new_for_transfer_in_place(spi: &mut Spi, spi_id: SpiId, words: &mut [u8]) -> Self {
        if words.is_empty() {
            panic!("read and write buffer unexpectedly empty");
        }
        let (idx, write_idx) = Self::generic_init_transfer(spi, spi_id, words);
        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::TransferInPlace);
            unsafe {
                context.rx_slice.set(words);
            }
            context.tx_slice.set_null();
            context.tx_progress = write_idx;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts();
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: unsafe { spi.inner.clone() },
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn generic_init_transfer(spi: &mut Spi, spi_id: SpiId, write: &[u8]) -> (usize, usize) {
        let idx = spi_id as usize;
        DONE[idx].store(false, core::sync::atomic::Ordering::Relaxed);
        spi.inner.disable();
        spi.inner.disable_interrupts();

        let write_idx = core::cmp::min(super::FIFO_DEPTH, write.len());
        (0..write_idx).for_each(|idx| {
            spi.inner.write_fifo_unchecked(write[idx]);
        });

        Self::set_triggers(spi, write_idx, write.len());
        // We assume that the slave select configuration was already performed, but we take
        // care of issuing a start if necessary.
        spi.issue_manual_start_for_manual_cfg();
        (idx, write_idx)
    }

    fn set_triggers(spi: &mut Spi, write_idx: usize, write_len: usize) {
        // This should never fail because it is never larger than the FIFO depth.
        spi.inner.set_rx_fifo_trigger(write_idx as u32).unwrap();
        // We want to re-fill the TX FIFO before it is completely empty if the full transfer size
        // is larger than the FIFO depth. I am not sure whether the default value of 1 ensures
        // this because the TMR says that this interrupt is triggered when the FIFO has less than
        // threshold entries.
        if write_len > super::FIFO_DEPTH {
            spi.inner.set_tx_fifo_trigger(2).unwrap();
        }
    }
}

impl Future for SpiFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        WAKERS[self.id as usize].register(cx.waker());
        if DONE[self.id as usize].swap(false, core::sync::atomic::Ordering::Relaxed) {
            critical_section::with(|cs| {
                let mut ctx = TRANSFER_CONTEXTS[self.id as usize].borrow(cs).borrow_mut();
                *ctx = TransferContext::default();
            });
            self.finished_regularly.set(true);
            return core::task::Poll::Ready(());
        }
        core::task::Poll::Pending
    }
}

impl Drop for SpiFuture {
    fn drop(&mut self) {
        if !self.finished_regularly.get() {
            // It might be sufficient to disable and enable the SPI.. But this definitely
            // ensures the SPI is fully reset.
            self.spi.reset_and_reconfigure(self.config);
        }
    }
}

/// Asynchronous SPI driver.
///
/// This is the primary data structure used to perform non-blocking SPI operations.
/// It implements the [embedded_hal_async::spi::SpiBus] as well.
pub struct SpiAsync(pub Spi);

impl SpiAsync {
    pub fn new(spi: Spi) -> Self {
        Self(spi)
    }

    async fn read(&mut self, words: &mut [u8]) {
        if words.is_empty() {
            return;
        }
        let id = self.0.inner.id;
        let spi_fut = SpiFuture::new_for_read(&mut self.0, id, words);
        spi_fut.await;
    }

    async fn write(&mut self, words: &[u8]) {
        if words.is_empty() {
            return;
        }
        let id = self.0.inner.id;
        let spi_fut = SpiFuture::new_for_write(&mut self.0, id, words);
        spi_fut.await;
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) {
        if read.is_empty() || write.is_empty() {
            return;
        }
        let id = self.0.inner.id;
        let spi_fut = SpiFuture::new_for_transfer(&mut self.0, id, read, write);
        spi_fut.await;
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) {
        if words.is_empty() {
            return;
        }
        let id = self.0.inner.id;
        let spi_fut = SpiFuture::new_for_transfer_in_place(&mut self.0, id, words);
        spi_fut.await;
    }
}

impl embedded_hal_async::spi::ErrorType for SpiAsync {
    type Error = Infallible;
}

impl embedded_hal_async::spi::SpiBus for SpiAsync {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.read(words).await;
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.write(words).await;
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer(read, write).await;
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer_in_place(words).await;
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// This structure is a wrapper for [SpiAsync] which implements the
/// [embedded_hal_async::spi::SpiDevice] trait as well.
pub struct SpiWithHwCsAsync<Delay: embedded_hal_async::delay::DelayNs> {
    pub spi: SpiAsync,
    pub cs: ChipSelect,
    pub delay: Delay,
}

impl<Delay: embedded_hal_async::delay::DelayNs> SpiWithHwCsAsync<Delay> {
    pub fn new(spi: SpiAsync, cs: ChipSelect, delay: Delay) -> Self {
        Self { spi, cs, delay }
    }

    pub fn release(self) -> SpiAsync {
        self.spi
    }
}

impl<Delay: embedded_hal_async::delay::DelayNs> embedded_hal_async::spi::ErrorType
    for SpiWithHwCsAsync<Delay>
{
    type Error = Infallible;
}

impl<Delay: embedded_hal_async::delay::DelayNs> embedded_hal_async::spi::SpiDevice
    for SpiWithHwCsAsync<Delay>
{
    async fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.spi.0.inner.select_hw_cs(self.cs);
        for op in operations {
            match op {
                embedded_hal::spi::Operation::Read(items) => {
                    self.spi.read(items).await;
                }
                embedded_hal::spi::Operation::Write(items) => {
                    self.spi.write(items).await;
                }
                embedded_hal::spi::Operation::Transfer(read, write) => {
                    self.spi.transfer(read, write).await;
                }
                embedded_hal::spi::Operation::TransferInPlace(items) => {
                    self.spi.transfer_in_place(items).await;
                }
                embedded_hal::spi::Operation::DelayNs(delay) => {
                    self.delay.delay_ns(*delay).await;
                }
            }
        }
        self.spi.flush().await?;
        self.spi.0.inner.no_hw_cs();
        Ok(())
    }
}
