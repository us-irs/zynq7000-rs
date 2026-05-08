//! Asynchronous PS SPI driver.
use core::{cell::RefCell, sync::atomic::AtomicBool};

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal_async::spi::SpiBus;
use raw_slice::{RawBufSlice, RawBufSliceMut};

use super::{ChipSelect, FIFO_DEPTH, Spi, SpiId, SpiLowLevel};

static WAKERS: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];
static TRANSFER_CONTEXTS: [Mutex<RefCell<TransferContext>>; 2] =
    [const { Mutex::new(RefCell::new(TransferContext::new())) }; 2];
// Completion flag. Kept outside of the context structure as an atomic to avoid
// critical section.
static DONE: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

#[derive(Debug, Clone, Copy, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[error("SPI RX FIFO overrun")]
pub struct RxOverrunError;

impl embedded_hal_async::spi::Error for RxOverrunError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Overrun
    }
}

/// This is a generic interrupt handler to handle asynchronous SPI  operations for a given
/// SPI peripheral.
///
/// The user has to call this once in the interrupt handler responsible for the SPI interrupts on
/// the given SPI bank.
pub fn on_interrupt(peripheral: SpiId) {
    let mut spi = unsafe { SpiLowLevel::steal(peripheral) };
    let index = peripheral as usize;
    let enabled_irqs = spi.read_enabled_interrupts();
    // Prevent spurious interrupts from messing with out logic here.
    spi.disable_interrupts();
    let interrupt_status = spi.read_interrupt_status();
    spi.clear_interrupts();
    // IRQ is not related.
    if !enabled_irqs.tx_below_threshold()
        && !enabled_irqs.tx_full()
        && !enabled_irqs.tx_underflow()
        && !enabled_irqs.rx_ovr()
        && !enabled_irqs.rx_full()
    {
        return;
    }
    if interrupt_status.rx_overrun() {
        // Not sure how to otherwise handle this cleanly..
        return handle_rx_overrun(&mut spi, index);
    }
    let mut context = critical_section::with(|cs| {
        let context_ref = TRANSFER_CONTEXTS[index].borrow(cs);
        *context_ref.borrow()
    });
    // No transfer active.
    if context.transfer_type.is_none() {
        return;
    }

    // Write the trigger to one, we want to empty the whole FIFO in the transfer handlers.
    // The trigger might have been set to a higher value, and the NOT FULL status bit will be
    // cleared when the FIFO falls below the threshold.
    spi.write_rx_trig(0x1);

    let transfer_type = context.transfer_type.unwrap();
    match transfer_type {
        TransferType::Read => on_interrupt_read(index, &mut context, &mut spi),
        TransferType::Write => on_interrupt_write(index, &mut context, &mut spi),
        TransferType::Transfer => on_interrupt_transfer(index, &mut context, &mut spi),
        TransferType::TransferInPlace => {
            on_interrupt_transfer_in_place(index, &mut context, &mut spi)
        }
    };
}

fn handle_rx_overrun(spi: &mut SpiLowLevel, idx: usize) {
    critical_section::with(|cs| {
        let context_ref = TRANSFER_CONTEXTS[idx].borrow(cs);
        context_ref.borrow_mut().rx_overrun = true;
    });
    // Clean state is re-configured by drop handler.
    reset_trigger_levels(spi);
    // At the very least disable the peripheral.
    spi.disable();

    // Interrupts were already disabled and cleared.
    DONE[idx].store(true, core::sync::atomic::Ordering::Relaxed);
    WAKERS[idx].wake();
}

#[inline]
fn reset_trigger_levels(spi: &mut SpiLowLevel) {
    spi.write_rx_trig(0x1);
    spi.write_tx_trig(0x1);
}

fn on_interrupt_read(idx: usize, context: &mut TransferContext, spi: &mut SpiLowLevel) {
    let read_slice = unsafe { context.rx_slice.get_mut().unwrap() };
    let transfer_len = read_slice.len();

    // Read data from RX FIFO first.
    while spi.read_interrupt_status().rx_not_empty() {
        read_slice[context.rx_progress] = spi.read_fifo_unchecked();
        context.rx_progress += 1;
    }

    // The FIFO still needs to be pumped.
    while context.tx_progress < read_slice.len() && !spi.read_interrupt_status().tx_full() {
        spi.write_fifo_unchecked(0);
        context.tx_progress += 1;
    }

    isr_finish_handler(idx, spi, context, transfer_len)
}

fn on_interrupt_write(idx: usize, context: &mut TransferContext, spi: &mut SpiLowLevel) {
    let write_slice = unsafe { context.tx_slice.get().unwrap() };
    let transfer_len = write_slice.len();

    // Read data from RX FIFO first.
    while spi.read_interrupt_status().rx_not_empty() {
        spi.read_fifo_unchecked();
        context.rx_progress += 1;
    }

    // Data still needs to be sent
    while context.tx_progress < transfer_len && !spi.read_interrupt_status().tx_full() {
        spi.write_fifo_unchecked(write_slice[context.tx_progress]);
        context.tx_progress += 1;
    }

    isr_finish_handler(idx, spi, context, transfer_len)
}

fn on_interrupt_transfer(idx: usize, context: &mut TransferContext, spi: &mut SpiLowLevel) {
    let read_slice = unsafe { context.rx_slice.get_mut().unwrap() };
    let read_len = read_slice.len();
    let write_slice = unsafe { context.tx_slice.get().unwrap() };
    let write_len = write_slice.len();
    let transfer_len = core::cmp::max(read_len, write_len);

    while context.tx_progress < transfer_len && !spi.read_interrupt_status().tx_full() {
        if context.tx_progress < write_len {
            spi.write_fifo_unchecked(write_slice[context.tx_progress]);
        } else {
            // Dummy write.
            spi.write_fifo_unchecked(0);
        }
        context.tx_progress += 1;
    }

    // Read data from RX FIFO first.
    while spi.read_interrupt_status().rx_not_empty() {
        if context.rx_progress < read_len {
            read_slice[context.rx_progress] = spi.read_fifo_unchecked();
        } else {
            spi.read_fifo_unchecked();
        }
        context.rx_progress += 1;
    }

    isr_finish_handler(idx, spi, context, transfer_len)
}

fn on_interrupt_transfer_in_place(
    idx: usize,
    context: &mut TransferContext,
    spi: &mut SpiLowLevel,
) {
    let transfer_slice = unsafe { context.rx_slice.get_mut().unwrap() };
    let transfer_len = transfer_slice.len();

    // Send data first to avoid overwriting data that still needs to be sent.
    while context.tx_progress < transfer_len && !spi.read_interrupt_status().tx_full() {
        spi.write_fifo_unchecked(transfer_slice[context.tx_progress]);
        context.tx_progress += 1;
    }

    // Read data from RX FIFO.
    while spi.read_interrupt_status().rx_not_empty() {
        transfer_slice[context.rx_progress] = spi.read_fifo_unchecked();
        context.rx_progress += 1;
    }

    isr_finish_handler(idx, spi, context, transfer_len)
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

    unfinished_transfer(spi, transfer_len, context);

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
    // Default reset values.
    spi.set_rx_fifo_trigger(1).unwrap();
    spi.set_tx_fifo_trigger(1).unwrap();
    // Interrupts were already disabled and cleared.
    DONE[idx].store(true, core::sync::atomic::Ordering::Relaxed);
    WAKERS[idx].wake();
}

fn unfinished_transfer(spi: &mut SpiLowLevel, transfer_len: usize, context: &TransferContext) {
    // Unwraps okay, checks ensure number is below FIFO depth.

    // By using the FIFO depth divided by two, give the software some time and allow more
    // latency in the system.
    spi.set_rx_fifo_trigger(
        core::cmp::min(FIFO_DEPTH / 2, transfer_len - context.rx_progress) as u32,
    )
    .unwrap();
    let tx_pending = context.tx_progress < transfer_len;
    if tx_pending {
        let remaining_tx = transfer_len - context.tx_progress;
        let tx_trigger = if remaining_tx < FIFO_DEPTH / 2 {
            FIFO_DEPTH as u32 - remaining_tx as u32
        } else {
            FIFO_DEPTH as u32 / 2
        };
        spi.set_tx_fifo_trigger(tx_trigger).unwrap();
    } else {
        spi.set_tx_fifo_trigger(1).unwrap();
    }
    // Re-enable interrupts with the new RX FIFO trigger level. Only enable TX threshold interrupt
    // if we are not done yet with pushing all bytes to the FIFO.
    spi.enable_interrupts(tx_pending);
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
    rx_overrun: bool,
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
            rx_overrun: false,
        }
    }
}

pub struct SpiFuture<'spi> {
    id: super::SpiId,
    spi: &'spi mut super::SpiLowLevel,
    config: super::Config,
    finished_regularly: core::cell::Cell<bool>,
}

impl<'spi> SpiFuture<'spi> {
    fn new_for_read(spi: &'spi mut Spi, spi_id: SpiId, words: &mut [u8]) -> Self {
        if words.is_empty() {
            panic!("words length unexpectedly 0");
        }
        Self::generic_init_transfer(spi, spi_id);

        let write_index = core::cmp::min(super::FIFO_DEPTH, words.len());
        // Send dummy bytes.
        (0..write_index).for_each(|_| {
            spi.inner.write_fifo_unchecked(0);
        });

        Self::set_triggers(spi, write_index, words.len());
        // We assume that the slave select configuration was already performed, but we take
        // care of issuing a start if necessary.
        spi.issue_manual_start_for_manual_cfg();

        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[spi_id as usize].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::Read);
            unsafe {
                context.rx_slice.set(words);
            }
            context.tx_slice.set_null();
            context.tx_progress = write_index;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts(write_index < words.len());
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: &mut spi.inner,
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn new_for_write(spi: &'spi mut Spi, spi_id: SpiId, words: &[u8]) -> Self {
        if words.is_empty() {
            panic!("words length unexpectedly 0");
        }
        let write_index = Self::generic_init_transfer_write_transfer_in_place(spi, spi_id, words);
        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[spi_id as usize].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::Write);
            unsafe {
                context.tx_slice.set(words);
            }
            context.rx_slice.set_null();
            context.tx_progress = write_index;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts(write_index < words.len());
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: &mut spi.inner,
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn new_for_transfer(spi: &'spi mut Spi, spi_id: SpiId, read: &mut [u8], write: &[u8]) -> Self {
        if read.is_empty() || write.is_empty() {
            panic!("read or write buffer unexpectedly empty");
        }
        let full_write_len = core::cmp::max(read.len(), write.len());
        let fifo_prefill = core::cmp::min(super::FIFO_DEPTH, full_write_len);

        Self::generic_init_transfer(spi, spi_id);

        for write_index in 0..fifo_prefill {
            let value = write.get(write_index).copied().unwrap_or(0);
            spi.inner.write_fifo_unchecked(value);
        }

        Self::set_triggers(spi, fifo_prefill, full_write_len);
        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[spi_id as usize].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::Transfer);
            unsafe {
                context.tx_slice.set(write);
                context.rx_slice.set(read);
            }
            context.tx_progress = fifo_prefill;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts(fifo_prefill < write.len());
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: &mut spi.inner,
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn new_for_transfer_in_place(spi: &'spi mut Spi, spi_id: SpiId, words: &mut [u8]) -> Self {
        if words.is_empty() {
            panic!("read and write buffer unexpectedly empty");
        }
        let write_index = Self::generic_init_transfer_write_transfer_in_place(spi, spi_id, words);
        critical_section::with(|cs| {
            let context_ref = TRANSFER_CONTEXTS[spi_id as usize].borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.transfer_type = Some(TransferType::TransferInPlace);
            unsafe {
                context.rx_slice.set(words);
            }
            context.tx_slice.set_null();
            context.tx_progress = write_index;
            context.rx_progress = 0;
            spi.inner.clear_interrupts();
            spi.inner.enable_interrupts(write_index < words.len());
            spi.inner.enable();
        });
        Self {
            id: spi_id,
            config: spi.config,
            spi: &mut spi.inner,
            finished_regularly: core::cell::Cell::new(false),
        }
    }

    fn generic_init_transfer(spi: &mut Spi, id: SpiId) {
        let idx = id as usize;
        DONE[idx].store(false, core::sync::atomic::Ordering::Relaxed);
        spi.inner.disable();
        spi.inner.disable_interrupts();
    }

    // Returns amount of bytes written to FIFO.
    fn generic_init_transfer_write_transfer_in_place(
        spi: &mut Spi,
        id: SpiId,
        write: &[u8],
    ) -> usize {
        Self::generic_init_transfer(spi, id);

        let write_idx = core::cmp::min(super::FIFO_DEPTH, write.len());
        Self::set_triggers(spi, write_idx, write.len());
        (0..write_idx).for_each(|idx| {
            spi.inner.write_fifo_unchecked(write[idx]);
        });

        // We assume that the slave select configuration was already performed, but we take
        // care of issuing a start if necessary.
        spi.issue_manual_start_for_manual_cfg();
        write_idx
    }

    fn set_triggers(spi: &mut Spi, write_idx: usize, write_len: usize) {
        spi.inner
            .set_rx_fifo_trigger(core::cmp::min(
                super::FIFO_DEPTH as u32 / 2,
                write_idx as u32,
            ))
            .unwrap();
        // We want to re-fill the TX FIFO before it is completely empty if the full transfer size
        // is larger than the FIFO depth. Otherwise, set it to 1. Not exactly sure what that does,
        // but we do not enable interrupts anyway.
        if write_len > super::FIFO_DEPTH {
            spi.inner
                .set_tx_fifo_trigger(super::FIFO_DEPTH as u32 / 2)
                .unwrap();
        } else {
            spi.inner.set_tx_fifo_trigger(1).unwrap();
        }
    }
}

impl Future for SpiFuture<'_> {
    type Output = Result<(), RxOverrunError>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        WAKERS[self.id as usize].register(cx.waker());
        if DONE[self.id as usize].swap(false, core::sync::atomic::Ordering::Relaxed) {
            let rx_overrun = critical_section::with(|cs| {
                let mut ctx = TRANSFER_CONTEXTS[self.id as usize].borrow(cs).borrow_mut();
                let overrun = ctx.rx_overrun;
                *ctx = TransferContext::default();
                overrun
            });
            self.finished_regularly.set(!rx_overrun);
            if rx_overrun {
                return core::task::Poll::Ready(Err(RxOverrunError));
            }
            return core::task::Poll::Ready(Ok(()));
        }
        core::task::Poll::Pending
    }
}

impl Drop for SpiFuture<'_> {
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

    pub fn read(&mut self, words: &mut [u8]) -> Option<SpiFuture<'_>> {
        if words.is_empty() {
            return None;
        }
        let id = self.0.inner.id;
        Some(SpiFuture::new_for_read(&mut self.0, id, words))
    }

    pub fn write(&mut self, words: &[u8]) -> Option<SpiFuture<'_>> {
        if words.is_empty() {
            return None;
        }
        let id = self.0.inner.id;
        Some(SpiFuture::new_for_write(&mut self.0, id, words))
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Option<SpiFuture<'_>> {
        if read.is_empty() || write.is_empty() {
            return None;
        }
        let id = self.0.inner.id;
        Some(SpiFuture::new_for_transfer(&mut self.0, id, read, write))
    }

    pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Option<SpiFuture<'_>> {
        if words.is_empty() {
            return None;
        }
        let id = self.0.inner.id;
        Some(SpiFuture::new_for_transfer_in_place(&mut self.0, id, words))
    }
}

impl embedded_hal_async::spi::ErrorType for SpiAsync {
    type Error = RxOverrunError;
}

impl embedded_hal_async::spi::SpiBus for SpiAsync {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        self.read(words).unwrap().await?;
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        self.write(words).unwrap().await?;
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        if read.is_empty() && write.is_empty() {
            return Ok(());
        }
        self.transfer(read, write).unwrap().await?;
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        self.transfer_in_place(words).unwrap().await?;
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
    type Error = RxOverrunError;
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
                    if let Some(fut) = self.spi.read(items) {
                        fut.await?;
                    }
                }
                embedded_hal::spi::Operation::Write(items) => {
                    if let Some(fut) = self.spi.write(items) {
                        fut.await?;
                    }
                }
                embedded_hal::spi::Operation::Transfer(read, write) => {
                    if let Some(fut) = self.spi.transfer(read, write) {
                        fut.await?;
                    }
                }
                embedded_hal::spi::Operation::TransferInPlace(items) => {
                    if let Some(fut) = self.spi.transfer_in_place(items) {
                        fut.await?;
                    }
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
