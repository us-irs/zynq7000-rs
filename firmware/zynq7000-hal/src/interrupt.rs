use crate::gic::{Interrupt, InterruptGuard};

pub type InterruptMap = heapless::index_map::FnvIndexMap<Interrupt, unsafe fn(), 128>;
static INTERRUPT_MAP: critical_section::Mutex<core::cell::RefCell<InterruptMap>> =
    critical_section::Mutex::new(core::cell::RefCell::new(
        heapless::index_map::FnvIndexMap::new(),
    ));

/// Register an interrupt handler for a specific [Interrupt].
///
/// It should be noted that the current implementation only allows one function for each interrupts.
/// If the HAL provided interrupt handler does not fulfill all your requirements, you need
/// to define your own interrupt handler and register it.
/// For example, you might need to handle both UART RX and TX, and the HAL handler only handles TX.
pub fn register_interrupt(interrupt: Interrupt, handler: unsafe fn()) {
    critical_section::with(|cs| {
        let mut map = INTERRUPT_MAP.borrow(cs).borrow_mut();
        map.insert(interrupt, handler).ok();
    });
}

/// Generic interrupt handler which retrieves the interrupt handler for individual [Interrupt]s
/// from a registry and calls it.
///
/// If no interrupt was registered or the number is [Interrupt::Invalid], returns the [Interrupt]
/// ID as an error. In any case, the generic handler acknowledges the interrupt in the GIC.
///
/// # Safety
///
/// This needs to be called ONCE in the interrupt handler function, which is any function annotated
/// with the `irq` attribute provided by `aarch32-rt`.
pub unsafe fn generic_interrupt_handler() -> Result<(), Interrupt> {
    let mut gic_guard = InterruptGuard::new();
    let irq_info = gic_guard.interrupt_info();
    let interrupt = irq_info.interrupt();
    if let Interrupt::Invalid(_) = interrupt {
        gic_guard.end_of_interrupt(irq_info);
        return Err(interrupt);
    }
    let opt_interrupt_handler = critical_section::with(|cs| {
        let map = INTERRUPT_MAP.borrow(cs).borrow_mut();
        map.get(&interrupt).copied()
    });
    if let Some(interrupt_handler) = opt_interrupt_handler {
        // Safety: The user made sure that this is only called once in the interrupt handler
        // function.
        unsafe {
            interrupt_handler();
        }
    }
    gic_guard.end_of_interrupt(irq_info);
    opt_interrupt_handler.ok_or(interrupt)?;
    Ok(())
}
