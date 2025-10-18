//! # System Level Control Register (SLCR) module
use zynq7000::slcr::MmioRegisters;

pub const LOCK_KEY: u32 = 0x767B;
pub const UNLOCK_KEY: u32 = 0xDF0D;

pub struct Slcr(zynq7000::slcr::MmioRegisters<'static>);

impl Slcr {
    /// Modify the SLCR register.
    ///
    /// # Safety
    ///
    /// This method unsafely steals the SLCR MMIO block and then calls a user provided function
    /// with the [SLCR MMIO][MmioSlcr] block as an input argument. It is the user's responsibility
    /// that the SLCR is not used concurrently in a way which leads to data races.
    pub unsafe fn with<F: FnOnce(&mut MmioRegisters<'static>)>(f: F) {
        let mut slcr = unsafe { zynq7000::slcr::Registers::new_mmio_fixed() };
        slcr.write_unlock(UNLOCK_KEY);
        f(&mut slcr);
        slcr.write_lock(LOCK_KEY);
    }

    /// Create a new SLCR peripheral wrapper.
    pub fn new(slcr: zynq7000::slcr::MmioRegisters<'static>) -> Self {
        Self(slcr)
    }

    /// Unsafely create a new SLCR peripheral wrapper.
    ///
    /// # Safety
    ///
    /// This allows to create an arbitrary number of SLCR peripheral wrappers. It is the user's
    /// responsibility that these wrappers are not used concurrently in a way which leads to
    /// data races.
    pub unsafe fn steal() -> Self {
        Self::new(unsafe { zynq7000::slcr::Registers::new_mmio_fixed() })
    }

    /// Returns a mutable reference to the SLCR MMIO block.
    ///
    /// The MMIO block will not be unlocked. However, the registers can still be read.
    pub fn regs(&self) -> &MmioRegisters<'static> {
        &self.0
    }

    /// Modify the SLCR register.
    ///
    /// This method unlocks the SLCR registers and then calls a user provided function
    /// with the [SLCR MMIO][MmioSlcr] block as an input argument. This allows the user
    /// to safely modify the SLCR registers. The SLCR will be locked afte the operation.
    pub fn modify<F: FnMut(&mut MmioRegisters)>(&mut self, mut f: F) {
        self.0.write_unlock(UNLOCK_KEY);
        f(&mut self.0);
        self.0.write_lock(LOCK_KEY);
    }

    /// Manually unlock the SLCR registers.
    pub fn unlock(&mut self) {
        self.0.write_unlock(UNLOCK_KEY);
    }

    /// Manually lock the SLCR registers.
    pub fn lock(&mut self) {
        self.0.write_lock(LOCK_KEY);
    }
}
