use zynq_mmu::L1Table;

pub struct Mmu(&'static mut L1Table);

impl Mmu {
    #[inline]
    pub const fn new(table: &'static mut L1Table) -> Self {
        Mmu(table)
    }

    pub fn update_l1_table(&mut self, f: impl FnOnce(&mut L1Table)) {
        // TODO: Disable MMU
        f(self.0);
        // DSB, ISB? enable MMU again.
    }
}
