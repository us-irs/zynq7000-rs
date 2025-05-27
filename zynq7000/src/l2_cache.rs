use arbitrary_int::{u4, u6};

pub const L2C_BASE_ADDR: usize = 0xF8F0_2000;

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct LockdownRegisters {
    data: u32,
    instruction: u32,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct CacheSync {
    #[bit(0, r)]
    busy: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct DebugControl {
    #[bit(2, rw)]
    spniden: bool,
    #[bit(1, rw)]
    disable_write_back: bool,
    #[bit(0, rw)]
    disable_cache_linefill: bool,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct CacheId {
    #[bits(24..=31, r)]
    implementer: u8,
    #[bits(10..=15, r)]
    cache_id: u6,
    #[bits(6..=9, r)]
    part_number: u4,
    #[bits(0..=5, r)]
    rtl_release: u6,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct L2Cache {
    #[mmio(PureRead)]
    cache_id: CacheId,
    #[mmio(PureRead)]
    cache_type: u32,

    _reserved: [u32; 0x3E],

    control: u32,
    aux_control: u32,
    tag_ram_control: u32,
    data_ram_control: u32,

    _reserved2: [u32; 0x3C],

    event_counter_control: u32,
    event_counter_1_config: u32,
    event_counter_0_config: u32,
    event_counter_1: u32,
    event_counter_0: u32,
    interrupt_mask: u32,
    #[mmio(PureRead)]
    interrupt_mask_status: u32,
    #[mmio(PureRead)]
    interrupt_raw_status: u32,
    #[mmio(Write)]
    interrupt_clear: u32,

    _reserved3: [u32; 0x143],

    cache_sync: CacheSync,

    _reserved4: [u32; 0xF],

    invalidate_by_pa: u32,

    _reserved5: [u32; 0x2],

    invalidate_by_way: u32,

    _reserved6: [u32; 0xC],

    clean_by_pa: u32,

    _reserved7: u32,

    clean_by_index: u32,
    clean_by_way: u32,

    _reserved8: [u32; 0xC],

    clean_invalidate_by_pa: u32,

    _reserved9: u32,

    clean_invalidate_by_index: u32,
    clean_invalidate_by_way: u32,

    _reserved10: [u32; 0x40],

    #[mmio(Inner)]
    lockdown_regs: [LockdownRegisters; 8],

    _reserved11: [u32; 0x4],

    lockdown_by_line_enable: u32,
    unlock_way: u32,

    _reserved12: [u32; 0xAA],

    addr_filtering_start: u32,
    addr_filtering_end: u32,

    _reserved13: [u32; 0xCE],

    debug_control: DebugControl,

    _reserved14: [u32; 0x7],

    prefetch_control: u32,

    _reserved15: [u32; 0x7],

    power_control: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<L2Cache>(), 0xF84);

impl L2Cache {
    /// Create a new L2C MMIO instance for for L2 Cache at address [I2C_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioL2Cache<'static> {
        unsafe { Self::new_mmio_at(L2C_BASE_ADDR) }
    }
}
