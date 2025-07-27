use arbitrary_int::{u2, u3, u4, u6};

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

#[repr(transparent)]
pub struct Control(u32);

impl Control {
    pub fn new_enabled() -> Self {
        Self(0x1)
    }

    pub fn new_disabled() -> Self {
        Self(0x0)
    }

    #[inline(always)]
    pub fn enabled(&mut self) -> bool {
        self.0 == 0x1
    }
}

#[bitbybit::bitenum(u1, exhaustive = true)]
pub enum ReplacementPolicy {
    PseudoRandomWithLfsr = 0,
    RoundRobin = 1,
}

#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Default, Debug)]
pub enum WaySize {
    __Reserved0 = 0b000,
    _16kB = 0b001,
    _32kB = 0b010,
    #[default]
    _64kB = 0b011,
    _128kB = 0b100,
    _256kB = 0b101,
    _512kB = 0b110,
    __Reserved1 = 0b111,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Default, Debug)]
pub enum Associativity {
    #[default]
    _8Way = 0,
    _16Way = 1,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct AuxControl {
    #[bit(30, rw)]
    early_bresp_enable: bool,
    #[bit(29, rw)]
    isntruction_prefetch_enable: bool,
    #[bit(28, rw)]
    data_prefetch_enable: bool,
    #[bit(27, rw)]
    nonsec_interrupt_access_control: bool,
    #[bit(26, rw)]
    nonsec_lockdown_enable: bool,
    #[bit(25, rw)]
    cache_replace_policy: ReplacementPolicy,
    #[bits(23..=24, rw)]
    force_write_alloc: u2,
    #[bit(22, rw)]
    shared_attr_override: bool,
    #[bit(21, rw)]
    parity_enable: bool,
    #[bit(20, rw)]
    event_monitor_bus_enable: bool,
    #[bits(17..=19, rw)]
    way_size: WaySize,
    #[bit(16, rw)]
    associativity: Associativity,
    #[bit(13, rw)]
    shared_attribute_invalidate: bool,
    #[bit(12, rw)]
    exclusive_cache_config: bool,
    #[bit(11, rw)]
    store_buff_device_limitation_enable: bool,
    #[bit(10, rw)]
    high_priority_so_dev_reads: bool,
    /// Disabled by default.
    #[bit(0, rw)]
    full_line_zero_enable: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug, PartialEq, Eq)]
pub struct LatencyConfig {
    /// Latency is the numerical value + 1 cycles.
    #[bits(8..=10, rw)]
    write_access_latency: u3,
    /// Latency is the numerical value + 1 cycles.
    #[bits(4..=6, rw)]
    read_access_latency: u3,
    /// Latency is the numerical value + 1 cycles.
    #[bits(0..=2, rw)]
    setup_latency: u3,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct InterruptStatus {
    #[bit(8, r)]
    dec_error_l3: bool,
    #[bit(7, r)]
    slave_error_l3: bool,
    #[bit(6, r)]
    error_data_ram_read: bool,
    #[bit(5, r)]
    error_tag_ram_read: bool,
    #[bit(4, r)]
    error_data_ram_write: bool,
    #[bit(3, r)]
    error_tag_ram_write: bool,
    #[bit(2, r)]
    parity_error_data_ram_read: bool,
    #[bit(1, r)]
    parity_error_tag_ram_read: bool,
    /// ECNTR
    #[bit(0, r)]
    event_counter_overflow_increment: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct InterruptControl {
    #[bit(8, w)]
    dec_error_l3: bool,
    #[bit(7, w)]
    slave_error_l3: bool,
    #[bit(6, w)]
    error_data_ram_read: bool,
    #[bit(5, w)]
    error_tag_ram_read: bool,
    #[bit(4, w)]
    error_data_ram_write: bool,
    #[bit(3, w)]
    error_tag_ram_write: bool,
    #[bit(2, w)]
    parity_error_data_ram_read: bool,
    #[bit(1, w)]
    parity_error_tag_ram_read: bool,
    /// ECNTR
    #[bit(0, w)]
    event_counter_overflow_increment: bool,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct L2Cache {
    #[mmio(PureRead)]
    cache_id: CacheId,
    #[mmio(PureRead)]
    cache_type: u32,

    _reserved: [u32; 0x3E],

    control: Control,
    aux_control: AuxControl,
    tag_ram_latency: LatencyConfig,
    data_ram_latency: LatencyConfig,

    _reserved2: [u32; 0x3C],

    event_counter_control: u32,
    event_counter_1_config: u32,
    event_counter_0_config: u32,
    event_counter_1: u32,
    event_counter_0: u32,
    interrupt_mask: u32,
    #[mmio(PureRead)]
    interrupt_mask_status: InterruptStatus,
    #[mmio(PureRead)]
    interrupt_raw_status: InterruptStatus,
    #[mmio(Write)]
    interrupt_clear: InterruptControl,

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
