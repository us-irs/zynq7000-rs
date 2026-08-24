use arbitrary_int::{u2, u3, u4, u6};

/// Base address of the L2 cache controller register block.
pub const L2C_BASE_ADDR: usize = 0xF8F0_2000;

/// Data and instruction lockdown registers for one CPU, restricting which cache ways it may
/// allocate into.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct LockdownRegisters {
    /// Data lockdown mask.
    data: u32,
    /// Instruction lockdown mask.
    instruction: u32,
}

/// Cache Sync register, used to drain the store buffer and wait for background operations.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct CacheSync {
    /// A cache maintenance or synchronization operation is still in progress.
    #[bit(0, r)]
    busy: bool,
}

/// Debug Control register, used to disable cache linefills and write-backs for debugging.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct DebugControl {
    /// Secure privileged non-invasive debug enable.
    #[bit(2, rw)]
    spniden: bool,
    /// Forces all writes to become write-through instead of write-back.
    #[bit(1, rw)]
    disable_write_back: bool,
    /// Forces linefills to be disabled, treating all accesses as cache misses.
    #[bit(0, rw)]
    disable_cache_linefill: bool,
}

/// Cache ID register, identifying the implementer and revision of the cache controller.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct CacheId {
    /// Implementer code.
    #[bits(24..=31, r)]
    implementer: u8,
    /// Cache ID number.
    #[bits(10..=15, r)]
    cache_id: u6,
    /// Part number.
    #[bits(6..=9, r)]
    part_number: u4,
    /// RTL release number.
    #[bits(0..=5, r)]
    rtl_release: u6,
}

/// Control register, used to enable or disable the L2 cache.
#[repr(transparent)]
pub struct Control(u32);

impl Control {
    /// Creates a control value with the cache enabled.
    pub fn new_enabled() -> Self {
        Self(0x1)
    }

    /// Creates a control value with the cache disabled.
    pub fn new_disabled() -> Self {
        Self(0x0)
    }

    /// Whether the cache is enabled.
    #[inline(always)]
    pub fn enabled(&mut self) -> bool {
        self.0 == 0x1
    }
}

/// Cache replacement policy used to select victim ways on eviction.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReplacementPolicy {
    /// Pseudo-random replacement using a linear feedback shift register.
    PseudoRandomWithLfsr = 0,
    /// Round-robin replacement.
    RoundRobin = 1,
}

/// Cache way size, configuring the size of each cache way.
#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Default, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WaySize {
    /// Reserved.
    __Reserved0 = 0b000,
    /// 16 kB way size.
    _16kB = 0b001,
    /// 32 kB way size.
    _32kB = 0b010,
    /// 64 kB way size.
    #[default]
    _64kB = 0b011,
    /// 128 kB way size.
    _128kB = 0b100,
    /// 256 kB way size.
    _256kB = 0b101,
    /// 512 kB way size.
    _512kB = 0b110,
    /// Reserved.
    __Reserved1 = 0b111,
}

/// Cache associativity, configuring the number of ways per set.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Default, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Associativity {
    /// 8-way associative.
    #[default]
    _8Way = 0,
    /// 16-way associative.
    _16Way = 1,
}

/// Auxiliary Control register, configuring cache organization and behavior.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct AuxControl {
    /// Enables early BRESP for cacheable write transactions.
    #[bit(30, rw)]
    early_bresp_enable: bool,
    /// Enables instruction prefetching.
    #[bit(29, rw)]
    isntruction_prefetch_enable: bool,
    /// Enables data prefetching.
    #[bit(28, rw)]
    data_prefetch_enable: bool,
    /// Controls whether Non-secure accesses can access interrupt registers.
    #[bit(27, rw)]
    nonsec_interrupt_access_control: bool,
    /// Controls whether Non-secure accesses can access lockdown registers.
    #[bit(26, rw)]
    nonsec_lockdown_enable: bool,
    /// Selects the cache replacement policy.
    #[bit(25, rw)]
    cache_replace_policy: ReplacementPolicy,
    /// Force write allocate configuration.
    #[bits(23..=24, rw)]
    force_write_alloc: u2,
    /// Overrides the shareable attribute of transactions.
    #[bit(22, rw)]
    shared_attr_override: bool,
    /// Enables parity checking.
    #[bit(21, rw)]
    parity_enable: bool,
    /// Enables the event monitor bus export.
    #[bit(20, rw)]
    event_monitor_bus_enable: bool,
    /// Selects the way size.
    #[bits(17..=19, rw)]
    way_size: WaySize,
    /// Selects the cache associativity.
    #[bit(16, rw)]
    associativity: Associativity,
    /// Enables the shared attribute invalidate feature.
    #[bit(13, rw)]
    shared_attribute_invalidate: bool,
    /// Enables exclusive cache configuration with the L1 cache.
    #[bit(12, rw)]
    exclusive_cache_config: bool,
    /// Limits the number of outstanding store buffer entries for Device/Strongly-Ordered
    /// accesses.
    #[bit(11, rw)]
    store_buff_device_limitation_enable: bool,
    /// Gives high priority to Strongly-Ordered and Device reads.
    #[bit(10, rw)]
    high_priority_so_dev_reads: bool,
    /// Disabled by default.
    #[bit(0, rw)]
    full_line_zero_enable: bool,
}

/// RAM latency configuration for tag or data RAM accesses.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
#[derive(PartialEq, Eq)]
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

/// Masked or raw interrupt status, indicating pending L2 cache interrupt sources.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct InterruptStatus {
    /// AXI decode error on the L3 (dedicated) port.
    #[bit(8, r)]
    dec_error_l3: bool,
    /// AXI slave error on the L3 (dedicated) port.
    #[bit(7, r)]
    slave_error_l3: bool,
    /// Error on a data RAM read.
    #[bit(6, r)]
    error_data_ram_read: bool,
    /// Error on a tag RAM read.
    #[bit(5, r)]
    error_tag_ram_read: bool,
    /// Error on a data RAM write.
    #[bit(4, r)]
    error_data_ram_write: bool,
    /// Error on a tag RAM write.
    #[bit(3, r)]
    error_tag_ram_write: bool,
    /// Parity error on a data RAM read.
    #[bit(2, r)]
    parity_error_data_ram_read: bool,
    /// Parity error on a tag RAM read.
    #[bit(1, r)]
    parity_error_tag_ram_read: bool,
    /// ECNTR
    #[bit(0, r)]
    event_counter_overflow_increment: bool,
}

/// Write-to-clear interrupt control, used to clear raw interrupt status bits.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
#[derive(Debug)]
pub struct InterruptControl {
    /// AXI decode error on the L3 (dedicated) port.
    #[bit(8, w)]
    dec_error_l3: bool,
    /// AXI slave error on the L3 (dedicated) port.
    #[bit(7, w)]
    slave_error_l3: bool,
    /// Error on a data RAM read.
    #[bit(6, w)]
    error_data_ram_read: bool,
    /// Error on a tag RAM read.
    #[bit(5, w)]
    error_tag_ram_read: bool,
    /// Error on a data RAM write.
    #[bit(4, w)]
    error_data_ram_write: bool,
    /// Error on a tag RAM write.
    #[bit(3, w)]
    error_tag_ram_write: bool,
    /// Parity error on a data RAM read.
    #[bit(2, w)]
    parity_error_data_ram_read: bool,
    /// Parity error on a tag RAM read.
    #[bit(1, w)]
    parity_error_tag_ram_read: bool,
    /// ECNTR
    #[bit(0, w)]
    event_counter_overflow_increment: bool,
}

/// L2 Cache register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Cache ID register.
    #[mmio(PureRead)]
    cache_id: CacheId,
    /// Cache type register.
    #[mmio(PureRead)]
    cache_type: u32,

    _reserved: [u32; 0x3E],

    /// Control register.
    control: Control,
    /// Auxiliary control register.
    aux_control: AuxControl,
    /// Tag RAM latency control register.
    tag_ram_latency: LatencyConfig,
    /// Data RAM latency control register.
    data_ram_latency: LatencyConfig,

    _reserved2: [u32; 0x3C],

    /// Event counter control register.
    event_counter_control: u32,
    /// Event counter 1 configuration register.
    event_counter_1_config: u32,
    /// Event counter 0 configuration register.
    event_counter_0_config: u32,
    /// Event counter 1 value register.
    event_counter_1: u32,
    /// Event counter 0 value register.
    event_counter_0: u32,
    /// Interrupt mask register.
    interrupt_mask: u32,
    /// Masked interrupt status register.
    #[mmio(PureRead)]
    interrupt_mask_status: InterruptStatus,
    /// Raw interrupt status register.
    #[mmio(PureRead)]
    interrupt_raw_status: InterruptStatus,
    /// Interrupt clear register.
    #[mmio(Write)]
    interrupt_clear: InterruptControl,

    _reserved3: [u32; 0x143],

    /// Cache sync register.
    cache_sync: CacheSync,

    _reserved4: [u32; 0xF],

    /// Invalidate line by physical address register.
    invalidate_by_pa: u32,

    _reserved5: [u32; 0x2],

    /// Invalidate by way register.
    invalidate_by_way: u32,

    _reserved6: [u32; 0xC],

    /// Clean line by physical address register.
    clean_by_pa: u32,

    _reserved7: u32,

    /// Clean line by index register.
    clean_by_index: u32,
    /// Clean by way register.
    clean_by_way: u32,

    _reserved8: [u32; 0xC],

    /// Clean and invalidate line by physical address register.
    clean_invalidate_by_pa: u32,

    _reserved9: u32,

    /// Clean and invalidate line by index register.
    clean_invalidate_by_index: u32,
    /// Clean and invalidate by way register.
    clean_invalidate_by_way: u32,

    _reserved10: [u32; 0x40],

    /// Data and instruction lockdown registers, one pair per CPU.
    #[mmio(Inner)]
    lockdown_regs: [LockdownRegisters; 8],

    _reserved11: [u32; 0x4],

    /// Lockdown by line enable register.
    lockdown_by_line_enable: u32,
    /// Unlock way register.
    unlock_way: u32,

    _reserved12: [u32; 0xAA],

    /// Address filtering start register.
    addr_filtering_start: u32,
    /// Address filtering end register.
    addr_filtering_end: u32,

    _reserved13: [u32; 0xCE],

    /// Debug control register.
    debug_control: DebugControl,

    _reserved14: [u32; 0x7],

    /// Prefetch control register.
    prefetch_control: u32,

    _reserved15: [u32; 0x7],

    /// Power control register.
    power_control: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0xF84);

impl Registers {
    /// Create a new L2C MMIO instance for for L2 Cache at address [L2C_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(L2C_BASE_ADDR) }
    }
}
