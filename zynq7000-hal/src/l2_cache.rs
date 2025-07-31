use core::sync::atomic::compiler_fence;

use arbitrary_int::{u2, u3};
pub use zynq7000::l2_cache::LatencyCfg;
use zynq7000::l2_cache::{
    Associativity, AuxCtrl, Ctrl, InterruptCtrl, MmioL2Cache, ReplacementPolicy, WaySize,
};

use crate::slcr::Slcr;

/// This is the default configuration used by Xilinx/AMD.
pub const AUX_CTRL_DEFAULT: AuxCtrl = AuxCtrl::builder()
    .with_early_bresp_enable(true)
    .with_isntruction_prefetch_enable(true)
    .with_data_prefetch_enable(true)
    .with_nonsec_interrupt_access_control(false)
    .with_nonsec_lockdown_enable(false)
    .with_cache_replace_policy(ReplacementPolicy::RoundRobin)
    .with_force_write_alloc(u2::new(0))
    .with_shared_attr_override(false)
    .with_parity_enable(true)
    .with_event_monitor_bus_enable(true)
    .with_way_size(WaySize::_64kB)
    .with_associativity(Associativity::_8Way)
    .with_shared_attribute_invalidate(false)
    .with_exclusive_cache_config(false)
    .with_store_buff_device_limitation_enable(false)
    .with_high_priority_so_dev_reads(false)
    .with_full_line_zero_enable(false)
    .build();

/// Xilinx/AMD default configuration. 2 cycles for setup, write and read.
pub const DEFAULT_TAG_RAM_LATENCY: LatencyCfg = LatencyCfg::builder()
    .with_write_access_latency(u3::new(0b001))
    .with_read_access_latency(u3::new(0b001))
    .with_setup_latency(u3::new(0b001))
    .build();

/// Xilinx/AMD default configuration. 2 cycles for setup and write, 3 cycles for read.
pub const DEFAULT_DATA_RAM_LATENCY: LatencyCfg = LatencyCfg::builder()
    .with_write_access_latency(u3::new(0b001))
    .with_read_access_latency(u3::new(0b010))
    .with_setup_latency(u3::new(0b001))
    .build();

// SLCR L2C ram configuration.
pub const SLCR_L2C_CONFIG_MAGIC_VALUE: u32 = 0x00020202;

/// Similar to [init], but uses Xilinx/AMD defaults for the latency configurations.
pub fn init_with_defaults(l2c_mmio: &mut MmioL2Cache<'static>) {
    init(l2c_mmio, DEFAULT_TAG_RAM_LATENCY, DEFAULT_DATA_RAM_LATENCY);
}

/// Generic intializer function for the L2 cache.
///
/// This function is based on the initialization sequence specified in the TRM p.94 and on
/// the runtime initialization provided by Xilinx/AMD.
pub fn init(
    l2c_mmio: &mut MmioL2Cache<'static>,
    tag_ram_latency: LatencyCfg,
    data_ram_latency: LatencyCfg,
) {
    l2c_mmio.write_control(Ctrl::new_disabled());
    l2c_mmio.write_aux_control(AUX_CTRL_DEFAULT);
    l2c_mmio.write_tag_ram_latency(tag_ram_latency);
    l2c_mmio.write_data_ram_latency(data_ram_latency);

    // Invalidate the whole cache.
    l2c_mmio.write_clean_invalidate_by_way(0xffff);
    while l2c_mmio.read_cache_sync().busy() {}
    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    let pending = l2c_mmio.read_interrupt_raw_status();
    l2c_mmio.write_interrupt_clear(InterruptCtrl::new_with_raw_value(pending.raw_value()));
    unsafe {
        Slcr::with(|slcr| {
            slcr.write_magic_l2c_register(SLCR_L2C_CONFIG_MAGIC_VALUE);
        });
    }
    l2c_mmio.write_control(Ctrl::new_enabled());
}
