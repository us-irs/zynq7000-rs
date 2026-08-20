Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

## Changed

- `Interrupt::Sgi` now wraps the new `SgiInterrupt` type instead of a raw `usize`.
- Merged the `zynq7000-mmu` crate into `mmu`. The `L1Table`, `L1TableRaw` and `L1TableWrapper`
  types now live here directly, since this HAL was their only remaining consumer.
- Moved MMU table setup here from `zynq7000-rt`. The `mmu` and `mmu_table` modules and the
  `first-segment-ddr-attr` feature now live in this crate. Enabling the MMU used to happen
  unconditionally in the run-time startup assembly, it is now an explicit Rust call gated by
  the new `Config::mmu_init` flag, which `init()` runs by default. The first 1 MB segment now
  defaults to the OCM attribute rather than DDR, matching that move.

## Added

- Added the `SgiInterrupt` type-safe SGI ID wrapper, plus `Configurator::set_sgi_interrupt_priority`/
  `read_sgi_interrupt_priority`, `Configurator::trigger_software_interrupt` and
  `Configurator::set_all_sgi_interrupt_targets_cpu0`. Used by the new `multiprio` example to run
  tasks on separate `InterruptExecutor`s at distinct SGI priorities.
- Added `cache::invalidate_data_cache_range_inner`, `cache::clean_data_cache_range_inner` and
  `cache::clean_and_invalidate_data_cache_range_inner`, which perform only the L1 (inner) half
  of cache maintenance. Useful for memory the L2 doesn't cache in the first place (e.g. OCM
  mapped with an outer-non-cacheable attribute), where the L2 half of the existing combined
  functions would just be wasted work. The existing `invalidate_data_cache_range`,
  `clean_data_cache_range` and `clean_and_invalidate_data_cache_range` now reuse these new
  functions internally for their inner-cache steps instead of duplicating the loops.

## Removed

- Removed the `std` feature. `thiserror/std` needs `extern crate std`, which this crate's only
  supported target (`armv7a-none-eabihf`) can never provide, so the feature could not build.

## Fixed

- `DDR` MMU sections used domain 15, but `enable_mmu_and_cache()` only grants Manager access to
  domain 0. All sections now use domain 0, matching what's actually enabled and avoiding a domain
  fault on DDR access.
- Marked the `OCM` MMU section attribute as shareable, matching the Xilinx `ps7_init.tcl`/boot.S
  reference. Keeps the low OCM alias snoop-coherent between the two cores' L1 caches.
- Bugfix for DDR initialization: `calibrate_iob_impedance_for_ddr3` and `calibrate_iob_impedance`
  now expect a `zynq7000::slcr::ddriob::DdrControl` input argument. This register write was
  missing
- Bugfix for `cache::invalidate_data_cache_range` which did not properly invalidate inner cache
  (L1) for sizes larger than 32.
- Several bugfixes and improvements for GIC module. Some of the registers previously were
  completely overwritten instead of only modifying their own bit portions. Also allow targeting
  interrupts without clearing other CPU target.
- Do not reset the UART on TX future creation anymore, which lead to glitches and invalid data.
- Robustness improvements for the asynchronous UART TX module.
- SPI1 AMBA clock control bits are now enabled and disabled properly
- `calculate_gem_1_ref_clock` read the GEM0 clock control register instead of GEM1's.
- `FpgaClockConfig::calculate`/`calculate_generic` now return `DivisorZeroError` for a zero
  target clock instead of panicking on divide by zero.

## Changed

- HAL init now brings out the PL out of reset by default.
- HAL init configuration is now a non-exhaustive configuration structure with a `Default` impl.
- Inside the `smoltcp` ethernet TX driver, only clean the cache.
- UART TX Async and SPI Async API is `unsafe` now.
- Asynch UART and SPI operations now borrow the passed buffer for their lifetime.
- Increased reliabily of PS UART interrupt reception, which was proven to be buggy for higher baud
  rates: Force user to configure RTO value, encouraging non-zero values, and use a RX FIFO trigger
  value of FIFO depth divided by 2 by default.
- `devcfg` moved to `pl` module
- Added division by zero check in gtc frequency_to_ticks to avoid runtime panic
- Increased UART type safety by providing dedicated MIO constructors for UART 0 and UART 1
  respectively.
- `log::rb` module replaced by `log::asynch` module which uses an asynchronous embassy pipe
  for logging.
- GIC data structures: Removed the `Gic` prefix which already is part of the module name.
- Renamed `GicInterruptHelper` to `InterruptGuard`. It acknowledges the end of interrupts on drop.

## Added

- Method to de-assert PL reset.
- ARM clock initialization for the `ArmClocks` structure
- The `ArmClocks` structure now caches the CPU clock ratio
- New generic interrupt registry and generic interrupt handler which uses the registry.
  Primary interface is the `crate::generic_interrupt_handler` function and the
  `crate::register_interrupt` function.

# [v0.1.1] 2025-10-10

Documentation fixes.

# [v0.1.0] 2025-10-09

Initial release

[unreleased]: https://github.com/us-irs/zynq7000-rs/compare/zynq7000-hal-v0.1.0...HEAD
[v0.1.1]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-hal-v0.1.0...zynq7000-hal-v0.1.1
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zynq7000-hal-v0.1.0
