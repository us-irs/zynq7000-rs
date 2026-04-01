Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

## Fixed

- Bugfix for DDR initialization: `calibrate_iob_impedance_for_ddr3` and `calibrate_iob_impedance`
  now expect a `zynq7000::slcr::ddriob::DdrControl` input argument. This register write was
  missing
- Several bugfixes and improvements for GIC module. Some of the registers previously were
  completely overwritten instead of only modifying their own bit portions. Also allow targeting
  interrupts without clearing other CPU target.

## Changed

- `devcfg` moved to `pl` module
- Added division by zero check in gtc frequency_to_ticks to avoid runtime panic
- Increased UART type safety by providing dedicated MIO constructors for UART 0 and UART 1
  respectively.

## Added

- Method to de-assert PL reset.
- ARM clock initialization for the `ArmClocks` structure
- The `ArmClocks` structure now caches the CPU clock ratio

# [v0.1.1] 2025-10-10

Documentation fixes.

# [v0.1.0] 2025-10-09

Initial release

[unreleased]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-hal-v0.1.0...HEAD
[v0.1.1]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-hal-v0.1.0...zynq7000-hal-v0.1.1
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zynq7000-hal-v0.1.0
