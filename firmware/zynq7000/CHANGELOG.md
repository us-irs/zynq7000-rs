Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

# [v0.6.0] 2026-08-25

- Added CAN controller (`can`) register definitions.
- Added DMA controller (`dmac`) register definitions.
- Added USB controller (`usb`) register definitions.
- Added System Watchdog Timer (`swdt`) register definitions.
- Restructured register helper types into a `pub mod types` submodule across most modules,
  re-exported at the module root so existing paths keep working.
- Fixed overlapping bit positions for `rx_overflow` and `tx_busy` in the I2C `Status` register.
  `rx_overflow` now correctly uses bit 7 instead of bit 6.
- AXI-HP registers were not included properly

# [v0.5.0] 2026-08-20

- `InterruptProcessorTargetRegister` now has a `default = 0x0` value and uses `bitbybit`'s
  `defmt_bitfields` attribute instead of a manual `defmt::Format` derive.

# [v0.4.0] 2026-05-15

- Better names for `uart` registers and register fields. Replaced various abbreviations.
- Update `gic` module: Add some better type for SGIR field.

# [v0.3.0] 2026-05-08

- Better names for various registers. Replaced abbreviations like SR, MR, CR, IER, IMR etc.

# [v0.2.0] 2026-04-01

- Renamed all register blocks to `Registers` to subblocks to `<Subblock>Registers`.
- Updated IPTR registers in the GIC module to use a custom register type instead of a raw u32.
- Added SDIO registers.
- Fixed wrong position in QSPI reset register in SLCR Module
- Added some missing reset register definitions.
- Added `defmt` support
- Some other minor renaming of registers (e.g. `ctrl` replaced by `control`)

# [v0.1.1] 2025-10-09

Documentation fix

# [v0.1.0] 2025-10-08

Initial release

[unreleased]: https://github.com/us-irs/zynq7000-rs/compare/zynq7000-v0.6.0...HEAD
[v0.6.0]: https://github.com/us-irs/zynq7000-rs/compare/zynq7000-v0.5.0...zynq7000-v0.6.0
[v0.5.0]: https://github.com/us-irs/zynq7000-rs/tags/zynq7000-v0.5.0
[v0.4.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-v0.3.0...zynq7000-v0.4.0
[v0.3.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-v0.2.0...zynq7000-v0.3.0
[v0.2.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-v0.1.0...zynq7000-v0.2.0
[v0.1.1]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-v0.1.0...zynq7000-v0.1.1
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zynq7000-v0.1.0
