Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

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

[unreleased]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-v0.2.0...HEAD
[v0.2.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-v0.1.0...zynq7000-v0.2.0
[v0.1.1]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-v0.1.0...zynq7000-v0.1.1
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/tag/zynq7000-v0.1.0
