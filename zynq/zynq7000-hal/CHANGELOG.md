Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

## Changed

- Increased UART type safety by providing dedicated MIO constructors for UART 0 and UART 1
  respectively.
- Several bugfixes and improvements for GIC module. Some of the registers previously were
  completely overwritten instead of only modifying their own bit portions. Also allow targeting
  interrupts without clearing other CPU target.

# [v0.1.1] 2025-10-10

Documentation fixes.

# [v0.1.0] 2025-10-09

Initial release

[unreleased]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-hal-v0.1.0...HEAD
[v0.1.1]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-hal-v0.1.0...zynq7000-hal-v0.1.1
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zynq7000-hal-v0.1.0
