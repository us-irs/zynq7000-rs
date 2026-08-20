Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

# [v0.2.0] 2026-08-20

## Fixed

- QSPI robustness fixes. Read, fast-read and write operations are now chunked according to the 252
  byte limit specified in the TRM.

## Added

- QSPI constructor can now optionally clear block protection and set latency configuration.

## Changed

- Alignment rules of Spansion QSPI page program now only require 4 byte aligned size.

# [v0.1.0]

Initial release

[unreleased]: https://github.com/us-irs/zynq7000-rs/compare/zedboard-bsp-v0.2.0...HEAD
[v0.2.0]: https://github.com/us-irs/zynq7000-rs/tags/zedboard-bsp-v0.2.0
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zedboard-bsp-v0.1.0
