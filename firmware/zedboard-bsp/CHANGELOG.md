Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

## Fixed

- QSPI robustness fixes. Read and fast-read operations are now chunked according to the 252 byte
  limit specified in the TRM.

## Added

- QSPI constructor can now optionally clear block protection and set latency configuration.

# [v0.1.0]

Initial release

[unreleased]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zedboard-bsp-v0.1.0...HEAD
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zedboard-bsp-v0.1.0
