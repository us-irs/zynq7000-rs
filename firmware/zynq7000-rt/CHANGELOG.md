Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

# [v0.2.0] 2026-02-14

Bugfixes in startup assembler code.

## Changed

- `.data` initialization is skipped if it is already in place, which is usually the default
  case because it is flashed to RAM.
- Runtime now calls a `kmain` method similar to the re-export `aarch32-rt` crate.
  Former `boot_core` method must be renamed to `kmain`, but it is recommended to use
  the `zynq7000-rt::entry` proc macro to annotate the main method.
- Bumped `aarch32-rt` to v0.2 which now requires the `memory.x` file to place the `STACKS` segment

## Fixed

- Stack initialization was bugged and stack was not properly initialized for some of the
  processor modes (all modes except system mode and IRQ mode).
- MMU is enabled after the MMU table was copied (which is done in the `.data` coping step).

# [v0.1.1] 2025-10-10

Documentation fixes.

# [v0.1.0] 2025-10-09

Initial release

[unreleased]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-rt-v0.2.0...HEAD
[v0.2.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-rt-v0.1.1...zynq7000-rt-v0.2.0
[v0.1.1]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-rt-v0.1.0...zynq7000-rt-v0.1.1
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zynq7000-rt-v0.1.0
