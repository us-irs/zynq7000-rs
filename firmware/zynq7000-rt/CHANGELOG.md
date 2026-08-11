Change Log
=======

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

# [unreleased]

## Changed

- Changed default mapping of first 1 MB segment to use OCM attributes.

## Added

- Added `first-segment-ddr-attr` which can be used to use DDR attributes for the first segment
  to allow accessing DDR through address 0x8000 to 0x10_0000
- Added `z7link.x`, a complete linker script shipped by this crate which supersedes
  `aarch32-rt`'s `link.x` (projects now pass `-Clink-arg=-Tz7link.x` instead of `-Tlink.x`). It
  is a copy of `aarch32-rt/link.x` with two new output sections, `.ocm.data` and `.ocm.bss`,
  spliced in alongside the regular `.data`/`.bss`.
- Added `.ocm.data`/`.ocm.bss` output sections (in `z7link.x`) and matching startup
  initialization (in `rt::rt`): statics tagged with `#[unsafe(link_section = ".ocm.data")]` or
  `".ocm.bss"` are placed in the `OCM` region of `memory.x` and initialized exactly like regular
  `.data`/`.bss` (copied from their load address / zeroed respectively) before `kmain` runs. See
  the crate-level docs for a usage example and the `zedboard` `axi-dma` example for a real one.
  Projects that don't use OCM placement just need `OCM` aliased to any existing region in their
  `memory.x` (e.g. `REGION_ALIAS("OCM", DATA);`); nothing gets placed there so the sections are
  zero-sized.

## Fixed

- The `OCM` MMU section attribute used an incorrect inner cache policy
  (`WriteThroughNoWriteAlloc`), which caused instability for code, data and stacks actually
  running from OCM (observed as an OCM-only project failing to boot). Changed to
  `WriteBackWriteAlloc`, matching the "Normal inner write-back cacheable" attribute documented
  for OCM in the Zynq TRM.

# [v0.3.0] 2026-05-08

Bumped `aarch32-rt` and `aarch32-cpu` to v0.3.

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

[unreleased]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-rt-v0.3.0...HEAD
[v0.3.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-rt-v0.2.0...zynq7000-rt-v0.3.0
[v0.2.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-rt-v0.1.1...zynq7000-rt-v0.2.0
[v0.1.1]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/compare/zynq7000-rt-v0.1.0...zynq7000-rt-v0.1.1
[v0.1.0]: https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/tag/zynq7000-rt-v0.1.0
