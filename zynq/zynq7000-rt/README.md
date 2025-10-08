[![Crates.io](https://img.shields.io/crates/v/zynq7000-rt)](https://crates.io/crates/zynq7000-rt)
[![docs.rs](https://img.shields.io/docsrs/zynq7000-rt)](https://docs.rs/zynq7000-rt)
[![ci](https://github.com/us-irs/zynq7000-rs/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/us-irs/zynq7000-rs/actions/workflows/ci.yml)

Zynq7000 Rust Run-Time Support
========

Startup code and minimal runtime for the AMD Zynq7000 SoC to write bare metal Rust code.
This run-time crate is strongly based on the
[startup code provided by AMD](https://github.com/Xilinx/embeddedsw/blob/master/lib/bsp/standalone/src/arm/cortexa9/gcc/boot.S).

Some major differences:

- No L2 cache initialization is performed.
- MMU table is specified as Rust code.
- Modification to the stack setup code, because a different linker script is used.

This crate pulls in the [cortex-a-rt](https://github.com/us-irs/cortex-ar/tree/cortex-a-addition/cortex-a-rt)
crate to provide ARM vectors and the linker script.

## Features

- `rt` is a default feature which activates the run-time.

## Re-Generating the MMU table

The MMU table is a static flat map of 4096 entries for each 1 MB in the memory map.
It was generated using the [`mmu-table-gen`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/tools/mmu-table-gen)
tool.
