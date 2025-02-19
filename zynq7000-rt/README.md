Zynq7000 Rust Run-Time Support
========

Startup code and minimal runtime for the AMD Zynq7000 SoC to write bare metal Rust code.
This run-time crate is strongly based on the
[startup code provided by AMD](https://github.com/Xilinx/embeddedsw/blob/master/lib/bsp/standalone/src/arm/cortexa9/gcc/boot.S).

One major difference is that the MMU table is specified as Rust code. There are also modification
to the stack setup code, because a different linker script is used.

This crate pulls in the [cortex-a-rt](https://github.com/us-irs/cortex-ar/tree/cortex-a-addition/cortex-a-rt)
crate to provide ARM vectors and the linker script.

## Features

- `rt` is a default feature which activates the run-time.

## Re-Generating the MMU table

The MMU table is a static flat map of 4096 entries for each 1 MB in the memory map.
It was generated using the `table-gen` binary tool.

You can re-run the tool using

```sh
cargo +stable --target <hostTarget> run --bin table-gen --no-default-features --features tools
```
