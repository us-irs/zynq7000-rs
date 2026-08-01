Zynq7000 PS7 Init Extractor
=========

AMD provides tooling to auto-generate some of the hardware initialization for the external DDR
as native Rust code.

The AMD tooling generates these files as `ps7init.tcl`, `ps7init.c`, `ps7init.h` files but not as
Rust files. The specific parameters required for different DDR chips are proprietary, so that
portion is required for Rust programs as well. Do avoid the need of compiling the PS7 initialization
scripts with a C compiler, this tool extracts all required configuration parameters for DDR and
DDRIOB initialization and configuration and exports them as native Rust constants.

The generates files can be placed in individual projects or board support packages to initialize
the DDR in conjunction with the [Zynq7000 HAL library](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/fsbl-rs/zynq/zynq7000-hal).

Right now, the script expects the `ps7init.tcl` file to be passed as a command line argument
for `-p` or `--path`. It then generates the configuration as a `ddrc_config_autogen.rs` and
`ddriob_config_autogen.rs` file.

It also generates a `ps7_ops_autogen.rs` file containing the PLL bring-up, clock control, and
DDR/DDRIOB register init sequences as ordered `RegOp` lists (`PLL_INIT_OPS`, `CLOCK_INIT_OPS`,
`DDR_INIT_OPS`, `DDRIOB_INIT_OPS`), preserving the exact write/mask-write/poll order from the
source script. This is meant for host-side tools that talk to the target directly (e.g. over
JTAG via probe-rs) rather than for on-target firmware, since some of these registers (in
particular the PLL control registers) need to be written multiple times in sequence to bring the
hardware up correctly, not just poked with a single final value.

For example, assuming that there is a `ps7init.tcl` script in the current directory, you can use

```sh
cargo run -- --path ./ps7init.tcl
```

to generate the configuration files.
