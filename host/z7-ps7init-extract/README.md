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
the DDR in conjunction with the [Zynq7000 HAL library](https://github.com/us-irs/zynq7000-rs/tree/fsbl-rs/zynq/zynq7000-hal).

Right now, the script expects the `ps7init.tcl` file to be passed as a command line argument
for `-p` or `--path`. It then generates the configuration as a `ddrc_config_autogen.rs` and
`ddriob_config_autogen.rs` file.

It also generates a `ps7_ops_autogen.rs` file containing the PLL bring-up, clock control,
DDR/DDRIOB, and post-config register init sequences as ordered `RegOp` lists (`PLL_INIT_OPS`,
`CLOCK_INIT_OPS`, `DDR_INIT_OPS`, `DDRIOB_INIT_OPS`, `POST_CONFIG_OPS`), preserving the exact
write/mask-write/poll order from the source script. `POST_CONFIG_OPS` is extracted from
`ps7_post_config_3_0`: it enables the AXI level shifters and deasserts the PL reset
(`FPGA_RST_CTRL`), which is needed to bring the PL out of its power-on reset state and matches
what AMD's tooling runs right after `ps7_init` regardless of whether/when a bitstream gets
loaded. This is meant for host-side tools that talk to the target directly (e.g. over
JTAG via probe-rs) rather than for on-target firmware, since some of these registers (in
particular the PLL control registers) need to be written multiple times in sequence to bring the
hardware up correctly, not just poked with a single final value.

The RON file (`z7_init_regs.ron`) containing these op sequences is always generated and is the
format consumed by the [`z7-run`](../z7-run) host tool, which reads it to replay the PS7
(PLL/clock/DDR/post-config) init sequence directly over JTAG instead of running a `ps7_init.tcl`
script through `xsct`/`hw_server`.

For example, assuming that there is a `ps7init.tcl` script in the current directory, you can use

```sh
cargo run -- --path ./ps7init.tcl
```

to generate the configuration files.
