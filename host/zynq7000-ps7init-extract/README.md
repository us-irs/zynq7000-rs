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
`ddrc_config_autogen.rs` file.

For example, assuming that there is a `ps7init.tcl` script in the current directory, you can use

```sh
cargo run -- --path ./ps7init.tcl
```

to generate the configuration files.
