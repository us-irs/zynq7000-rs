Zynq 7000 Bare-Metal Rust Support
=========

This crate collection provides support to write bare-metal Rust applications for the AMD Zynq 7000
family of SoCs.

# List of crates

This project contains the following crates:

## [Firmware Workspace](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware)

This workspace contains libraries and application which can only be run on the target system.

- The [`zynq7000-rt`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zynq7000-rt)
  run-time crate containing basic low-level startup code necessary to boot a Rust app on the
  Zynq7000.
- The [`zynq7000`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zynq7000) PAC
  crate containing basic low-level register definitions.
- The [`zynq7000-mmu`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zynq7000-hal)
  crate containing common MMU abstractions used by both the HAL and the run-time crate.
- The [`zynq7000-hal`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zynq7000-hal)
  HAL crate containing higher-level abstractions on top of the PAC register crate.
- The [`zynq7000-embassy`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zynq7000-embassy)
  crate containing support for running the embassy-rs asynchronous run-time.

This project was developed using a Zedboard, so there are several crates available targeted towards
this board:

- The [`zedboard-bsp`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zedboard-bsp)
  crate containing board specific components for the Zedboard.
- The [`zedboard-fsbl`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zedboard-fsbl)
  contains a simple first-stage bootloader application for the Zedboard.
- The [`zedboard-qspi-flasher`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/zedboard-qspi-flasher)
  contains an application which is able to flash a boot binary from DDR to the QSPI.

It also contains the following helper crates:

- The [`examples`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/firmware/examples)
  folder contains various example applications crates using the HAL and the PAC.
  This folder also contains dedicated example applications using the
  [`embassy`](https://github.com/embassy-rs/embassy) native Rust RTOS.

## Other libraries and tools

- The [`zedboard-fpga-design`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zedboard-fpga-design)
  folder contains a sample FPGA design and block design which was used in some of the provided software examples. The project was created with Vivado version 2024.1.
  The folder contains a README with all the steps required to load this project from a TCL script.
- The [`zynq7000-boot-image`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/host/zynq7000-boot-image)
  library contains generic helpers to interface with the AMD
  [boot binary](https://docs.amd.com/r/en-US/ug1283-bootgen-user-guide).
- The [`tools/zynq7000-ps7init-extract`](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/host/zynq7000-ps7init-extract)
  tool allows extracting configuration from the AMD generated `ps7init.tcl` file which contains
  static configuration parameters for DDR initialization.

# Using the `.cargo/config.toml` file

This is mostly relevant for development directly inside this repostiory.
Use the following command to have a starting `config.toml` file

```sh
cp .cargo/config.toml.template .cargo/config.toml
```

You then can adapt the `config.toml` to your needs. For example, you can configure runners
to conveniently flash with `cargo run`.

# Using the sample VS Code files

Use the following command to have a starting configuration for VS Code:

```sh
cp -rT vscode .vscode
```

You can then adapt the files in `.vscode` to your needs.

# Building the blinky example

Assuming you have the following segments inside your `.cargo/config.toml`

```toml
[target.armv7a-none-eabihf]
rustflags = [
  "-Ctarget-cpu=cortex-a9",
  "-Ctarget-feature=+vfp3",
  "-Ctarget-feature=+neon",
  "-Clink-arg=-Tlink.x",
  # If this is not enabled, debugging / stepping can become problematic.
  "-Cforce-frame-pointers=yes",
  # Can be useful for debugging.
  # "-Clink-args=-Map=app.map"
]

[build]
target = "armv7a-none-eabihf"
```

You can build the blinky example app using

```sh
cd zynq
cargo build --bin blinky
```


## Flashing, running and debugging the software

This repository was only tested with the [Zedboard](https://digilent.com/reference/programmable-logic/zedboard/start)
but should be easily adaptable to other Zynq7000 based platforms and boards.

If you want to test and use this crate on a Zedboard, make sure that the board jumpers on the
Zedboard are configured for JTAG boot.

### Pre-Requisites

- [`xsct`](https://docs.amd.com/r/en-US/ug1165-zynq-embedded-design-tutorial/XSCT-Xilinx-Software-Command-Tool)
  tool installation. You have to install the [Vitis tool](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis.html)
  to get access to this tool. You can minimize the download size by de-selecting all SoC families
  that you do not require. Vitis also includes a Vivado installation which is required to do
  anything with the FPGA.
- [`hw_server`](https://docs.amd.com/r/en-US/ug908-vivado-programming-debugging/Connecting-to-a-Hardware-Target-Using-hw_server)
  installation which is generally included with Vitis/Vivado. This tool starts a GDB server
  which will be used to connect to and debug the target board. It also allows initializing the
  processing system and uploading the FPGA design via JTAG.
- A valid `ps7_init.tcl` script to configure the processing system. This is necessary to even
  be able flashing application into the DDR via JTAG. There are multiple ways to generate this
  startup script, but the recommended way here is to the the `sdtgen` tool included in `xsct` which
  also generates a bitstream for the FPGA configuration. You can find a sample `ps7_init.tcl`
  script inside the `scripts` folder. However, it is strongly recommended to get familiar with
  Vivado and generate the SDT folder yourself.
- `gdb-multiarch` installation to debug applications.
- `python3` installation to use the provided tooling.

### Programming and Debug Flow

1. Start the `hw_server` application first. This is required for other tooling provided by this
   repository as well.

2. The provided `scripts/zynq7000-init.py` script can be used to initialize the processing system
   with the `ps7_init.tcl` script, program the bitstream, and load an ELF file to the DDR.
   You can run `scripts/zynq7000-init.py` to get some help and additional information.
   Here is an example command to initialize the processing system without loading a bitstream
   using the provided initialization script (adapt `AMD_TOOLS` to your system):

   ```sh
   export AMD_TOOLS="/tools/Xilinx/Vitis/2024.1"
   ./scripts/zynq7000-init.py --itcl ./scripts/ps7_init.tcl
   ```

3.  Assuming you have managed to build the blinky example, you can flash the example now
    using GDB:

    ```sh
    gdb-multiarch -q -x gdb.gdb target/armv7a-none-eabihf/debug/blinky
    ```

    You can use the `-tui` argument to also have a terminal UI.
    This repository provides a `scripts/runner.sh` which performs all the steps specified above.
    The `.cargo/def-config.toml` script contains the runner and some template environmental
    variables that need to be set for this to work. The command above also loaded the app, but
    this task can be performed by the `zynq7000-init.py` wrapper as well.

# Using VS Code

The provided VS Code configuration files can be used to perform the CLI steps specified above
in a GUI completely and to also have a graphical debugging interface. You can use this
as a starting point for your own application. You need to adapt the `options.env` variables
in `.vscode/tasks.json` for this to work.

# Embedded Rust

If you have not done this yet, it is recommended to read some of the excellent resources available
to learn Rust:

- [Rust Embedded Book](https://docs.rust-embedded.org/book/)
- [Rust Discovery Book](https://docs.rust-embedded.org/discovery/)
