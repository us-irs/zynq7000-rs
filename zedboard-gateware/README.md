Zedboard FPGA design for Rust
=======

This is an example/reference design which was used to verify various components provided
by this library. To minimize the amount of HW designs required, one project is provided.
The design was kept as generic as possible. In principle, it should be possible to adapt the
hardware design to other boards with modifications.

If you just want a pre-built bitstream and don't need to modify the hardware design, you don't
need Vivado at all: run `just download-zed-gateware` from the repository root to download one
directly into `zedboard-gateware/zedboard-rust.bit`.

# Pre-Requisites

- [Vivado installation](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools.html).
  This example design was created with/for Vivado 2025.2, but also might work for newer versions.
  You might have to manually adjust some variables in `src/zedboard-bd.tcl` for newer versions.
  A [Vitis installation](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis.html)
  (which includes Vivado) is only needed if you use the optional `sdtgen` flow described below -
  the recommended workflow of unzipping the `*.xsa` directly does not require Vitis at all.

# Loading the project and the block design with the GUI

You can load the project using the batch mode of `vivado` inside the folder where you want to
create the `zedboard-rust` project:

```sh
vivado -mode batch -source <path to zedboard-rust.tcl> -tclargs --overwrite
```

for example, to create the directory directly insdie this directory:

```sh
vivado -mode batch -source zedboard-rust.tcl -tclargs --overwrite
```

This should create a `zedboard-rust` Vivado project folder containing a `zedboard-rust.xpr`
project file. You can load this project file with Vivado:

```sh
vivado zedboard-rust.xpr
```

You can perform all the steps specified in the Vivado GUI as well using `Execute TCL script` and
`Load Project`.

# Updating the project

If you add custom RTL code, you might have to edit the `zedboard-rust.tcl` project file and
add the source files there. This file was created using the `write_project_tcl` but was
optimized, so it might be easier to manually update the file.

# Generating the SDT folder from a hardware description

You can generate a hardware description by building the block design by using `Generate Bitstream`
inside the Vivado GUI and then exporting the hardware description via
`File -> Export -> Export Hardware`. This allows to generate a `*.xsa` file which describes the
hardware.

## Extracting `ps7_init.tcl` and the bitstream directly from the `*.xsa` (recommended)

A `*.xsa` file is just a zip archive, so the easiest way to get at the `ps7_init.tcl` script and
the bitstream is to unzip it directly - no Vitis/`sdtgen` invocation required:

```sh
unzip -l zedboard-rust/zedboard-rust.xsa
```

```sh
unzip -o zedboard-rust/zedboard-rust.xsa ps7_init.tcl '*.bit' -d xsa_extract
```

This gives you the same `ps7_init.tcl` (and the `.bit` bitstream, named after the block design
rather than the project) that the SDT flow below produces, in a `xsa_extract` folder - kept
separate from `sdt_out` below since this isn't actually SDT output, just the raw files pulled out
of the `*.xsa` archive - without needing a Vitis/AMD_TOOLS installation at all.

## Generating the SDT folder via `sdtgen`

Alternatively, you can generate the full SDT output folder, which contains the same
`ps7_init.tcl` script plus some additional files (e.g. `zedboard.hwh`) that the simple unzip above
doesn't give you. The provided `sdtgen.tcl` and `sdtgen.py` script simplify this process.

For example, the following command generates the SDT output folder inside a folder
named `sdt_out` for a hardware description files `zedboard-rust/zedboard-rust.xsa`,
assuming that the Vitis tool suite is installed at `/tools/Xilinx/Vitis/2024.1`:

```sh
export AMD_TOOLS="/tools/2025.2/Vitis"
./sdtgen.py -x ./zedboard-rust/zedboard-rust.xsa
```

If you have already sourced the setting script, you do not have to set the `AMD_TOOLS` environment.
Run `sdtgen.py -h` for more information and configuration options. The `sdtgen.py` is a helper
script which will invoke `sdtgen.tcl` to generate the SDT.
