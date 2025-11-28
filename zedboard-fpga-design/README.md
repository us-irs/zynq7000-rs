Zedboard FPGA design for Rust
=======

This is an example/reference design which was used to verify various components provided
by this library. To minimize the amount of HW designs required, one project is provided.
The design was kept as generic as possible. In principle, it should be possible to adapt the
hardware design to other boards with modifications.

# Pre-Requisites

- [Vivado installation](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools.html)
  or [Vitis installation](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis.html)
  which includes Vivado. This example design was created with/for Vivado 2025.2, but also might work
  for newer versions. You might have to manually adjust some variables in `src/zedboard-bd.tcl`
  for newer versions.

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

# Generating the SDT folder from a hardware description

You can generate a hardware description by building the block design by using `Generate Bitstream`
inside the Vivado GUI and then exporting the hardware description via
`File -> Export -> Export Hardware`. This allows to generate a `*.xsa` file which describes the
hardware.

After that, you can generate the SDT output folder which contains various useful files like
the `ps7_init.tcl` script. The provided ` sdtgen.tcl` and `stdgen.py` script simplify this process.

For example, the following command generates the SDT output folder inside a folder
named `sdt_out` for a hardware description files `zedboard-rust/zedboard-rust.xsa`,
assuming that the Vitis tool suite is installed at `/tools/Xilinx/Vitis/2024.1`:

```sh
export AMD_TOOLS="/tools/2025.2/Vitis"
./sdtgen.py -x ./zedboard-rust/zedboard-rust.xsa
```

Run `stdgen.py -h` for more information and configuration options. The `stdgen.py` is a helper
script which will invoke `sdtgen.tcl` to generate the SDT.
