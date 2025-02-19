Zedboard FPGA design for Rust
=======

This is an example/reference design which was used to verify various components provided
by this library. To minimize the amount of HW designs required, one project is provided.
The design was kept as generic as possible. In principle, it should be possible to adapt the
hardware design to other boards with modifications.

# Pre-Requisites

- [Vivado installation](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools.html)
  or [Vitis installation](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis.html)
  which includes Vivado. This example design was created with/for Vivado 2024.1, but also might work
  for newer versions.
- [Zedboard board files](https://github.com/Digilent/vivado-boards) added to the Vivado installation.

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
