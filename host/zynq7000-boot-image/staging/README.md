Boot Image Creation Staging
========

This folder provides the basic files required to create bare metal boot images.

This includes a simple `boot.bif` file which can be used by the AMD
[bootgen](https://docs.amd.com/r/en-US/ug1283-bootgen-user-guide) utility
to create a boot binary consisting of

- A first-stage bootloader (FSBL)
- A FPGA bitstream which can be loaded to the Zynq PL by the FSBL.
- A primary application which runs in DDR memory and is also copied to DDR by the FSBL.

## Example for the Zedboard

An example use-case for the Zedboard would be a `boot.bin` containing the `zedboard-fsbl` Rust
FSBL or the Xilinx FSBL, a bitstream `zedboard-rust.bit` generated from the `zedboard-fpga-design`
project or your own Vivado project and finally any primary application of your choice which
should run in DDR.

You can copy the files into the staging area, using the file names `fsbl.elf`, `fpga.bit` and
`application.elf`, which are also ignored by the VCS.

Then, you can simply run `bootgen` to generate the boot binary:

```sh
bootgen -arch zynq -image boot.bif -o boot.bin -w on
```
