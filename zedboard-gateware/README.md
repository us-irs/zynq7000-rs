Zedboard FPGA design for Rust
=======

This is an example/reference design which was used to verify various components provided
by this library. To minimize the amount of HW designs required, one project is provided.
The design was kept as generic as possible. In principle, it should be possible to adapt the
hardware design to other boards with modifications.

If you just want a pre-built bitstream and don't need to modify the hardware design, you don't
need Vivado at all: run `just download-zed-gateware` from the repository root to download one
directly into `zedboard-gateware/zedboard-rust.bit`.

# Board resource assignment

Overview of how the Zedboard's on-board user I/O is wired up and accessed in this design.

| Board resource | Count | Signal(s) | Access method | Notes |
|---|---|---|---|---|
| Push buttons | 5 | BTNC, BTNU, BTND, BTNL, BTNR | EMIO 24 to EMIO 28 | |
| LEDs | 8 | LD0 - LD7 | LD0 to LD5 by EMIO0 to EMIO5, LD6 to LD7 by AXI GPIO Channel 1 pins 0 and 1 | |
| DIP switches | 8 | SW0 - SW7 | SW0 to SW6 by EMIO16 tp EMIO22, SW7 by AXI GPIO Channel 2 pin 0 | |
| OLED display | 1 (6 signals) | VBAT, VDD, RES, D/C, SCLK, SDIN | PS SPI0 for SCLK and SDIN, D/C by EMIO11, RES by EMIO12, VDD by EMIO13, VBAT by EMIO14   | 128x32 monochrome |

# AXI FPGA resources

Table of various resources which are part of the FPGA design, and their purpose.

| IP core | Base address | Drives | IRQ_F2P |
|---|---|---|---|
| [AXI GPIO](https://www.amd.com/en/products/adaptive-socs-and-fpgas/intellectual-property/axi_gpio.html) | `0x41200000` | LEDs (channel 1, shared with EMIO) and DIP switches (channel 2, shared with EMIO), see the board resource table above | 4 |
| [AXI UART Lite](https://www.amd.com/en/products/adaptive-socs-and-fpgas/intellectual-property/axi_uartlite.html) | `0x42C00000` | UART, muxed onto the same physical UART pins as AXI UART16550 and PS UART0 via [`uart_mux_0`](src/uart_mux.vhd) | 0 |
| [AXI UART16550](https://www.amd.com/de/products/adaptive-socs-and-fpgas/intellectual-property/axi_uart16550.html) | `0x43C00000` | UART, muxed onto the same physical UART pins as AXI UART Lite and PS UART0 via [`uart_mux_0`](src/uart_mux.vhd) | 1 |
| [AXI DMA](https://www.amd.com/en/products/adaptive-socs-and-fpgas/intellectual-property/axi_dma.html) | `0x40400000` | Scatter-gather-less DMA between its `S_AXI_LITE` control interface and DDR/QSPI memory | 2 (mm2s), 3 (s2mm) |
| [`uart_mux_0`](src/uart_mux.vhd) | - | Selects which of AXI UART Lite, AXI UART16550, or PS UART0 drives the physical `UART_txd`/`UART_rxd` pins, select inputs driven by EMIO8, 9, 10 | - |

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
`Load Project`. Once the project exists, `just vivado-gui` (from the repository root) is a
shortcut for the `vivado zedboard-rust.xpr` step above.

# Updating the project

If you add custom RTL code, you might have to edit the `zedboard-rust.tcl` project file and
add the source files there. This file was created using the `write_project_tcl` but was
optimized, so it might be easier to manually update the file.

If instead you change the block design itself in the Vivado GUI, run `just export-bd` from the
repository root afterward to regenerate the checked-in `src/zedboard-bd.tcl` from it. This also
strips machine-specific memory-init paths (`CONFIG.Coe_File`, `CONFIG.Load_Init_File`) so the
checked-in file stays portable across machines.

# Exporting the hardware description and/or bitstream

You can generate a hardware description by building the block design by using `Generate Bitstream`
inside the Vivado GUI and then exporting the hardware description via
`File -> Export -> Export Hardware`. This allows to generate a `*.xsa` file which describes the
hardware. `just export-hw` (from the repository root) does the same thing from the command line,
assuming the project has already been implemented and a bitstream generated.

If you've only changed RTL/timing and just need the new bitstream, without re-exporting the whole
hardware platform description, `just export-bit` copies the freshly implemented bitstream to both
`zedboard-rust/zedboard-rust.bit` (next to the project) and `zedboard-gateware/zedboard-rust.bit`
(the same path `just download-zed-gateware` uses).

## Extracting `ps7_init.tcl` and the bitstream directly from the `*.xsa` (recommended)

A `*.xsa` file is just a zip archive, so the easiest way to get at the `ps7_init.tcl` script and
the bitstream is to unzip it directly - no Vitis/`sdtgen` invocation required:

```sh
unzip -l zedboard-rust/zedboard-rust.xsa
```

```sh
unzip -o zedboard-rust/zedboard-rust.xsa ps7_init.tcl '*.bit' -d xsa_extract
```

`just extract-hw` (from the repository root) runs the equivalent unzip for you, straight into the
`zedboard-rust` project folder the `.xsa` already lives in, rather than a separate `xsa_extract`
folder. `just export-zed-gateware` chains `export-hw` and `extract-hw` together, so a single
command takes you from an implemented design to an extracted `ps7_init.tcl`/bitstream.

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
