all: check build check-fmt-all clippy docs-zynq

check: (check-dir "firmware") (check-dir "host")
clean: (clean-dir "firmware") (clean-dir "host")
build: build-zynq (build-dir "host")
fmt: (fmt-dir "firmware") (fmt-dir "host")
check-fmt-all: (check-fmt "firmware") (check-fmt "host")
clippy: (clippy-dir "firmware") (clippy-dir "host")

check-dir target:
  cd {{target}} && cargo check

build-dir target:
  cd {{target}} && cargo build

build-zynq: (build-dir "firmware")
  cd "firmware/zedboard-fsbl" && cargo build --release

clean-dir target:
  cd {{target}} && cargo clean

check-fmt target:
  cd {{target}} && cargo +stable fmt --all -- --check

fmt-dir target:
  cd {{target}} && cargo +stable fmt

clippy-dir target:
  cd {{target}} && cargo clippy -- -D warnings

[working-directory: 'firmware']
docs-zynq: docs-pac docs-hal
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-mmu
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-rt
[working-directory: 'firmware']
docs-pac:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000
[working-directory: 'firmware']
docs-pac-html:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000 --open
[working-directory: 'firmware']
docs-hal:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-hal --features alloc --no-deps
[working-directory: 'firmware']
docs-hal-html:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-hal --features alloc --open

[working-directory: 'firmware/zynq-boot-image/staging']
bootgen:
  bootgen -arch zynq -image boot.bif -o boot.bin -w on
  echo "Generated boot.bin at zynq-boot-image/staging"

[no-cd]
run binary:
  # Run the initialization script. It needs to be run inside the justfile directory.
  python3 {{justfile_directory()}}/scripts/zynq7000-init.py

  # Run the GDB debugger in GUI mode.
  gdb-multiarch -q -x {{justfile_directory()}}/firmware/gdb.gdb {{binary}} -tui

flash-nor-zedboard boot_binary:
  cd {{justfile_directory()}}/firmware/zedboard-qspi-flasher && cargo build --release
  xsct firmware/zedboard-qspi-flasher/qspi-flasher.tcl scripts/ps7_init.tcl -b {{boot_binary}}
