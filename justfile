all: check-all build-all clean-all fmt-all clippy-all docs-zynq

check-all: (check "firmware") (check "host")
clean-all: (clean "firmware") (clean "host")
build-all: build-zynq (build "host")
fmt-all: (fmt "firmware") (fmt "host")
clippy-all: (clippy "firmware") (clippy "host")

check target:
  cd {{target}} && cargo check

build target:
  cd {{target}} && cargo build

build-zynq: (build "firmware")
  cd "firmware/zedboard-fsbl" && cargo build --release

clean target:
  cd {{target}} && cargo clean

fmt target:
  cd {{target}} && cargo +stable fmt

clippy target:
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
  gdb-multiarch -q -x {{justfile_directory()}}/zynq/gdb.gdb {{binary}} -tui
