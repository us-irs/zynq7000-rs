all: check-all build-all clean-all fmt-all clippy-all docs-zynq

check-all: (check "zynq") (check "tools") (check "zynq7000-boot-image")
clean-all: (clean "zynq") (clean "tools") (clean "zynq7000-boot-image")
build-all: build-zynq (build "tools") (build "zynq7000-boot-image")
fmt-all: (fmt "zynq") (fmt "tools") (fmt "zynq7000-boot-image")
clippy-all: (clippy "zynq") (clippy "tools") (clippy "zynq7000-boot-image")

check target:
  cd {{target}} && cargo check

build target:
  cd {{target}} && cargo build

build-zynq: (build "zynq")
  cd "zynq/zedboard-fsbl" && cargo build --release

clean target:
  cd {{target}} && cargo clean

fmt target:
  cd {{target}} && cargo +stable fmt

clippy target:
  cd {{target}} && cargo clippy -- -D warnings

docs-zynq: docs-pac docs-hal
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-mmu
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-rt
[working-directory: 'zynq']
docs-pac:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000
[working-directory: 'zynq']
docs-pac-html:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000 --open
[working-directory: 'zynq']
docs-hal:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-hal --features alloc
[working-directory: 'zynq']
docs-hal-html:
  RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options" cargo +nightly doc -p zynq7000-hal --features alloc --open

[working-directory: 'zynq-boot-image/staging']
bootgen:
  bootgen -arch zynq -image boot.bif -o boot.bin -w on
  echo "Generated boot.bin at zynq-boot-image/staging"

[no-cd]
run binary:
  # Run the initialization script. It needs to be run inside the justfile directory.
  python3 {{justfile_directory()}}/scripts/zynq7000-init.py

  # Run the GDB debugger in GUI mode.
  gdb-multiarch -q -x {{justfile_directory()}}/zynq/gdb.gdb {{binary}} -tui
