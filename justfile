# Common paths (single source of truth)
INIT_SCRIPT := justfile_directory() / "scripts/zynq7000-init.py"
GDB_CMD := justfile_directory() / "firmware/gdb.gdb"

all: check build check-fmt clippy test docs-zynq
check: (check-dir "firmware") (check-dir "host")
clean: (clean-dir "firmware") (clean-dir "host")
build: build-zynq (build-dir "host")
fmt: (fmt-dir "firmware") (fmt-dir "host")
check-fmt: (check-fmt-dir "firmware") (check-fmt-dir "host")
clippy: (clippy-dir "firmware") (clippy-dir "host")
test: test-clock-calc test-run-data

prepare-repo: download-zed-gateware
  cd {{justfile_directory()}}/host/z7-run && cargo install --path .

# Download a pre-built bitstream
download-zed-gateware:
    curl -L -o {{justfile_directory()}}/zedboard-gateware/zedboard-rust.bit "https://www.dropbox.com/scl/fi/oos5l6qknb4nom7tvbx1t/zedboard-rust.bit?rlkey=ikjec7e6v6rdih7hbti4jhet3&st=av4wf83u&dl=1"

# Export the zedboard-rust Vivado project as a *.xsa hardware platform (zedboard-rust.xsa),
# including the bitstream. Requires that the project has already been implemented (`impl_1`) and
# a bitstream generated, e.g. via the Vivado GUI, and that `vivado` is on PATH.
[working-directory: 'zedboard-gateware']
export-hw:
  vivado -mode batch -source export-hw.tcl

# Unzip ps7_init.tcl and the bitstream out of zedboard-rust.xsa, into the zedboard-rust folder
# next to the xsa itself.
[working-directory: 'zedboard-gateware']
extract-hw:
  unzip -o zedboard-rust/zedboard-rust.xsa ps7_init.tcl '*.bit' -d zedboard-rust

# Export the hardware platform from Vivado and extract ps7_init.tcl/the bitstream from it in one go.
export-zed-gateware: export-hw extract-hw

# Re-generate src/zedboard-bd.tcl from the block design and strip machine-specific memory-init
# config (CONFIG.Coe_File paths, CONFIG.Load_Init_File) so the checked-in tcl stays portable.
# Assumes `vivado` is already on PATH (e.g. settings64.sh/settings64.bat already sourced).
[working-directory: 'zedboard-gateware']
export-bd: && clean-bd
  vivado -mode batch -source export-bd.tcl

clean-bd:
  sed -i '/CONFIG.Coe_File/d; s/CONFIG.Load_Init_File {true}/CONFIG.Load_Init_File {false}/' zedboard-gateware/src/zedboard-bd.tcl

# Copy the impl_1 bitstream out to zedboard-rust.bit, next to the xsa/project.
[working-directory: 'zedboard-gateware']
export-bit:
  vivado -mode batch -source export-bit.tcl

# Open the zedboard-rust Vivado project in the GUI. Assumes `vivado` is already on PATH.
[working-directory: 'zedboard-gateware']
vivado-gui:
  vivado zedboard-rust/zedboard-rust.xpr

check-dir target:
  cd {{target}} && cargo check

build-dir target:
  cd {{target}} && cargo build

[working-directory: 'firmware']
build-zynq: (build-dir "firmware")
  cd "zynq7000" && cargo build --all-features
  cd "zedboard-fsbl" && cargo build --release
  cd "zynq7000-hal" && cargo build --features "time-driver-gtc"

clean-dir target:
  cd {{target}} && cargo clean

check-fmt-dir target:
  cd {{target}} && cargo fmt --all -- --check

fmt-dir target:
  cd {{target}} && cargo fmt

clippy-dir target:
  cd {{target}} && cargo clippy -- -D warnings

# Run the z7-clock-calc test suite (the pure clock/divisor calculators used by
# zynq7000-hal). zynq7000-hal itself cannot build for the host target - see
# host/z7-clock-calc/src/lib.rs - which is why these calculators were split out into
# their own crate in the first place.
test-clock-calc:
  cd {{justfile_directory()}}/host/z7-clock-calc && cargo test --features alloc

test-run-data:
  cd {{justfile_directory()}}/host/z7-run-data && cargo test

[working-directory: 'firmware']
docs-zynq: docs-pac docs-hal
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
  z7-run --config {{justfile_directory()}}/scripts/z7_run.toml --regs {{justfile_directory()}}/scripts/zedboard_init_regs.ron {{binary}}

# Runner which uses TCL scripting but relies on an external hw_server running.
# Set HW_SERVER_IP (same env var zynq7000-init.py already uses for -i/--ip) to the host running
# hw_server/the gdbserver bridge when debugging a board that isn't attached to this machine.
[no-cd]
run-tcl binary:
  python3 {{INIT_SCRIPT}}
  gdb-multiarch -q -ex "target remote {{env_var_or_default('HW_SERVER_IP', 'localhost')}}:3000" -x {{GDB_CMD}} {{binary}} -tui

flash-nor-zedboard boot_binary:
  cd {{justfile_directory()}}/firmware/zedboard-qspi-flasher && cargo build --release
  xsct firmware/zedboard-qspi-flasher/qspi-flasher.tcl scripts/ps7_init.tcl -b {{invocation_directory()}}/{{boot_binary}}
