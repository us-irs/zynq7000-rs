set script_dir [file dirname [info script]]
open_project $script_dir/zedboard-rust/zedboard-rust.xpr

set bitstream $script_dir/zedboard-rust/zedboard-rust.runs/impl_1/zedboard_wrapper.bit
file copy -force $bitstream $script_dir/zedboard-rust/zedboard-rust.bit
# Also copy next to the project folder, at the same path `just download-zed-gateware` uses, so
# both flows leave the bitstream in the same place.
file copy -force $bitstream $script_dir/zedboard-rust.bit
