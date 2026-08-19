set script_dir [file dirname [info script]]
open_project $script_dir/zedboard-rust/zedboard-rust.xpr

file copy -force $script_dir/zedboard-rust/zedboard-rust.runs/impl_1/zedboard_wrapper.bit $script_dir/zedboard-rust/zedboard-rust.bit
