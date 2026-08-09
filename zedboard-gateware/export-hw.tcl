set script_dir [file dirname [info script]]
open_project $script_dir/zedboard-rust/zedboard-rust.xpr
open_run impl_1

set hw_file "[get_property DIRECTORY [current_project]]/zedboard-rust.xsa"
write_hw_platform -fixed -include_bit -force -file $hw_file
