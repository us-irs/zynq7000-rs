set script_dir [file dirname [info script]]
open_project $script_dir/zedboard-rust/zedboard-rust.xpr

open_bd_design $script_dir/zedboard-rust/zedboard-rust.srcs/sources_1/bd/zedboard/zedboard.bd

write_bd_tcl -force $script_dir/src/zedboard-bd.tcl
