set hw_file "[get_property DIRECTORY [current_project]]/zedboard-rust.xsa"
write_hw_platform -fixed -include_bit -force -file $hw_file
