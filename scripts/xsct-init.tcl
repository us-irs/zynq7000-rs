if {[info exists env(ip_address_hw_server)]} {
  set ip $env(ip_address_hw_server)
} else {
  set ip "localhost"
}

set init_tcl ""
if {[llength $argv] >= 1} {
    set init_tcl [lindex $argv 0]
} else {
    puts "error: missing required first argument pointing to initialization TCL script"
    exit 1
}

if {![file exists $init_tcl]} {
  puts "the ps init tcl script '$init_tcl' does not exist"
  exit 0
}

# parse command-line arguments
set bitstream ""
if {[llength $argv] >= 2} {
    set bitstream [lindex $argv 1]
}

puts "Hardware server IP address: $ip"
connect -url tcp:$ip:3121

set devices [targets]

set apu_line [string trim [targets -nocase -filter {name =~ "APU"}]]
set arm_core_0_line [string trim [targets -nocase -filter {name =~ "ARM Cortex-A9 MPCore #0"}]]
set fpga_line [string trim [targets -nocase -filter {name =~ "xc7z020"}]]

set apu_device_num [string index $apu_line 0]
set arm_core_0_num [string index $arm_core_0_line 0]
set fpga_device_num [string index $fpga_line 0]

puts "Select ps target with number: $apu_device_num"

# Select the target
target $apu_device_num

# Resetting the target involved problems when an image is stored on the flash.
# It has turned out that it is not essential to reset the system before loading
# the software components into the device.
puts "Reset target"
# TODO: Make the reset optional/configurable via input argument.
# Reset the target
rst

# Check if bitstream is set and the file exists before programming FPGA
if {$bitstream eq ""} {
    puts "Skipping bitstream programming (bitstream argument not set)"
} elseif {![file exists $bitstream]} {
    puts "Error: The bitstream file '$bitstream' does not exist"
} else {
    puts "Set FPGA target with number: $fpga_device_num"
    target $fpga_device_num

    # Without this delay, the FPGA programming may fail
    after 1500

    puts "Programming FPGA with bitstream: $bitstream"
    fpga -f $bitstream
}

puts "Set ps target with device number: $arm_core_0_num"
targets $arm_core_0_num

puts "Initialize processing system"
# Init processing system
source $init_tcl

ps7_init
ps7_post_config

puts "Set arm core 0 target with number: $arm_core_0_num"
target $arm_core_0_num

if {[info exists env(APP)] && [file exists $env(APP)]} {
  puts "Download app $env(APP) to target"
  dow $env(APP)
}

puts "Successful"
