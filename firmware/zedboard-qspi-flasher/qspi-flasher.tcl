if {[info exists env(ip_address_hw_server)]} {
  set ip $env(ip_address_hw_server)
} else {
  set ip "localhost"
}

# absolute directory that contains *this* script
set script_dir [file dirname [info script]]

# Defaults
set boot_bin_addr 0x10000000
set boot_bin_size_addr 0x900000
set init_tcl ""
set bin ""
set bitstream ""

# Usage helper
proc usage {} {
  puts "Usage: xsct qspi-flasher.tcl <init.tcl> \[-b|--bin <boot.bin>\]"
  puts "Options:"
  puts "  -b, --bin    Path to boot binary to flash"
  puts "  -h, --help   Show this help"
}

# Compact, robust parser
set expecting ""
set endopts 0
foreach arg $argv {
  # If previous option expects a value, take this arg
  if {$expecting ne ""} {
    set $expecting $arg
    set expecting ""
    continue
  }

  # Option handling (until we see --)
  if {!$endopts && [string match "-*" $arg]} {
    if {$arg eq "--"} { set endopts 1; continue }
    if {$arg eq "-h" || $arg eq "--help"} { usage; exit 0 }
    if {$arg eq "-b" || $arg eq "--bin"} { set expecting bin; continue }
    puts "error: unknown option: $arg"; usage; exit 1
  }

  # Positional: expect only <init.tcl>
  if {$init_tcl eq ""} {
    set init_tcl $arg
  } else {
    puts "error: unexpected positional argument: $arg"
    usage
    exit 1
  }
}

# Check that QSPI flasher app exists.
set flasher_app [file join $script_dir .. target armv7a-none-eabihf release zedboard-qspi-flasher]
if {![file exists $flasher_app]} {
    error "QSPI flasher application not found at path: $flasher_app"
}
set flasher_app [file normalize $flasher_app]

# Validate required init script
if {$init_tcl eq ""} {
  puts "error: missing required first argument pointing to initialization TCL script"
  usage
  exit 1
}
if {![file exists $init_tcl]} {
  puts "error: the PS init tcl script '$init_tcl' does not exist"
  exit 1
}

# Resolve app: CLI takes precedence over env(APP)
if {$bin ne ""} {
  if {![file exists $bin]} {
    puts "error: the boot binary file '$bin' does not exist"
    exit 1
  }
} elseif {[info exists env(BOOTBIN)]} {
  if {[file exists $env(BOOTBIN)]} {
    set bin $env(BOOTBIN)
  } else {
    puts "warning: BOOTBIN environment variable is set but file does not exist: $env(BOOTBIN)"
  }
}

if {$bin eq ""} {
  puts "error: boot.bin binary required"
  usage
  exit 1
}

# Validate bitstream if provided
if {$bitstream ne "" && ![file exists $bitstream]} {
  puts "error: the bitstream file '$bitstream' does not exist"
  exit 1
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

puts "Set ps target with device number: $arm_core_0_num"
targets $arm_core_0_num

puts "Initialize processing system"
# Init processing system
source $init_tcl

ps7_init
ps7_post_config

puts "Set arm core 0 target with number: $arm_core_0_num"
target $arm_core_0_num

puts "Download boot.bin $bin to target DDR at address $boot_bin_addr"
dow -data $bin $boot_bin_addr

# Write boot.bin binary size to specific address.
set boot_bin_size [file size $bin]
puts "Writing boot.bin size $boot_bin_size to target DDR at address $boot_bin_size_addr"
mwr ${boot_bin_size_addr} ${boot_bin_size}

puts "Flashing QSPI flasher app"
dow $flasher_app

puts "Starting QSPI flasher app"
con

puts "Success"
