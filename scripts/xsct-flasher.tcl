if {[info exists env(XSCT_HW_SERVER_IP)]} {
  set ip $env(XSCT_HW_SERVER_IP)
} else {
  set ip "localhost"
}

# Defaults
set init_tcl ""
set app ""
set bitstream ""

# Usage helper
proc usage {} {
  puts "Usage: xsct xsct-helper.tcl <init.tcl> \[-a|--app app.elf\] \[-b|--bit design.bit]"
  puts "Options:"
  puts "  -a, --app    Path to application ELF to download"
  puts "  -b, --bit    Path to FPGA bitstream (.bit) to program"
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
    if {$arg eq "-a" || $arg eq "--app"} { set expecting app; continue }
    if {$arg eq "-b" || $arg eq "--bit"} { set expecting bitstream; continue }
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
if {$app ne ""} {
  if {![file exists $app]} {
    puts "error: the app file '$app' does not exist"
    exit 1
  }
} elseif {[info exists env(APP)]} {
  if {[file exists $env(APP)]} {
    set app $env(APP)
  } else {
    puts "warning: APP environment variable is set but file does not exist: $env(APP)"
  }
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

if {$app ne ""} {
  puts "Download app $app to target"
  dow $app
  puts "Starting app"
  con
}

puts "Success"
