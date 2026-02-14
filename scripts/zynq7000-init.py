#!/usr/bin/env python3
import shlex
import argparse
import os
import subprocess
import sys
from pathlib import Path

# Define the default values
TCL_SCRIPT_NAME = "xsct-flasher.tcl"
SCRIPTS_DIR = "scripts"
DEFAULT_IP_ADDRESS_HW_SERVER = "localhost"


def main():
    parser = argparse.ArgumentParser(
        description="Wrapper script for xsct-init.tcl\n"
        "It launches xsct, initialize and prepare the  processing system for debugging and "
        "allows loading a bitstream and/or bare-metal application to the board.",
        formatter_class=argparse.RawTextHelpFormatter,  # This preserves line breaks
    )
    parser.add_argument(
        "-t",
        "--tools",
        # Use env var if set
        default=os.getenv("AMD_TOOLS"),
        help="The path to the tool to use. Must point to a valid Vivado tools installation which"
        "also provides xsct, for example a Vitis installation.\nThe directory where the path "
        "points to should contain a shell script named settings64.sh.\nYou can also set the "
        "AMD_TOOLS env variable to set this.",
    )
    parser.add_argument(
        "--sdt",
        dest="sdt",
        help="Path to a SDT output folder which contains a PS7 init TCL script and a bitstream.",
    )
    parser.add_argument(
        "--no-bit",
        dest="no_bit",
        action="store_true",
        help="No bitstream flashing for initialization with SDT.",
    )
    parser.add_argument("-a", "--app", dest="app", help="Path to the app to program")
    default_ip = os.getenv("HW_SERVER_IP")
    if not default_ip:
        default_ip = DEFAULT_IP_ADDRESS_HW_SERVER
    parser.add_argument(
        "-i",
        "--ip",
        default=default_ip,
        help="The IP address of the hardware server (default: localhost)",
    )
    parser.add_argument(
        "--itcl",
        dest="init_tcl",
        default=os.getenv("TCL_INIT_SCRIPT"),
        help="Path to the ps7 initialization TCL file to prepare the processing system.\n"
        "You can also set the TCL_INIT_SCRIPT env variable to set this.\n"
        "It is also set implicitely when specifying the SDT folder with --sdt",
    )
    parser.add_argument(
        "-b",
        "--bit",
        dest="bit",
        default=os.getenv("ZYNQ_BITSTREAM"),
        help="Optional path to the bitstream which will also be programed to the device. It is"
        " also searched automatically if the --sdt option is used.\n",
    )

    args = parser.parse_args()

    settings_script = os.path.join(args.tools, "settings64.sh")

    if not os.path.isfile(settings_script):
        print(f"Invalid tool path {args.tools}, did not find settings file.")
        sys.exit(1)

    # Source the settings script and check for xsdb availability
    command = f"source {settings_script} && command -v xsct"
    result = subprocess.run(
        command,
        shell=True,
        capture_output=True,
        executable="/bin/bash",
    )

    if result.returncode != 0:
        print("Error: 'xsct' could not be found after sourcing settings64.sh.")
        sys.exit(1)

    if args.app and not os.path.isfile(args.app):
        print(f"The app '{args.app}' does not exist")
        sys.exit(1)

    os.environ["XSCT_HW_SERVER_IP"] = args.ip
    init_tcl = None
    bitstream = None
    if args.bit:
        bitstream = args.bit
    if args.sdt:
        sdt_path = Path(args.sdt)
        # CLI bitstream argument overrides implicit SDT bitstream.
        if not args.no_bit and not bitstream:
            # Search for .bit files in the SDT folder
            bit_files = list(sdt_path.glob("*.bit"))
            if len(bit_files) == 0:
                print("Error: No .bit file found in the SDT folder.")
            elif len(bit_files) > 1:
                print("Error: Multiple .bit files found in the SDT folder.")
            bitstream = str(bit_files[0])
        # Search for the ps7_init.tcl file
        init_script = sdt_path / "ps7_init.tcl"
        if not init_script.exists():
            sys.exit("Error: ps7_init.tcl file not found in the SDT folder.")

        init_tcl = str(init_script)
    else:
        if not args.init_tcl:
            print("Error: No ps7_init.tcl file specified.")
            sys.exit(1)
        if args.bit:
            bitstream = args.bit
        init_tcl = args.init_tcl

    # Get the script's directory
    script_dir = Path(__file__).resolve().parent
    xsct_script = script_dir / TCL_SCRIPT_NAME

    if not xsct_script.exists():
        xsct_script = Path(os.path.join(SCRIPTS_DIR, TCL_SCRIPT_NAME))
        if not xsct_script.exists():
            print(f"Could not find the xsct initialization script {TCL_SCRIPT_NAME}")
            sys.exit(1)

    # Launch xsct with the initialization script
    # Prepare tcl_args as a list to avoid manual string concatenation issues
    cmd_list = ["xsct", str(xsct_script), init_tcl]
    if bitstream:
        cmd_list.append("--bit")
        cmd_list.append(bitstream)
    if args.app:
        cmd_list.append("--app")
        cmd_list.append(args.app)

    # Join safely for shell execution
    xsct_cmd = shlex.join(cmd_list)
    print(f"Running xsct command: {xsct_cmd}")
    command = f"bash -c 'source {settings_script} && {xsct_cmd} | tee xsct-output.log'"
    subprocess.run(
        command,
        shell=True,
        check=True,
    )


if __name__ == "__main__":
    main()
