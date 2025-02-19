#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Script to generate SDT files from a XSA file"
    )
    parser.add_argument(
        "-t",
        "--tools",
        # Required only if env var is not set
        required=not bool(os.getenv("AMD_TOOLS")),
        # Use env var if set
        default=os.getenv("AMD_TOOLS"),
        help="The path to the tool to use. Must point to a valid Vivado tools installation which"
        "also provides xsct, for example a Vitis installation.\nThe directory where the path "
        "points to should contain a shell script named settings64.sh.\n You can also set the "
        "AMD_TOOLS env variable to set this.",
    )
    parser.add_argument(
        "-x",
        "--xsa",
        help="Path to the input XSA file",
        default="zedboard-rust/zedboard-rust.xsa",
    )
    parser.add_argument(
        "-o",
        "--out",
        default="sdt_out",
        help="Directory to store the generated SDT files",
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

    xsct_script = "sdtgen.tcl"

    command = f"bash -c 'source {settings_script} && xsct {xsct_script} {args.xsa} {args.out} | tee xsct-output.log'"
    subprocess.run(
        command,
        shell=True,
        check=True,
    )


if __name__ == "__main__":
    main()
