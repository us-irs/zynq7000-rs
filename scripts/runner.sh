#!/bin/bash

# Exit if no arguments are provided
if [ "$#" -eq 0 ]; then
    echo "Error: No arguments provided."
    echo "Usage: run.sh <binary>"
    exit 1
fi

# Get the absolute path to the `scripts/` directory
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"

# Get the absolute path to the project root
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Run the initialization script
"$SCRIPT_DIR/zynq7000-init.py"

# Run the GDB debugger in GUI mode.
gdb-multiarch -q -x gdb.gdb "$@" -tui