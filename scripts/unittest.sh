#!/bin/bash
cargo +stable test --target $(rustc -vV | grep host | cut -d ' ' -f2) -p zynq7000-hal
cargo +stable test --target $(rustc -vV | grep host | cut -d ' ' -f2) -p zynq7000
