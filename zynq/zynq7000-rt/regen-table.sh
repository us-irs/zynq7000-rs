#!/bin/bash
cargo +stable run --target $(rustc -vV | grep host | cut -d ' ' -f2) --bin table-gen --no-default-features --features tools
