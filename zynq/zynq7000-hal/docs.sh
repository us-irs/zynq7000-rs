#!/bin/sh
export RUSTDOCFLAGS="--cfg docsrs --generate-link-to-definition -Z unstable-options"
cargo +nightly doc --features alloc --open
