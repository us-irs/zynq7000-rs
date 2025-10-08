[![Crates.io](https://img.shields.io/crates/v/zynq7000-hal)](https://crates.io/crates/zynq7000-hal)
[![docs.rs](https://img.shields.io/docsrs/zynq7000-hal)](https://docs.rs/zynq7000-hal)
[![ci](https://github.com/us-irs/zynq7000-rs/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/us-irs/zynq7000-rs/actions/workflows/ci.yml)

# HAL for the AMD Zynq 7000 SoC family

This repository contains the **H**ardware **A**bstraction **L**ayer (HAL), which is an additional
hardware abstraction on top of the [peripheral access API](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq/zynq7000).

It is the result of reading the datasheet for the device and encoding a type-safe layer over the
raw PAC. This crate also implements traits specified by the
[embedded-hal](https://github.com/rust-embedded/embedded-hal) project, making it compatible with
various drivers in the embedded rust ecosystem.

The [top-level README](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs) and the documentation
contain more information on how to use this crate.
