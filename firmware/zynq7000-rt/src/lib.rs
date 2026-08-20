//! # Startup code and minimal run-time support for the AMD Zynq 7000 SoCs
//!
//! This run-time crate is based on both the
//! [startup code provided by AMD](https://github.com/Xilinx/embeddedsw/blob/master/lib/bsp/standalone/src/arm/cortexa9/gcc/boot.S)
//! and the [aarch32-rt](https://github.com/rust-embedded/aarch32/tree/main/aarch32-rt) generic
//! run-time library.
//!
//! It activates the `fpu-d32` feature of the [`aarch32_rt`] library and overrides the
//! `_default_start` method to add necessary setup code for the Zynq7000. It re-exports the
//! [`aarch32_rt`] crate, including the attributes macros. The
//! [documentation](https://docs.rs/aarch32-rt/latest/aarch32_rt/) specifies these in detail.
//!
//! Some major differences to the startup code provided by AMD:
//!
//! - L2 cache initialization is **not** performed.
//! - MMU table is **not** configured and enabled.
//! - Modification to the stack setup code, because a different linker script is used.
//!
//! The [zynq7000-hal](https://docs.rs/zynq7000-hal) provides components for L2 cache and MMU
//! configuration and initialization.
//!
//! ## Features
//!
//! * `rt` - Default feature which activates the run-time.
//!
//! ## Linker script
//!
//! This crate ships its own complete linker script, `z7link.x`, which supersedes
//! `aarch32-rt`'s `link.x`. Your project must pass `-Clink-arg=-Tz7link.x` in its rustflags
//! instead of `-Tlink.x`, or linking will fail with undefined symbols. `z7link.x` is a copy of
//! `aarch32-rt/link.x` with two extra output sections, `.ocm.data` and `.ocm.bss`, spliced in
//! alongside the regular `.data`/`.bss`.
//!
//! Add the flag to your project's `.cargo/config.toml`, e.g.:
//!
//! ```toml
//! [target.armv7a-none-eabihf]
//! rustflags = [
//!   "-Ctarget-cpu=cortex-a9",
//!   "-Clink-arg=-Tz7link.x",
//! ]
//!
//! [build]
//! target = "armv7a-none-eabihf"
//! ```
//!
//! See `firmware/.cargo/config.toml` in this repository for a complete example.
//!
//! ## Memory Description
//!
//! `z7link.x` expects regions `VECTORS`, `CODE`, `DATA`, `STACKS`, and `OCM` (see
//! [Placing statics in OCM](#placing-statics-in-ocm) below). The `MEMORY` block should name
//! regions after the physical memory they describe (`OCM`, `OCM_UPPER`, `DDR`, ...); the region
//! aliases are what map those onto the logical names `z7link.x` expects. Two minimal example
//! layouts which are provided in a user `memory.x` file:
//!
//! Running entirely out of OCM, e.g. for an FSBL (see `zedboard-fsbl`'s `memory.x`):
//!
//! ```ignore
//! MEMORY
//! {
//!   /* The Zynq7000 has 256 kB of OCM memory of which 196 kB can be used for the FSBL */
//!   OCM : ORIGIN = 0x00000000, LENGTH = 192K
//!   /* Your DDR might be a lot larger. We reserve 63 MB of DDR for the app */
//!   DDR : ORIGIN = 0x00100000, LENGTH = 63M
//!   OCM_UPPER(rx): ORIGIN = 0xFFFF0000, LENGTH = 64K
//! }
//!
//! REGION_ALIAS("VECTORS", OCM);
//! REGION_ALIAS("CODE", OCM);
//! REGION_ALIAS("DATA", OCM);
//! /* Use the upper OCM as the stack */
//! REGION_ALIAS("STACKS", OCM_UPPER);
//! ```
//!
//! Running out of DDR (see the `zedboard` example's `memory.x`), and also providing an uncached
//! memory region for something like DMA descriptors.
//!
//! ```ignore
//! MEMORY
//! {
//!   OCM : ORIGIN = 0x00000000, LENGTH = 192K
//!   /* Your DDR might be a lot larger. We reserve 63 MB of DDR for the app */
//!   DDR : ORIGIN = 0x00100000, LENGTH = 63M
//!   OCM_UPPER(rx): ORIGIN = 0xFFFF0000, LENGTH = 64K
//!   /* Special uncached memory region which is the 64th MB of DDR. Can be configured
//!   uncached at run-time using the MMU */
//!   UNCACHED : ORIGIN = ORIGIN(DDR) + LENGTH(DDR), LENGTH = 1M
//! }
//!
//! REGION_ALIAS("VECTORS", DDR);
//! REGION_ALIAS("CODE", DDR);
//! REGION_ALIAS("DATA", DDR);
//! REGION_ALIAS("STACKS", DDR);
//! ```
//!
//! The linker only searches your project root by default, which is not enough once you're in a
//! workspace or have a more complex build layout. A `build.rs` copying `memory.x` into `OUT_DIR`
//! and adding that to the linker search path makes it work regardless of layout, and also makes
//! Cargo rebuild the project whenever `memory.x` changes:
//!
//! ```ignore
//! use std::env;
//! use std::fs::File;
//! use std::io::Write;
//! use std::path::PathBuf;
//!
//! fn main() {
//!     // Put `memory.x` in our output directory and ensure it's
//!     // on the linker search path.
//!     let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
//!     File::create(out.join("memory.x"))
//!         .unwrap()
//!         .write_all(include_bytes!("memory.x"))
//!         .unwrap();
//!     println!("cargo:rustc-link-search={}", out.display());
//!
//!     // By default, Cargo will re-run a build script whenever
//!     // any file in the project changes. By specifying `memory.x`
//!     // here, we ensure the build script is only re-run when
//!     // `memory.x` is changed.
//!     println!("cargo:rerun-if-changed=memory.x");
//! }
//! ```
//!
//! See any of the `firmware/examples/*/build.rs` files in this repository for a real copy.
//!
//! ## Dual-core (SMP)
//!
//! This library has support for dual-core (SMP) setups where two cores are executing the
//! same software. The [`smp`] module documentation provides more detailed information.
//!
//! ## Placing statics in OCM
//!
//! To place a static in OCM, tag it with the matching link section:
//!
//! ```ignore
//! // Zero-initialized (like regular `.bss`).
//! #[unsafe(link_section = ".ocm.bss")]
//! static mut RX_BUF: [u8; 64] = [0; 64];
//!
//! // Has an explicit non-zero initial value, copied from its DDR load address at boot (like
//! // regular `.data`).
//! #[unsafe(link_section = ".ocm.data")]
//! static TX_BUF: [u8; 64] = [0xAA; 64];
//! ```
//!
//! Both sections are initialized by the startup code in [`rt`] exactly like `.data`/`.bss` are:
//! `.ocm.bss` is zeroed and `.ocm.data` is copied from its load address, before `kmain` runs.
//!
//! Your project's `memory.x` must define a region named `OCM` for `.ocm.data`/`.ocm.bss` to
//! resolve to (see the `zedboard` example's `memory.x` for a real template). If your project
//! doesn't use OCM placement at all, `OCM` can just be aliased to any other region you already
//! have, e.g. `REGION_ALIAS("OCM", DATA);`. Nothing will actually be placed there, so the two
//! sections end up zero-sized and cost nothing.
#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]
#[cfg(all(feature = "rt", arm_profile = "a"))]
pub use aarch32_rt::*;

#[cfg(all(feature = "rt", arm_profile = "a"))]
pub mod rt;
#[cfg(all(feature = "rt", arm_profile = "a"))]
pub mod smp;
