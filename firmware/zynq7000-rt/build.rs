use std::io::Write;

fn main() {
    arm_targets::process();
    write("z7link.x", include_bytes!("z7link.x"));
    check_link_arg();
}

fn write(file: &str, contents: &[u8]) {
    // Put the linker script in our output directory and ensure it's on the linker search path.
    let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    std::fs::File::create(out.join(file))
        .unwrap()
        .write_all(contents)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={file}");
}

// The same target rustflags apply to every crate built for the target, including this one, so
// checking our own CARGO_ENCODED_RUSTFLAGS is a reliable proxy for what the final binary will
// get. Skipped for non-Arm builds (e.g. `cargo doc`/`cargo check` on the host), where the flag
// is irrelevant.
fn check_link_arg() {
    println!("cargo:rerun-if-env-changed=CARGO_ENCODED_RUSTFLAGS");
    if std::env::var("CARGO_CFG_TARGET_ARCH").as_deref() != Ok("arm") {
        return;
    }
    let rustflags = std::env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    let args: Vec<&str> = rustflags.split('\u{1f}').collect();

    if !args.iter().any(|arg| arg.contains("link.x")) {
        println!(
            "cargo:warning=zynq7000-rt: your rustflags do not reference a linker script at \
             all (nothing contains `link.x`). You likely forgot to add it, see the crate docs."
        );
    }
    if !args.iter().any(|arg| arg.contains("z7link.x")) {
        println!(
            "cargo:warning=zynq7000-rt: your rustflags do not pass `-Clink-arg=-Tz7link.x`. \
             Without it, the final binary either fails to link with undefined symbols or links \
             against aarch32-rt's own link.x instead. Add `-Clink-arg=-Tz7link.x` to the target \
             rustflags in your `.cargo/config.toml`."
        );
    }
}
