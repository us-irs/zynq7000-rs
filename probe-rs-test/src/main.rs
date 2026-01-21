use std::{path::Path, time::Duration};

use anyhow::Context;
use probe_rs::{
    Permissions,
    architecture::arm::{FullyQualifiedApAddress, core::registers::cortex_m::PC},
    flashing::{ElfOptions, Format, build_loader},
    probe::list::Lister,
    vendor::amd::sequences::x7z::AccessPort,
};

fn main() -> anyhow::Result<()> {
    let lister = Lister::new();
    let probes = lister.list_all();
    let probe = probes[0].open()?;
    let mut session = probe.attach("X7Z", Permissions::default()).unwrap();

    //mem_ap.
    let mut core = session.core(0)?;
    core.halt(Duration::from_millis(200))?;
    drop(core);
    let binary = Path::new("../firmware/target/armv7a-none-eabihf/release/defmt");
    let format = Format::Elf(ElfOptions::default());
    let loader = build_loader(
        &mut session,
        binary,
        format,
        Some(probe_rs::InstructionSet::A32),
    )?;
    let raw_builder = loader.flash_builder();
    for segment in &raw_builder.data {
        println!("{} bytes of data at base {:?}", segment.1.len(), segment.0);
    }
    let aps = session
        .get_arm_interface()?
        .access_ports(probe_rs::architecture::arm::dp::DpAddress::Default)?;
    println!("APs: {:?}", aps);
    // Using the debug memory here makes a later call to core(0) fail (????).
    let ap_addr = FullyQualifiedApAddress::v1_with_default_dp(AccessPort::SystemMemory as u8);
    let mut mem_ap = session.get_arm_interface()?.memory_interface(&ap_addr)?;
    for segment in &raw_builder.data {
        println!(
            "writing {} bytes of data at base {:?}",
            segment.1.len(),
            segment.0
        );
        mem_ap.write(*segment.0, segment.1)?;
    }
    drop(mem_ap);

    let mut core = session
        .core(0)
        .with_context(|| "retrieving core 0 failed")?;
    println!("running core");
    core.write_core_reg(PC.id, loader.vector_table_addr().unwrap())
        .with_context(|| "setting PC")?;
    core.run().with_context(|| "resuming core")?;
    loop {
        std::thread::sleep(Duration::from_millis(100));
    }
}
