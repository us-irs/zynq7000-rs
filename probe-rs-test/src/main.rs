use std::{path::Path, time::Duration};

use probe_rs::{
    Permissions,
    architecture::arm::FullyQualifiedApAddress,
    flashing::{ElfOptions, Format, build_loader},
    probe::list::Lister,
    vendor::amd::sequences::x7z::AccessPort,
};

fn main() -> anyhow::Result<()> {
    let lister = Lister::new();
    let probes = lister.list_all();
    let probe = probes[0].open()?;
    let mut session = probe.attach("X7Z", Permissions::default()).unwrap();

    let ap_addr = FullyQualifiedApAddress::v1_with_default_dp(AccessPort::SystemMemory as u8);
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
    let mut core = session.core(0)?;
    println!("running core");
    core.run()?;
    loop {
        std::thread::sleep(Duration::from_millis(100));
    }
}
