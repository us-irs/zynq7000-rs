use std::{path::Path, time::Duration};

use anyhow::Context;
use probe_rs::{
    MemoryInterface, Permissions,
    architecture::arm::{FullyQualifiedApAddress, core::registers::cortex_m::PC},
    flashing::{ElfOptions, Format, build_loader},
    probe::list::Lister,
    rtt::{Rtt, RttChannel, ScanRegion},
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
    println!("trying to find CB");
    let range = vec![(0x100000_u64..0x110000)];
    let rtt_cb = Rtt::find_contol_block(&mut core, &ScanRegion::Ranges(range))?;
    println!("RTT CB: {:#x}", rtt_cb);
    let mut rtt = Rtt::attach_at(&mut core, rtt_cb)?;
    let mut buf: [u8; 1024] = [0; 1024];
    core.read(rtt_cb, &mut buf[0..12])?;
    println!("RTT CB: {:#x?}", &buf[0..12]);
    loop {
        /*
                for up_ch in rtt.up_channels() {
                    let read_bytes = up_ch.read(&mut core, &mut buf)?;
                    if read_bytes > 0 {
                        println!("read bytes {} on UP channel", read_bytes);
                    }
                }
        */
        std::thread::sleep(Duration::from_millis(100));
    }
}
