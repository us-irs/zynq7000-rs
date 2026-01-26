use std::{
    path::Path,
    time::{Duration, Instant},
};

use probe_rs::{
    Permissions,
    architecture::arm::FullyQualifiedApAddress,
    flashing::{self, DownloadOptions, ElfOptions, Format},
    probe::{WireProtocol, list::Lister},
    rtt,
    vendor::amd::sequences::x7z::AccessPort,
};
use tracing_subscriber::EnvFilter;

fn main() -> anyhow::Result<()> {
    // Initialize the subscriber with an environment filter.
    // This allows you to control log levels using the `RUST_LOG` environment variable.
    tracing_subscriber::fmt()
        .with_env_filter(
            // Reads the RUST_LOG environment variable.
            // Defaults to "info" if not set.
            EnvFilter::from_default_env(),
        )
        .init();

    let lister = Lister::new();
    let probes = lister.list_all();
    let mut probe = probes[0].open()?;
    //let actual = probe.set_speed(10000)?;
    probe.select_protocol(WireProtocol::Jtag).unwrap();
    //println!("probe speed: {}", actual);
    let mut session = probe.attach("X7Z", Permissions::default()).unwrap();
    let elf_path = Path::new("../firmware/target/armv7a-none-eabihf/release/defmt");
    let format = Format::Elf(ElfOptions::default());
    let mut download_opts = DownloadOptions::default();
    download_opts.verify = true;
    download_opts.skip_reset = true;

    //session.get_arm_interface()?.read_dp_register(dp)

    println!("flashing ELF file");
    let loader = flashing::build_loader(&mut session, elf_path, format.clone(), None)?;

    let elf_file = std::fs::read(elf_path).unwrap();
    let rtt_addr = rtt::find_rtt_control_block_in_elf(&elf_file)?.unwrap();
    let vector_table_addr = loader.vector_table_addr().unwrap();

    loader.commit(&mut session, download_opts)?;

    /*

        //probe_rs::util::rtt::
        println!("RTT addr: {:#x}", rtt_addr);

        let loader = flashing::build_loader(
            &mut session,
            elf_path,
            format,
            None
        )?;

        let ap_addr = FullyQualifiedApAddress::v1_with_default_dp(AccessPort::SystemMemory as u8);
        let mut mem_ap = session.get_arm_interface()?.memory_interface(&ap_addr)?;
        let raw_builder = loader.flash_builder();
        for segment in &raw_builder.data {
            let now = Instant::now();
            println!(
                "writing {} bytes of data at base {:?}",
                segment.1.len(),
                segment.0
            );
            mem_ap.write(*segment.0, segment.1)?;
            println!("elapsed {:?}", now.elapsed());
        }
        drop(mem_ap);
    */

    // Select a core.
    //println!("attaching RTT");

    println!("running core");

    let mut core = session.core(0)?;
    let mut rtt = rtt::Rtt::attach_at(&mut core, rtt_addr)?;
    core.prepare_running_on_ram(vector_table_addr)?;
    core.run()?;


    let defmt_table = defmt_decoder::Table::parse(&elf_file)?.unwrap();
    let mut stream_decoder = defmt_table.new_stream_decoder();

    // Read from a channel
    loop {
        for channel in rtt.up_channels() {
            let mut buf = [0u8; 1024];
            let count = channel.read(&mut core, &mut buf[..])?;

            if count > 0 {
                stream_decoder.received(&buf[..count]);
                // decode the received data
                match stream_decoder.decode() {
                    Ok(frame) => {
                        println!("defmt frame: {}", frame.display_message())
                    }
                    Err(defmt_decoder::DecodeError::UnexpectedEof) => {
                        println!("unexpected EOF");
                    }
                    Err(defmt_decoder::DecodeError::Malformed) => {
                        println!("malformed defmt frame");
                    }
                }
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    //mem_ap.
    /*
    let mut core = session.core(0)?;
    core.halt(Duration::from_millis(200))?;
    drop(core);
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
    */
    /*
    loop {
                for up_ch in rtt.up_channels() {
                    let read_bytes = up_ch.read(&mut core, &mut buf)?;
                    if read_bytes > 0 {
                        println!("read bytes {} on UP channel", read_bytes);
                    }
                }
    }
    */
}
