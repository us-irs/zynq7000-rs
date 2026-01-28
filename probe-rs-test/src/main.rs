use std::{path::Path, time::Duration};

use probe_rs::{
    Permissions,
    architecture::arm::dp::DpAddress,
    flashing::{self, DownloadOptions, ElfOptions, Format},
    probe::{WireProtocol, list::Lister},
    rtt,
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
    probe.select_protocol(WireProtocol::Jtag).unwrap();

    let mut session = probe.attach("X7Z", Permissions::default()).unwrap();
    println!("target: {:?}", session.target().memory_ports);
    let arm_if = session.get_arm_interface()?;
    arm_if.select_debug_port(DpAddress::Default)?;
    let elf_path = Path::new("../firmware/target/armv7a-none-eabihf/release/defmt");
    let format = Format::Elf(ElfOptions::default());
    let mut download_opts = DownloadOptions::default();
    download_opts.verify = true;
    download_opts.skip_reset = true;

    println!("flashing ELF file");
    let loader = flashing::build_loader(&mut session, elf_path, format.clone(), None)?;

    let elf_file = std::fs::read(elf_path).unwrap();
    let rtt_addr = rtt::find_rtt_control_block_in_raw_file(&elf_file)?.unwrap();
    let vector_table_addr = loader.vector_table_addr().unwrap();

    loader.commit(&mut session, download_opts)?;

    println!("attaching RTT");

    let mut memory = session.memory_access_port(0)?;
    let mut rtt = rtt::Rtt::attach_at(&mut memory, rtt_addr)?;
    drop(memory);
    session.prepare_running_on_ram(vector_table_addr, 0)?;
    let mut core = session.core(0)?;

    println!("running core");
    core.run()?;

    let defmt_table = defmt_decoder::Table::parse(&elf_file)?.unwrap();
    let mut stream_decoder = defmt_table.new_stream_decoder();
    drop(core);

    // Read from a channel
    loop {
        let mut memory = session.memory_access_port(0)?;
        for channel in rtt.up_channels() {
            let mut buf = [0u8; 1024];
            let count = channel.read(&mut memory, &mut buf[..])?;

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
}
