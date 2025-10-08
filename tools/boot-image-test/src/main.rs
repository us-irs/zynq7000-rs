//! Small tester app to verify some of the API offers by the zynq-boot-image crate.
use std::{io::Read, path::Path};

use clap::Parser as _;
use zynq7000_boot_image::{BootHeader, FIXED_BOOT_HEADER_SIZE};

#[derive(clap::Parser, Debug)]
#[command(version, about)]
pub struct Cli {
    /// Path to boot.bin file to test.
    #[arg(short, long)]
    path: String,
}

fn main() {
    let cli = Cli::parse();
    let boot_bin = Path::new(&cli.path);
    if !boot_bin.exists() {
        eprintln!("File not found: {}", boot_bin.display());
        std::process::exit(1);
    }
    let mut boot_bin_file = std::fs::File::open(boot_bin).expect("failed to open boot.bin file");
    let mut header_buf = Box::new([0u8; 8192]);
    boot_bin_file
        .read_exact(&mut header_buf[0..FIXED_BOOT_HEADER_SIZE])
        .expect("failed to read boot header");
    let mut boot_header = BootHeader::new(&header_buf[0..FIXED_BOOT_HEADER_SIZE])
        .expect("failed to parse boot header");
    let source_offset = boot_header.source_offset();
    boot_bin_file
        .read_exact(&mut header_buf[FIXED_BOOT_HEADER_SIZE..source_offset - FIXED_BOOT_HEADER_SIZE])
        .expect("failed to read full boot binary metadata");
    // Re-assign with newly read data.
    boot_header = BootHeader::new_unchecked(&header_buf[0..source_offset]);
    let image_header_table = boot_header
        .image_header_table()
        .expect("failed extracting image header table");
    let image_headers = image_header_table.count_of_headers();
    println!(
        "Image headers: {}, first image header offset {}, first partition header offset {}",
        image_headers,
        image_header_table
            .first_image_header_offset()
            .expect("failed reading first image header offset"),
        image_header_table
            .first_partition_header_offset()
            .expect("failed reading first partition header offset")
    );

    let image_header_iter = boot_header
        .image_header_iterator()
        .expect("failed extracting boot header iterator");
    for (idx, image_header) in image_header_iter.enumerate() {
        println!("--------------------------------------");
        println!(
            "Image header {} with partition count {}",
            idx,
            image_header.partition_count()
        );
        let mut test: [u8; 64] = [0; 64];
        let image_name = image_header
            .image_name(&mut test)
            .expect("image name error");
        println!("image name: {}", image_name);
        let partition_iter = image_header
            .partition_header_iterator(header_buf.as_slice())
            .unwrap();
        if image_header.partition_count() > 0 {
            println!("--------------------------------------");
        }
        for partition in partition_iter {
            println!(
                "partition with size {} and load address {:#08x}, section count {}",
                partition.total_partition_length().unwrap(),
                partition.destination_load_address(),
                partition.section_count()
            );
            println!("section attributes: {:?}", partition.section_attributes());
        }
        println!("--------------------------------------\n\r");
    }
}
