//! # Device Configuration Module
#[derive(Debug, thiserror::Error)]
#[error("unaligned address: {0}")]
pub struct UnalignedAddrError(usize);

/// Configures the bitstream using the PCAP interface in non-secure mode.
///
/// Blocking function which only returns when the bitstream configuration is complete.
pub fn configure_bitstream_non_secure(
    init_pl: bool,
    bitstream: &[u8],
) -> Result<(), UnalignedAddrError> {
    if !(bitstream.as_ptr() as usize).is_multiple_of(64) {
        return Err(UnalignedAddrError(bitstream.as_ptr() as usize));
    }
    if bitstream.is_empty() {
        return Ok(());
    }
    let mut devcfg = unsafe { zynq7000::devcfg::Registers::new_mmio_fixed() };
    devcfg.modify_control(|mut val| {
        val.set_config_access_select(zynq7000::devcfg::PlConfigAccess::ConfigAccessPort);
        val.set_access_port_select(zynq7000::devcfg::ConfigAccessPortSelect::Pcap);
        val
    });
    devcfg.write_interrupt_status(zynq7000::devcfg::Interrupt::new_with_raw_value(0xFFFF_FFFF));
    if init_pl {
        devcfg.modify_control(|mut val| {
            val.set_prog_b_bit(true);
            val
        });
        devcfg.modify_control(|mut val| {
            val.set_prog_b_bit(false);
            val
        });
        while devcfg.read_status().pcfg_init() {}
        devcfg.modify_control(|mut val| {
            val.set_prog_b_bit(true);
            val
        });
        devcfg.write_interrupt_status(
            zynq7000::devcfg::Interrupt::ZERO.with_pl_programming_done(true),
        );
    }
    while !devcfg.read_status().pcfg_init() {}
    if !init_pl {
        while devcfg.read_status().dma_command_queue_full() {}
    }
    devcfg.modify_misc_control(|mut val| {
        val.set_loopback(false);
        val
    });
    devcfg.modify_control(|mut val| {
        val.set_pcap_rate_enable(false);
        val
    });

    // As specified in the TMR,
    // Setting the two LSBs of the source and destination address to 2'b01 indicates to the DevC
    // DMA module the last DMA command of an overall transfer
    devcfg.write_dma_source_addr(bitstream.as_ptr() as u32 | 0b01);
    devcfg.write_dma_dest_addr(0xFFFF_FFFF);
    devcfg.write_dma_source_len(bitstream.len() as u32 / 4);
    devcfg.write_dma_dest_len(bitstream.len() as u32 / 4);

    while !devcfg.read_interrupt_status().dma_done() {}
    // TODO: Check for errors.
    while !devcfg.read_interrupt_status().pl_programming_done() {}
    Ok(())
}
