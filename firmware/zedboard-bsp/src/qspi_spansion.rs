use core::cell::RefCell;

use arbitrary_int::{prelude::*, u24};
use zynq7000_hal::qspi::{
    FIFO_DEPTH, LinearQspiConfig, MAX_BYTES_PER_TRANSFER_IO_MODE, QspiIoMode, QspiIoTransferGuard,
    QspiLinearAddressing, QspiLinearReadGuard,
};

pub const QSPI_DEV_COMBINATION_REV_F: zynq7000_hal::qspi::QspiDeviceCombination =
    zynq7000_hal::qspi::QspiDeviceCombination {
        vendor: zynq7000_hal::qspi::QspiVendor::WinbondAndSpansion,
        operating_mode: zynq7000_hal::qspi::OperatingMode::FastReadQuadOutput,
        two_devices: false,
    };

#[derive(Debug, Clone, Copy)]
pub enum RegisterId {
    /// WRR
    WriteRegisters = 0x01,
    /// PP
    PageProgram = 0x02,
    /// READ
    Read = 0x03,
    /// WRDI
    WriteDisable = 0x04,
    /// RDSR1
    ReadStatus1 = 0x05,
    /// RDSR2
    ReadStatus2 = 0x07,
    /// WREN
    WriteEnable = 0x06,
    /// FAST_READ
    FastRead = 0x0B,
    /// SE
    SectorErase = 0xD8,
    /// CSLR
    ClearStatus = 0x30,
    /// RDCR
    ReadConfig = 0x35,
    /// RDID
    ReadId = 0x9F,
}

#[derive(Debug, Clone, Copy, num_enum::TryFromPrimitive)]
#[repr(u8)]
pub enum MemoryInterfaceType {
    _128Mb = 0x20,
    _256Mb = 0x02,
}

#[derive(Debug, Clone, Copy, num_enum::TryFromPrimitive)]
#[repr(u8)]
pub enum Density {
    _128Mb = 0x18,
    _256Mb = 0x19,
}

#[derive(Debug, Clone, Copy, num_enum::TryFromPrimitive)]
#[repr(u8)]
pub enum SectorArchictecture {
    /// Uniform 256 kB sectors.
    Uniform = 0x00,
    /// 32 4kB sectors and 64 kB sectors.
    Hybrid = 0x01,
}

pub const PAGE_SIZE: usize = 0x100;
pub const SECTOR_SIZE: usize = 0x10000;

#[derive(Debug, Clone, Copy)]
pub struct BaseDeviceId {
    manufacturer_id: u8,
    device_id: u16,
}

impl BaseDeviceId {
    #[inline]
    pub const fn new(manufacturer_id: u8, device_id: u16) -> Self {
        BaseDeviceId {
            manufacturer_id,
            device_id,
        }
    }

    #[inline]
    pub const fn from_raw(raw: &[u8; 3]) -> Self {
        BaseDeviceId::new(raw[0], ((raw[1] as u16) << 8) | raw[2] as u16)
    }

    #[inline]
    pub const fn manufacturer_id(&self) -> u8 {
        self.manufacturer_id
    }

    #[inline]
    pub const fn device_id_raw(&self) -> u16 {
        self.device_id
    }

    #[inline]
    pub fn memory_interface_type(&self) -> Result<MemoryInterfaceType, u8> {
        MemoryInterfaceType::try_from(((self.device_id >> 8) & 0xff) as u8).map_err(|e| e.number)
    }

    #[inline]
    pub fn density(&self) -> Result<Density, u8> {
        Density::try_from((self.device_id & 0xff) as u8).map_err(|e| e.number)
    }
}

#[derive(Debug)]
pub struct ExtendedDeviceId {
    base: BaseDeviceId,
    id_cfi_len: u8,
    sector_arch: u8,
    family_id: u8,
    model_number: [u8; 2],
}

impl ExtendedDeviceId {
    pub const fn from_raw(raw: &[u8; 8]) -> Self {
        ExtendedDeviceId {
            base: BaseDeviceId::from_raw(&[raw[0], raw[1], raw[2]]),
            id_cfi_len: raw[3],
            sector_arch: raw[4],
            family_id: raw[5],
            model_number: [raw[6], raw[7]],
        }
    }

    pub const fn id_cfi_len(&self) -> u8 {
        self.id_cfi_len
    }

    pub const fn sector_arch_raw(&self) -> u8 {
        self.sector_arch
    }

    pub fn sector_arch(&self) -> Result<SectorArchictecture, u8> {
        SectorArchictecture::try_from(self.sector_arch_raw()).map_err(|e| e.number)
    }

    pub const fn family_id(&self) -> u8 {
        self.family_id
    }

    pub const fn model_number_raw(&self) -> &[u8; 2] {
        &self.model_number
    }

    pub const fn model_number(&self) -> [char; 2] {
        [self.model_number[0] as char, self.model_number[1] as char]
    }
}

impl ExtendedDeviceId {
    #[inline]
    pub fn base_id(&self) -> BaseDeviceId {
        self.base
    }
}

#[bitbybit::bitfield(u8, debug)]
pub struct StatusRegister1 {
    #[bit(7, rw)]
    status_register_write_disable: bool,
    #[bit(6, r)]
    programming_error: bool,
    #[bit(5, r)]
    erase_error: bool,
    #[bits(2..=4, rw)]
    block_protection: u3,
    #[bit(1, r)]
    write_enable_latch: bool,
    #[bit(0, r)]
    write_in_progress: bool,
}

#[bitbybit::bitfield(u8, debug)]
pub struct ConfigRegister1 {
    #[bits(6..=7, rw)]
    latency_code: u2,
    /// This is an OTP bit. It can not be set back to 0 once it has been set to 1!
    #[bit(5, rw)]
    tbprot: bool,
    #[bit(3, rw)]
    bpnv: bool,
    /// This is an OTP bit. It can not be set back to 0 once it has been set to 1!
    #[bit(2, rw)]
    tbparm: bool,
    #[bit(1, rw)]
    quad: bool,
    #[bit(0, rw)]
    freeze: bool,
}

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum AddrError {
    #[error("address out of range")]
    OutOfRange,
    #[error("address not aligned")]
    Alignment,
}

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum EraseError {
    #[error("erase error bit set in status register")]
    EraseErrorBitSet,
    #[error("address error: {0}")]
    Addr(#[from] AddrError),
}

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum ProgramPageError {
    #[error("programming error bit set in status register")]
    ProgrammingErrorBitSet,
    #[error("address error: {0}")]
    Addr(#[from] AddrError),
    #[error("data is larger than page size {PAGE_SIZE}")]
    DataLargerThanPage,
}

#[derive(Default, Debug, PartialEq, Eq)]
pub struct Config {
    pub set_quad_bit_if_necessary: bool,
    pub latency_config: Option<u2>,
    pub clear_write_protection: bool,
}

impl Config {
    pub fn sr_or_cr_update_possibly_required(&self) -> bool {
        self.set_quad_bit_if_necessary
            || self.latency_config.is_some()
            || self.clear_write_protection
    }
}

pub struct QspiSpansionS25Fl256SIoMode(RefCell<QspiIoMode>);

impl QspiSpansionS25Fl256SIoMode {
    pub fn new(qspi: QspiIoMode, config: Config) -> Self {
        let mut spansion_qspi = QspiSpansionS25Fl256SIoMode(RefCell::new(qspi));
        spansion_qspi.clear_status();
        let mut write_required = false;
        if config.sr_or_cr_update_possibly_required() {
            let mut cr1 = spansion_qspi.read_configuration_register();
            if config.set_quad_bit_if_necessary && !cr1.quad() {
                cr1.set_quad(true);
                write_required = true;
            }
            if let Some(latency_config) = config.latency_config
                && cr1.latency_code() != latency_config
            {
                cr1.set_latency_code(latency_config);
                write_required = true;
            }
            // Preserve the status register by reading it first.
            let mut sr1 = spansion_qspi.read_status_register_1();
            if config.clear_write_protection && sr1.block_protection() != u3::ZERO {
                sr1.set_status_register_write_disable(false);
                sr1.set_block_protection(u3::ZERO);
                write_required = true;
            }
            if write_required {
                // Safety: Only the QUAD bit was set while all other bits are preserved.
                unsafe {
                    spansion_qspi.write_status_and_config_register(sr1, cr1);
                }
            }
        }
        spansion_qspi
    }

    pub fn into_linear_addressed(
        self,
        config: LinearQspiConfig,
    ) -> QspiSpansionS25Fl256SLinearMode {
        let qspi = self.0.into_inner().into_linear_addressed(config);
        QspiSpansionS25Fl256SLinearMode(qspi)
    }

    pub fn set_write_protection(&mut self, write_protection: u3) {
        unsafe {
            self.modify_status_and_config_register(|mut sr, cr| {
                sr.set_status_register_write_disable(false);
                sr.set_block_protection(write_protection);
                (sr, cr)
            });
        }
    }

    pub fn write_enable(&mut self) {
        let qspi = self.0.get_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_01(RegisterId::WriteEnable as u32);
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        transfer.read_rx_data();
    }

    pub fn write_disable(&mut self) {
        let qspi = self.0.get_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_01(RegisterId::WriteDisable as u32);
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        transfer.read_rx_data();
    }

    /// Write a new value for the status register.
    ///
    /// This API may not be used if the QUAD bit (CR1\[1\]) is set.
    pub fn write_status(&mut self, sr: StatusRegister1) {
        self.write_enable();
        let mut qspi = self.0.borrow_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_10(u32::from_ne_bytes([
            RegisterId::WriteRegisters as u8,
            sr.raw_value(),
            0x00,
            0x00,
        ]));
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        transfer.read_rx_data();
    }

    /// Write a new value for the status register. It is strongly recommended to read both
    /// the status and config register first and preserve all unchanged bits.
    ///
    /// This API must be used if the QUAD bit (CR1\[1\]) is set.
    ///
    /// # Safety
    ///
    /// Misuse of this API does not lead to undefined behavior. However, it writes the
    /// configuration register, which is OTP bits. Changing these bits from 0 to 1 is an
    /// irreversible operation.
    pub unsafe fn modify_status_and_config_register(
        &mut self,
        f: impl FnOnce(StatusRegister1, ConfigRegister1) -> (StatusRegister1, ConfigRegister1),
    ) {
        self.write_enable();
        let mut qspi = self.0.borrow_mut();
        let mut transfer = qspi.transfer_guard();
        let sr1 = self.read_status_register_1();
        let cr1 = self.read_configuration_register();
        let (sr1, cr1) = f(sr1, cr1);
        transfer.write_word_txd_11(u32::from_ne_bytes([
            RegisterId::WriteRegisters as u8,
            sr1.raw_value(),
            cr1.raw_value(),
            0x00,
        ]));
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        transfer.read_rx_data();
    }

    /// Write a new value for the status register. It is strongly recommended to read both
    /// the status and config register first and preserve all unchanged bits.
    ///
    /// This API must be used if the QUAD bit (CR1\[1\]) is set.
    ///
    /// # Safety
    ///
    /// Misuse of this API does not lead to undefined behavior. However, it writes the
    /// configuration register, which is OTP bits. Changing these bits from 0 to 1 is an
    /// irreversible operation.
    pub unsafe fn write_status_and_config_register(
        &mut self,
        sr: StatusRegister1,
        cr: ConfigRegister1,
    ) {
        self.write_enable();
        let mut qspi = self.0.borrow_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_11(u32::from_ne_bytes([
            RegisterId::WriteRegisters as u8,
            sr.raw_value(),
            cr.raw_value(),
            0x00,
        ]));
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        transfer.read_rx_data();
    }

    pub fn read_status_register_1(&self) -> StatusRegister1 {
        let mut qspi = self.0.borrow_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_10(RegisterId::ReadStatus1 as u32);
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        let reply = transfer.read_rx_data();
        drop(transfer);
        // little-endian architecture, so the second byte received is the MSB.
        StatusRegister1::new_with_raw_value(((reply >> 24) & 0xff) as u8)
    }

    pub fn read_configuration_register(&self) -> ConfigRegister1 {
        let mut qspi = self.0.borrow_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_10(RegisterId::ReadConfig as u32);
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        let reply = transfer.read_rx_data();
        drop(transfer);
        // little-endian architecture, so the second byte received is the MSB.
        ConfigRegister1::new_with_raw_value(((reply >> 24) & 0xff) as u8)
    }

    pub fn clear_status(&mut self) {
        let qspi = self.0.get_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_01(RegisterId::ClearStatus as u32);
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        transfer.read_rx_data();
    }

    pub fn read_rdid_base(&self) -> BaseDeviceId {
        let mut qspi = self.0.borrow_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_00(RegisterId::ReadId as u32);
        transfer.start();
        while !transfer.read_status().rx_above_threshold() {}
        let reply = transfer.read_rx_data();
        drop(transfer);
        BaseDeviceId::from_raw(reply.to_ne_bytes()[1..].try_into().unwrap())
    }

    pub fn read_rdid_extended(&self) -> ExtendedDeviceId {
        let mut reply: [u8; 12] = [0; 12];
        let mut qspi = self.0.borrow_mut();
        let mut transfer = qspi.transfer_guard();
        transfer.write_word_txd_00(RegisterId::ReadId as u32);
        transfer.write_word_txd_00(0x00);
        transfer.write_word_txd_00(0x00);
        transfer.start();
        let mut read_index = 0;

        while read_index < 3 {
            if transfer.read_status().rx_above_threshold() {
                reply[read_index * 4..(read_index + 1) * 4]
                    .copy_from_slice(&transfer.read_rx_data().to_ne_bytes());
                read_index += 1;
            }
        }
        ExtendedDeviceId::from_raw(reply[1..9].try_into().unwrap())
    }

    /// This function will block until the operation has completed.
    pub fn erase_sector(&mut self, addr: u32) -> Result<(), EraseError> {
        if addr + SECTOR_SIZE as u32 > u24::MAX.as_u32() {
            return Err(AddrError::OutOfRange.into());
        }
        if !addr.is_multiple_of(SECTOR_SIZE as u32) {
            return Err(AddrError::Alignment.into());
        }
        self.write_enable();
        let qspi = self.0.get_mut();
        let mut transfer = qspi.transfer_guard();
        let raw_word: [u8; 4] = [
            RegisterId::SectorErase as u8,
            ((addr >> 16) & 0xff) as u8,
            ((addr >> 8) & 0xff) as u8,
            (addr & 0xff) as u8,
        ];
        transfer.write_word_txd_00(u32::from_ne_bytes(raw_word));
        transfer.start();

        // Finish transfer
        while !transfer.read_status().rx_above_threshold() {}
        transfer.read_rx_data();

        // Drive CS high to initiate the sector erase operation.
        drop(transfer);

        // Now poll for completion.
        loop {
            let rdsr1 = self.read_status_register_1();
            if rdsr1.erase_error() {
                // The datasheet mentions that the status should be cleared and writes
                // should be disabled explicitely.
                self.clear_status();
                self.write_disable();
                return Err(EraseError::EraseErrorBitSet);
            }
            if !rdsr1.write_in_progress() {
                return Ok(());
            }
        }
    }

    pub fn write_pages(&mut self, mut addr: u32, data: &[u8]) -> Result<(), ProgramPageError> {
        if addr + data.len() as u32 > u24::MAX.as_u32() {
            return Err(AddrError::OutOfRange.into());
        }
        if !addr.is_multiple_of(PAGE_SIZE as u32) {
            return Err(AddrError::Alignment.into());
        }
        for chunk in data.chunks(PAGE_SIZE) {
            self.program_page(addr, chunk)?;
            addr += PAGE_SIZE as u32;
        }
        Ok(())
    }

    /// This function also takes care of enabling writes before programming the page.
    /// This function will block until the operation has completed.
    ///
    /// The data length max not exceed the page size [PAGE_SIZE].
    pub fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), ProgramPageError> {
        if addr + data.len() as u32 > u24::MAX.as_u32() {
            return Err(AddrError::OutOfRange.into());
        }
        if !addr.is_multiple_of(PAGE_SIZE as u32) {
            return Err(AddrError::Alignment.into());
        }
        if data.len() > PAGE_SIZE {
            return Err(ProgramPageError::DataLargerThanPage);
        }
        self.write_enable();
        let qspi = self.0.get_mut();
        let mut transfer = qspi.transfer_guard();
        let raw_word: [u8; 4] = [
            RegisterId::PageProgram as u8,
            ((addr >> 16) & 0xff) as u8,
            ((addr >> 8) & 0xff) as u8,
            (addr & 0xff) as u8,
        ];
        transfer.write_word_txd_00(u32::from_ne_bytes(raw_word));
        let mut read_index: u32 = 0;
        let mut current_byte_index = 0;
        let fifo_writes = data.len().div_ceil(4);
        // Fill the FIFO until it is full.
        for _ in 0..core::cmp::min(fifo_writes, FIFO_DEPTH - 1) {
            transfer.write_word_txd_00(u32::from_ne_bytes(
                data[current_byte_index..current_byte_index + 4]
                    .try_into()
                    .unwrap(),
            ));
            current_byte_index += 4;
        }
        transfer.start();

        let mut wait_for_tx_slot = |transfer: &mut QspiIoTransferGuard| loop {
            let status_read = transfer.read_status();
            // Double read to avoid RX underflows as specified in TRM.
            if status_read.rx_above_threshold() && transfer.read_status().rx_above_threshold() {
                transfer.read_rx_data();
                read_index = read_index.wrapping_add(4);
            }
            if !status_read.tx_full() {
                break;
            }
        };

        while current_byte_index < data.len() {
            // Immediately fill the FIFO again with the remaining 8 bytes.
            wait_for_tx_slot(&mut transfer);

            let word = match core::cmp::min(4, data.len() - current_byte_index) {
                1 => {
                    let mut bytes = [0; 4];
                    bytes[0] = data[current_byte_index];
                    u32::from_ne_bytes(bytes)
                }
                2 => {
                    let mut bytes = [0; 4];
                    bytes[0..2].copy_from_slice(&data[current_byte_index..current_byte_index + 2]);
                    u32::from_ne_bytes(bytes)
                }
                3 => {
                    let mut bytes = [0; 4];
                    bytes[0..3].copy_from_slice(&data[current_byte_index..current_byte_index + 3]);
                    u32::from_ne_bytes(bytes)
                }
                4 => u32::from_ne_bytes(
                    data[current_byte_index..current_byte_index + 4]
                        .try_into()
                        .unwrap(),
                ),
                _ => unreachable!(),
            };
            transfer.write_word_txd_00(word);
            current_byte_index += 4;
        }

        while read_index < data.len() as u32 {
            // Double read to avoid RX underflows as specified in TRM.
            let status_read = transfer.read_status();
            if status_read.rx_above_threshold() && transfer.read_status().rx_above_threshold() {
                transfer.read_rx_data();
                read_index = read_index.wrapping_add(4);
            }
        }
        drop(transfer);

        // Now poll for completion.
        loop {
            let rdsr1 = self.read_status_register_1();
            if rdsr1.programming_error() {
                // The datasheet mentions that the status should be cleared and writes
                // should be disabled explicitely.
                self.clear_status();
                self.write_disable();
                return Err(ProgramPageError::ProgrammingErrorBitSet);
            }
            if !rdsr1.write_in_progress() {
                return Ok(());
            }
        }
    }

    fn generic_read_page(&self, addr: u32, buf: &mut [u8], dummy_byte: bool, fast_read: bool) {
        let mut offset = 0;
        let reg_id = if fast_read {
            RegisterId::FastRead
        } else {
            RegisterId::Read
        };
        let mut qspi = self.0.borrow_mut();
        let mut max_chunk_size = MAX_BYTES_PER_TRANSFER_IO_MODE - 4;
        if dummy_byte {
            max_chunk_size -= 1;
        }

        while offset < buf.len() {
            // Calculate the size of the current chunk (max 248 bytes)
            let chunk_size = core::cmp::min(max_chunk_size, buf.len() - offset);
            let current_addr = addr + offset as u32;

            // Create a mutable slice for the current chunk
            let chunk_slice = &mut buf[offset..offset + chunk_size];

            // This ensures the hardware transaction (Chip Select, etc.) restarts for each chunk.
            {
                let mut transfer = qspi.transfer_guard();

                let raw_word: [u8; 4] = [
                    reg_id as u8,
                    ((current_addr >> 16) & 0xff) as u8,
                    ((current_addr >> 8) & 0xff) as u8,
                    (current_addr & 0xff) as u8,
                ];
                transfer.write_word_txd_00(u32::from_ne_bytes(raw_word));

                let mut read_index = 0;
                let mut written_words = 0;
                // Use chunk_size instead of the full buffer length
                let mut bytes_to_write = chunk_size;

                if dummy_byte {
                    bytes_to_write += 1;
                }
                let fifo_writes = bytes_to_write.div_ceil(4);

                // Fill the FIFO until it is full or all 0 bytes have been written.
                for _ in 0..core::cmp::min(fifo_writes, FIFO_DEPTH - 1) {
                    transfer.write_word_txd_00(0);
                    written_words += 1;
                }

                transfer.start();
                let mut reply_word_index = 0;

                // Loop based on the current chunk's size
                while read_index < chunk_size {
                    let rx_is_above_threshold = transfer.read_status().rx_above_threshold();

                    // See p.374 of the TRM: Do a double read to ensure this is correct information.
                    if rx_is_above_threshold && transfer.read_status().rx_above_threshold() {
                        let reply = transfer.read_rx_data();
                        if reply_word_index == 0 {
                            reply_word_index += 1;
                            continue;
                        }
                        let reply_as_bytes = reply.to_ne_bytes();
                        // Calculate remaining bytes in this specific chunk
                        let reply_size = core::cmp::min(chunk_size - read_index, 4);
                        read_index += match (reply_size, reply_word_index == 1 && dummy_byte) {
                            (1, false) => {
                                chunk_slice[read_index] = reply_as_bytes[0];
                                1
                            }
                            (1, true) => {
                                chunk_slice[read_index] = reply_as_bytes[1];
                                1
                            }
                            (2, false) => {
                                chunk_slice[read_index..read_index + 2]
                                    .copy_from_slice(&reply_as_bytes[0..2]);
                                2
                            }
                            (2, true) => {
                                chunk_slice[read_index..read_index + 2]
                                    .copy_from_slice(&reply_as_bytes[1..3]);
                                2
                            }
                            (3, false) => {
                                chunk_slice[read_index..read_index + 3]
                                    .copy_from_slice(&reply_as_bytes[0..3]);
                                3
                            }
                            (3, true) => {
                                chunk_slice[read_index..read_index + 3]
                                    .copy_from_slice(&reply_as_bytes[1..4]);
                                3
                            }
                            (4, false) => {
                                chunk_slice[read_index..read_index + 4]
                                    .copy_from_slice(&reply_as_bytes[0..4]);
                                4
                            }
                            (4, true) => {
                                chunk_slice[read_index..read_index + 3]
                                    .copy_from_slice(&reply_as_bytes[1..4]);
                                3
                            }
                            _ => unreachable!(),
                        };
                        reply_word_index += 1;
                    }
                    if written_words < fifo_writes && !transfer.read_status().tx_full() {
                        transfer.write_word_txd_00(0);
                        written_words += 1;
                    }
                }
            }

            offset += chunk_size;
        }
    }

    pub fn read_page_fast_read(&self, addr: u32, buf: &mut [u8], dummy_byte: bool) {
        self.generic_read_page(addr, buf, dummy_byte, true)
    }

    /// Only works if the clock speed is slower than 50 MHz according to datasheet.
    pub fn read_page_read(&self, addr: u32, buf: &mut [u8]) {
        self.generic_read_page(addr, buf, false, false)
    }
}

/// If the Spansion QSPI is used in linear addressed mode, no IO operations are allowed.
pub struct QspiSpansionS25Fl256SLinearMode(QspiLinearAddressing);

impl QspiSpansionS25Fl256SLinearMode {
    pub const BASE_ADDR: usize = QspiLinearAddressing::BASE_ADDRESS;
    pub const PAGE_SIZE: usize = PAGE_SIZE;
    pub const SECTOR_SIZE: usize = SECTOR_SIZE;

    pub fn into_io_mode(self, dual_flash: bool) -> QspiSpansionS25Fl256SIoMode {
        let qspi = self.0.into_io_mode(dual_flash);
        QspiSpansionS25Fl256SIoMode::new(qspi, Config::default())
    }

    pub fn read_guard(&mut self) -> QspiLinearReadGuard<'_> {
        self.0.read_guard()
    }
}
