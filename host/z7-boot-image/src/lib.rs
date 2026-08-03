#![no_std]

use core::str::Utf8Error;

/// ASCII 'XLNX'
pub const IMAGE_ID_U32: u32 = 0x584C4E58;

/// This is the fixed size of the boot header.
pub const FIXED_BOOT_HEADER_SIZE: usize = 0xA0;

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum InvalidBootHeader {
    #[error("image ID invalid")]
    ImageIdInvalid,
    #[error("checksum is invalid")]
    ChecksumInvalid,
    #[error("provided data slice too small")]
    DataTooSmall,
}

pub struct BootHeader<'a> {
    //base_addr: usize,
    // Snapshot of the boot header and following data (at least fixed header size)
    data: &'a [u8],
}

impl<'a> BootHeader<'a> {
    pub const FIXED_SIZED_PART: usize = FIXED_BOOT_HEADER_SIZE;

    /// Create a new boot header parser structure without performing any additional checks.
    pub const fn new_unchecked(data: &'a [u8]) -> Self {
        BootHeader { data }
    }

    /// Create a new boot header parser structure while also performing any additonal checks.
    ///
    /// The passed buffer must have a minimal size of [Self::FIXED_SIZED_PART].
    /// This constructor calls [Self::check_image_id_validity] and [Self::verify_header_checksum]
    /// to verify whether the boot header structure is actually valid.
    pub fn new(data: &'a [u8]) -> Result<Self, InvalidBootHeader> {
        if data.len() < Self::FIXED_SIZED_PART {
            return Err(InvalidBootHeader::DataTooSmall);
        }
        let header = BootHeader { data };
        if !header.check_image_id_validity() {
            return Err(InvalidBootHeader::ImageIdInvalid);
        }
        if !header.verify_header_checksum() {
            return Err(InvalidBootHeader::ChecksumInvalid);
        }
        Ok(header)
    }

    #[inline]
    fn read_u32_le(&self, offset: usize) -> u32 {
        let bytes = &self.data[offset..offset + 4];
        u32::from_le_bytes(bytes.try_into().unwrap())
    }

    #[inline]
    pub fn image_id(&self) -> u32 {
        self.read_u32_le(0x24)
    }

    /// Check whether the image ID has the mandatory [IMAGE_ID_U32] value.
    #[inline]
    pub fn check_image_id_validity(&self) -> bool {
        self.image_id() == IMAGE_ID_U32
    }

    /// Offset to the FSBL image in bytes. This information can be used to only extract the boot
    /// binary metadata (everything except actual partition data).
    #[inline]
    pub fn source_offset(&self) -> usize {
        self.read_u32_le(0x30) as usize
    }

    #[inline]
    pub fn header_checksum(&self) -> u32 {
        self.read_u32_le(0x48)
    }

    #[inline]
    pub fn image_header_table_offset(&self) -> usize {
        self.read_u32_le(0x98) as usize
    }

    pub fn image_header_table(&self) -> Option<ImageHeaderTable<'a>> {
        let offset = self.image_header_table_offset();
        if offset + ImageHeaderTable::SIZE > self.data.len() {
            return None;
        }
        Some(
            ImageHeaderTable::new(
                &self.data[self.image_header_table_offset()
                    ..self.image_header_table_offset() + ImageHeaderTable::SIZE],
            )
            .unwrap(),
        )
    }

    pub fn image_header_iterator(&self) -> Option<ImageHeaderIterator<'a>> {
        let first_header_offset = self.image_header_table()?.first_image_header_offset()?;
        ImageHeaderIterator::new(self.data, first_header_offset)
    }

    #[inline]
    pub fn partition_header_table_offset(&self) -> usize {
        self.read_u32_le(0x9C) as usize
    }

    #[inline]
    pub fn verify_header_checksum(&self) -> bool {
        let checksum = self.header_checksum();
        let mut sum = 0u32;
        let mut ofs = 0x20;
        while ofs < 0x48 {
            sum = sum.wrapping_add(self.read_u32_le(ofs));
            ofs += 4;
        }
        !sum == checksum
    }
}

pub struct ImageHeaderTable<'a> {
    data: &'a [u8],
}

impl<'a> ImageHeaderTable<'a> {
    pub const SIZE: usize = 0x18;

    pub const fn new(data: &'a [u8]) -> Option<Self> {
        if data.len() != Self::SIZE {
            return None;
        }
        Some(ImageHeaderTable { data })
    }

    #[inline]
    fn read_u32_le(&self, offset: usize) -> u32 {
        let bytes = &self.data[offset..offset + 4];
        u32::from_le_bytes(bytes.try_into().unwrap())
    }

    #[inline]
    pub fn count_of_headers(&self) -> usize {
        self.read_u32_le(0x04) as usize
    }

    /// Returns [None] if the number of words times 4 exeeds [u32::MAX].
    #[inline]
    pub fn first_image_header_offset(&self) -> Option<usize> {
        self.read_u32_le(0x0C).checked_mul(4).map(|v| v as usize)
    }

    /// Returns [None] if the number of words times 4 exeeds [u32::MAX].
    #[inline]
    pub fn first_partition_header_offset(&self) -> Option<usize> {
        self.read_u32_le(0x08).checked_mul(4).map(|v| v as usize)
    }
}

pub struct ImageHeaderIterator<'a> {
    data: &'a [u8],
    current_header_offset: usize,
}

impl<'a> ImageHeaderIterator<'a> {
    #[inline]
    pub const fn new(data: &'a [u8], first_header_offset: usize) -> Option<Self> {
        if first_header_offset + ImageHeader::MIN_SIZE > data.len() {
            return None;
        }
        Some(ImageHeaderIterator {
            data,
            current_header_offset: first_header_offset,
        })
    }
}
impl<'a> Iterator for ImageHeaderIterator<'a> {
    type Item = ImageHeader<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_header_offset == 0 {
            return None;
        }
        let next_image_header = ImageHeader::new(&self.data[self.current_header_offset..])?;
        self.current_header_offset = next_image_header.next_image_header_offset()?;
        Some(next_image_header)
    }
}

pub struct ImageHeader<'a> {
    header_data: &'a [u8],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("buffer too small")]
pub struct BufferTooSmallError;

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum NameParsingError {
    #[error("ut8 error")]
    Utf8(#[from] Utf8Error),
    #[error("buffer too small")]
    BufferTooSmall(#[from] BufferTooSmallError),
}

impl<'a> ImageHeader<'a> {
    pub const MIN_SIZE: usize = 0x1C;

    #[inline]
    pub fn new(data: &'a [u8]) -> Option<Self> {
        if data.len() < Self::MIN_SIZE {
            return None;
        }
        let mut current_offset = 0x14;
        let mut prev_word =
            u32::from_le_bytes(data[current_offset..current_offset + 4].try_into().unwrap());
        current_offset += 4;
        // TODO: Upper bound.
        loop {
            if current_offset + 4 > data.len() {
                return None;
            }
            let current_word =
                u32::from_le_bytes(data[current_offset..current_offset + 4].try_into().unwrap());
            current_offset += 4;
            if current_word == 0xffff_ffff || prev_word == 0x0000_0000 {
                break;
            }
            prev_word = current_word;
        }
        Some(ImageHeader {
            header_data: &data[0..current_offset],
        })
    }

    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.header_data.len()
    }

    #[inline]
    fn read_u32_le(&self, offset: usize) -> u32 {
        let bytes = &self.header_data[offset..offset + 4];
        u32::from_le_bytes(bytes.try_into().unwrap())
    }

    pub fn partition_header_iterator(&self, data: &'a [u8]) -> Option<PartitionHeaderIterator<'a>> {
        let first_partition_header = self.first_partition_header_offset()?;
        Some(PartitionHeaderIterator {
            data,
            current_partition_header_addr: first_partition_header,
            current_partition_index: 0,
            num_of_partitions: self.partition_count(),
        })
    }

    #[inline]
    pub fn next_image_header_offset(&self) -> Option<usize> {
        self.read_u32_le(0x00).checked_mul(4).map(|v| v as usize)
    }

    #[inline]
    pub fn partition_count(&self) -> usize {
        self.read_u32_le(0x0C) as usize
    }

    pub fn first_partition_header_offset(&self) -> Option<usize> {
        self.read_u32_le(0x04).checked_mul(4).map(|v| v as usize)
    }

    pub fn image_name_copied(&self, buf: &mut [u8]) -> Result<usize, BufferTooSmallError> {
        let mut current_offset = 0x10;
        let mut current_buf_idx = 0;
        let mut null_byte_found = false;
        loop {
            let next_bytes = &self.header_data[current_offset..current_offset + 4];
            for &byte in next_bytes.iter().rev() {
                if byte == 0 {
                    null_byte_found = true;
                    break;
                }
                if current_buf_idx >= buf.len() {
                    return Err(BufferTooSmallError);
                }
                buf[current_buf_idx] = byte;
                current_buf_idx += 1;
            }
            if null_byte_found {
                break;
            }
            current_offset += 4;
        }
        Ok(current_buf_idx)
    }

    pub fn image_name<'b>(&self, buf: &'b mut [u8]) -> Result<&'b str, NameParsingError> {
        let name_len = self.image_name_copied(buf)?;
        core::str::from_utf8(&buf[0..name_len]).map_err(NameParsingError::from)
    }
}

pub struct PartitionHeaderIterator<'a> {
    data: &'a [u8],
    current_partition_header_addr: usize,
    current_partition_index: usize,
    num_of_partitions: usize,
}

impl<'a> Iterator for PartitionHeaderIterator<'a> {
    type Item = PartitionHeader<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_partition_index >= self.num_of_partitions {
            return None;
        }
        if self.current_partition_header_addr + PartitionHeader::SIZE > self.data.len() {
            return None;
        }
        let header = PartitionHeader::new(
            &self.data[self.current_partition_header_addr
                ..self.current_partition_header_addr + PartitionHeader::SIZE],
        )
        .ok()?;
        self.current_partition_index += 1;
        self.current_partition_header_addr += 0x40;
        Some(header)
    }
}

pub struct PartitionHeader<'a> {
    header_data: &'a [u8],
}

#[bitbybit::bitenum(u2, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum PartitionOwner {
    Fsbl = 0,
    Uboot = 1,
}

#[bitbybit::bitenum(u3, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ChecksumType {
    None = 0,
    Md5 = 1,
}

#[bitbybit::bitenum(u4, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum DestinationDevice {
    None = 0,
    Ps = 1,
    Pl = 2,
    Int = 3,
}

#[bitbybit::bitfield(u32, debug)]
pub struct SectionAttributes {
    #[bits(16..=17, rw)]
    partition_owner: Option<PartitionOwner>,
    #[bit(15, rw)]
    rsa_signature_present: bool,
    #[bits(12..=14, rw)]
    checksum_type: Option<ChecksumType>,
    #[bits(4..=7, rw)]
    destination_device: Option<DestinationDevice>,
}

impl<'a> PartitionHeader<'a> {
    pub const SIZE: usize = 0x40;

    // TODO: Checksum check.
    #[inline]
    pub const fn new(header_data: &'a [u8]) -> Result<Self, BufferTooSmallError> {
        if header_data.len() < Self::SIZE {
            return Err(BufferTooSmallError);
        }
        Ok(PartitionHeader { header_data })
    }

    #[inline]
    fn read_u32_le(&self, offset: usize) -> u32 {
        let bytes = &self.header_data[offset..offset + 4];
        u32::from_le_bytes(bytes.try_into().unwrap())
    }

    #[inline]
    pub fn encrypted_partition_length(&self) -> u32 {
        self.read_u32_le(0x00)
    }

    #[inline]
    pub fn unencrypted_partition_length(&self) -> u32 {
        self.read_u32_le(0x04)
    }

    #[inline]
    pub fn total_partition_length(&self) -> Option<usize> {
        self.read_u32_le(0x08).checked_mul(4).map(|v| v as usize)
    }

    #[inline]
    pub fn destination_load_address(&self) -> u32 {
        self.read_u32_le(0x0C)
    }

    #[inline]
    pub fn destination_exec_address(&self) -> u32 {
        self.read_u32_le(0x10)
    }

    #[inline]
    pub fn section_attributes(&self) -> SectionAttributes {
        SectionAttributes::new_with_raw_value(self.section_attributes_raw())
    }

    #[inline]
    pub fn section_attributes_raw(&self) -> u32 {
        self.read_u32_le(0x18)
    }

    #[inline]
    pub fn section_count(&self) -> usize {
        self.read_u32_le(0x1C) as usize
    }

    #[inline]
    pub fn data_offset(&self) -> Option<usize> {
        self.read_u32_le(0x14).checked_mul(4).map(|v| v as usize)
    }
}
