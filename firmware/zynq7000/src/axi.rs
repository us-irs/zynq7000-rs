use arbitrary_int::{u2, u3, u4};

pub const AXI_HP_0_BASE_ADDR: usize = 0xF800_8000;
pub const AXI_HP_1_BASE_ADDR: usize = 0xF800_9000;
pub const AXI_HP_2_BASE_ADDR: usize = 0xF800_A000;
pub const AXI_HP_3_BASE_ADDR: usize = 0xF800_B000;

#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ReadChannelControl {
    #[bit(3, rw)]
    qos_head_of_cmdq_enable: bool,
    #[bit(2, rw)]
    fabric_out_cmd_enable: bool,
    #[bit(1, rw)]
    fabric_qos_enable: bool,
    #[bit(0, rw)]
    enable_32bit: bool,
}

#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelIssuingCapability {
    #[bits(4..=6, rw)]
    read_issue_cap_1: u3,
    #[bits(0..=2, rw)]
    read_issue_cap_0: u3,
}

#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelQos {
    #[bits(0..=3, rw)]
    static_qos: u4,
}

#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelFifoLevel {
    #[bits(0..=7, r)]
    level: u8,
}

#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelDebug {
    #[bits(1..=4, r)]
    n_commands: u4,
    #[bit(0, r)]
    fifo_overflow: bool,
}

#[bitbybit::bitenum(u2, exhaustive = false)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq)]
pub enum WriteReleaseMode {
    Last = 0,
    Threshold = 1,
}

#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct WriteChannelControl {
    #[bits(8..=11, rw)]
    write_data_threshold: u4,
    #[bits(4..=5, rw)]
    write_command_release_mode: WriteReleaseMode,
    #[bit(3, rw)]
    qos_head_of_cmdq_enable: bool,
    #[bit(2, rw)]
    fabric_out_cmd_enable: bool,
    #[bit(1, rw)]
    fabric_qos_enable: bool,
    #[bit(0, rw)]
    enable_32bit: bool,
}

/// AXI-HP register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    read_ctrl: ReadChannelControl,
    read_cap: ChannelIssuingCapability,
    read_qos: ChannelQos,
    read_fifo: ChannelFifoLevel,
    read_dbg: ChannelDebug,
    write_ctrl: WriteChannelControl,
    write_cap: ChannelIssuingCapability,
    write_qos: ChannelQos,
    write_fifo: ChannelFifoLevel,
    write_dbg: ChannelDebug,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 40);

impl Registers {
    /// Create a new AXI-HP MMIO instance for AXI_HP0 at address [AXI_HP_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(AXI_HP_0_BASE_ADDR) }
    }

    /// Create a new AXI-HP MMIO instance for AXI_HP1 at address [AXI_HP_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(AXI_HP_1_BASE_ADDR) }
    }

    /// Create a new AXI-HP MMIO instance for AXI_HP2 at address [AXI_HP_2_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_2() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(AXI_HP_2_BASE_ADDR) }
    }

    /// Create a new AXI-HP MMIO instance for AXI_HP3 at address [AXI_HP_3_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_3() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(AXI_HP_3_BASE_ADDR) }
    }

}
