//! # AXI high-performance port control registers.
pub use arbitrary_int::{u3, u4};

/// Port 0 base address.
pub const AXI_HP_0_BASE_ADDR: usize = 0xF800_8000;
/// Port 1 base address.
pub const AXI_HP_1_BASE_ADDR: usize = 0xF800_9000;
/// Port 2 base address.
pub const AXI_HP_2_BASE_ADDR: usize = 0xF800_A000;
/// Port 3 base address.
pub const AXI_HP_3_BASE_ADDR: usize = 0xF800_B000;

/// Read channel control.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ReadChannelControl {
    /// Enables QoS for the command at the head of the command queue.
    #[bit(3, rw)]
    qos_head_of_cmdq_enable: bool,
    /// Enables output of commands to the fabric.
    #[bit(2, rw)]
    fabric_out_cmd_enable: bool,
    /// Enables QoS signalling on the fabric interface.
    #[bit(1, rw)]
    fabric_qos_enable: bool,
    /// Enables 32-bit (instead of 64-bit) read data width.
    #[bit(0, rw)]
    enable_32bit: bool,
}

/// Channel issuing capability.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelIssuingCapability {
    /// Maximum number of outstanding issuing commands, capability set 1.
    #[bits(4..=6, rw)]
    read_issue_cap_1: u3,
    /// Maximum number of outstanding issuing commands, capability set 0.
    #[bits(0..=2, rw)]
    read_issue_cap_0: u3,
}

/// Channel QoS configuration.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelQos {
    /// Static QoS value applied to commands on this channel.
    #[bits(0..=3, rw)]
    static_qos: u4,
}

/// Channel FIFO fill level.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelFifoLevel {
    /// Current fill level of the channel FIFO.
    #[bits(0..=7, r)]
    level: u8,
}

/// Channel debug status.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct ChannelDebug {
    /// Number of outstanding commands currently in the channel.
    #[bits(1..=4, r)]
    n_commands: u4,
    /// Set when the channel FIFO has overflowed.
    #[bit(0, r)]
    fifo_overflow: bool,
}

/// Selects when write data is released back to the fabric.
#[bitbybit::bitenum(u2, exhaustive = false)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq)]
pub enum WriteReleaseMode {
    /// Release write data on the last beat of the burst.
    Last = 0,
    /// Release write data once the configured threshold is reached.
    Threshold = 1,
}

/// Write channel control.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct WriteChannelControl {
    /// Data threshold (in words) used when the release mode is `Threshold`.
    #[bits(8..=11, rw)]
    write_data_threshold: u4,
    /// Selects the write command release mode (see [`WriteReleaseMode`]).
    #[bits(4..=5, rw)]
    write_command_release_mode: Option<WriteReleaseMode>,
    /// Enables QoS for the command at the head of the command queue.
    #[bit(3, rw)]
    qos_head_of_cmdq_enable: bool,
    /// Enables output of commands to the fabric.
    #[bit(2, rw)]
    fabric_out_cmd_enable: bool,
    /// Enables QoS signalling on the fabric interface.
    #[bit(1, rw)]
    fabric_qos_enable: bool,
    /// Enables 32-bit (instead of 64-bit) write data width.
    #[bit(0, rw)]
    enable_32bit: bool,
}

/// AXI-HP register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Read channel control register.
    read_ctrl: ReadChannelControl,
    /// Read channel issuing capability register.
    read_cap: ChannelIssuingCapability,
    /// Read channel QoS register.
    read_qos: ChannelQos,
    /// Read channel FIFO level register.
    read_fifo: ChannelFifoLevel,
    /// Read channel debug register.
    read_dbg: ChannelDebug,
    /// Write channel control register.
    write_ctrl: WriteChannelControl,
    /// Write channel issuing capability register.
    write_cap: ChannelIssuingCapability,
    /// Write channel QoS register.
    write_qos: ChannelQos,
    /// Write channel FIFO level register.
    write_fifo: ChannelFifoLevel,
    /// Write channel debug register.
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
