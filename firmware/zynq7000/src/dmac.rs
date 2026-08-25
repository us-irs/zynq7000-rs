//! # DMA Controller register module
//!
//! This is an Arm PL330. Unlike most peripherals, the channel state (source and
//! destination address, transfer control, loop counters, ...) is not configured by writing
//! registers directly. Instead, the DMA manager thread executes a small instruction program
//! that configures and drives each channel, and the registers here mostly let software read
//! back that state. The only registers writable from the AXI bus are
//! [crate::dmac::InterruptEnable], [crate::dmac::InterruptClear] and the debug instruction port
//! ([crate::dmac::DbgCommand], [crate::dmac::DbgInst0], [crate::dmac::DbgInst1]), which is how
//! software loads and starts a DMA program in the first place.

/// Base address of the non-secure register view.
pub const BASE_ADDR_NS: usize = 0xF800_4000;
/// Base address of the secure register view.
pub const BASE_ADDR_S: usize = 0xF800_3000;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u2, u3, u4, u5, u10};

    /// Operating state of the DMA manager or a DMA channel thread.
    #[bitbybit::bitenum(u4, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ThreadState {
        /// Stopped.
        Stopped = 0b0000,
        /// Executing.
        Executing = 0b0001,
        /// Fetching an instruction after a cache miss.
        CacheMiss = 0b0010,
        /// Updating the program counter.
        UpdatingProgramCounter = 0b0011,
        /// Waiting for an event, see [DmaManagerStatus::wakeup_event] or
        /// [ChannelStatus::wakeup_number].
        WaitingForEvent = 0b0100,
        /// Waiting at a barrier. Channel threads only.
        AtBarrier = 0b0101,
        /// Waiting for a peripheral, see [ChannelStatus::wakeup_number]. Channel threads only.
        WaitingForPeripheral = 0b0111,
        /// Being killed. Channel threads only.
        Killing = 0b1000,
        /// Completing the current AXI transfer before stopping. Channel threads only.
        Completing = 0b1001,
        /// Completing the current AXI transfer before entering the Faulting state. Channel
        /// threads only.
        FaultingCompleting = 0b1110,
        /// Faulting, see the fault status and fault type registers.
        Faulting = 0b1111,
    }

    /// DMA manager status register. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct DmaManagerStatus {
        /// Security state of the DMA manager thread. 0: Secure, 1: Non-secure.
        #[bit(9, r)]
        non_secure: bool,
        /// Event the manager is waiting for while executing a DMAWFE instruction, valid only
        /// while [Self::status] is [ThreadState::WaitingForEvent]. Values 0 to 15 select event
        /// 0 to 15, values 16 and above are reserved.
        #[bits(4..=8, r)]
        wakeup_event: u5,
        /// Current operating state.
        #[bits(0..=3, r)]
        status: Option<ThreadState>,
    }

    /// Interrupt enable register. For each channel bit, controls how the channel thread
    /// responds to a DMASEV instruction: broadcast the event to the other threads, or assert
    /// the channel's interrupt line to the PS interrupt controller.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptEnable {
        /// Bit N selects the response for channel N. 0: signal the event to the other
        /// threads. 1: assert the channel's interrupt signal.
        #[bits(0..=7, rw)]
        assert_interrupt: u8,
    }

    /// Raw event and interrupt status register. Read-only. Bits 0 to 15 give the raw state of
    /// events 0 to 15, of which events 0 to 7 also serve as the raw interrupt state for
    /// channels 0 to 7 (see [InterruptStatus]).
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct EventStatus {
        /// Bit N is set while event N is active.
        #[bits(0..=15, r)]
        active: u16,
    }

    /// Interrupt status register. Read-only, mirrors which channel interrupt lines are
    /// currently asserted. Cleared through [InterruptClear].
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct InterruptStatus {
        /// Bit N is set while the interrupt signal for channel N is high.
        #[bits(0..=7, r)]
        pending: u8,
    }

    /// Interrupt clear register. Write-only, writing 1 to bit N clears the interrupt signal
    /// for channel N.
    #[bitbybit::bitfield(u32, default = 0, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(Debug)]
    pub struct InterruptClear {
        /// Write 1 to bit N to clear the interrupt for channel N.
        #[bits(0..=7, w)]
        clear: u8,
    }

    /// DMA manager fault status register. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ManagerFaultStatus {
        /// The DMA manager is in the Faulting state, see [ManagerFaultType].
        #[bit(0, r)]
        faulting: bool,
    }

    /// DMA channel fault status register. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ChannelFaultStatus {
        /// Bit N is set while channel N is in the Faulting or Faulting completing state, see
        /// [ChannelFaultType].
        #[bits(0..=7, r)]
        faulting: u8,
    }

    /// DMA manager fault type register. Read-only, valid while [ManagerFaultStatus::faulting]
    /// is set.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ManagerFaultType {
        /// The aborting instruction was read from the debug interface rather than system
        /// memory.
        #[bit(30, r)]
        read_from_debug_interface: bool,
        /// The instruction fetch received an erroneous AXI response.
        #[bit(16, r)]
        instruction_fetch_error: bool,
        /// The manager attempted a DMAWFE or DMASEV targeting a secure event or interrupt
        /// while in the Non-secure state.
        #[bit(5, r)]
        event_security_error: bool,
        /// The manager attempted a DMAGO targeting the Secure state while in the Non-secure
        /// state.
        #[bit(4, r)]
        start_security_error: bool,
        /// The instruction operand was not valid for this DMAC configuration.
        #[bit(1, r)]
        invalid_operand: bool,
        /// The instruction is not a defined DMA instruction.
        #[bit(0, r)]
        undefined_instruction: bool,
    }

    /// Fault type register, shared by all eight DMA channels. Read-only, valid while the
    /// corresponding bit in [ChannelFaultStatus] is set.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ChannelFaultType {
        /// The channel locked up due to resource starvation. Imprecise abort.
        #[bit(31, r)]
        lockup: bool,
        /// The aborting instruction was read from the debug interface rather than system
        /// memory.
        #[bit(30, r)]
        read_from_debug_interface: bool,
        /// A data read received an erroneous AXI response. Imprecise abort.
        #[bit(18, r)]
        data_read_error: bool,
        /// A data write received an erroneous AXI response. Imprecise abort.
        #[bit(17, r)]
        data_write_error: bool,
        /// The instruction fetch received an erroneous AXI response. Precise abort.
        #[bit(16, r)]
        instruction_fetch_error: bool,
        /// A DMAST could not complete because the MFIFO does not hold enough data yet.
        /// Precise abort.
        #[bit(13, r)]
        store_data_unavailable: bool,
        /// The MFIFO was too small for a DMALD or DMAST to complete. Imprecise abort.
        #[bit(12, r)]
        mfifo_error: bool,
        /// The channel, in the Non-secure state, attempted a secure read or write of the
        /// channel control register. Precise abort.
        #[bit(7, r)]
        secure_register_access_error: bool,
        /// The channel, in the Non-secure state, attempted a DMAWFP, DMALDP, DMASTP or
        /// DMAFLUSHP targeting a secure peripheral. Precise abort.
        #[bit(6, r)]
        peripheral_security_error: bool,
        /// The channel, in the Non-secure state, attempted a DMAWFE or DMASEV targeting a
        /// secure event or interrupt. Precise abort.
        #[bit(5, r)]
        event_security_error: bool,
        /// The instruction operand was not valid for this DMAC configuration. Precise abort.
        #[bit(1, r)]
        invalid_operand: bool,
        /// The instruction is not a defined DMA instruction. Precise abort.
        #[bit(0, r)]
        undefined_instruction: bool,
    }

    /// Channel status register, shared by all eight DMA channels. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ChannelStatus {
        /// Security state of the channel thread. 0: Secure, 1: Non-secure.
        #[bit(21, r)]
        non_secure: bool,
        /// The channel is executing a DMAWFP with the peripheral operand set. Only
        /// meaningful while the channel is connected to one of the four peripheral request
        /// interfaces.
        #[bit(15, r)]
        waiting_for_peripheral: bool,
        /// While [Self::waiting_for_peripheral] is set, whether the DMAWFP requested a burst
        /// (1) or single (0) transfer.
        #[bit(14, r)]
        waiting_for_burst: bool,
        /// Event or peripheral the channel is waiting for while executing a DMAWFE or DMAWFP,
        /// valid only while [Self::state] is [ThreadState::WaitingForEvent] or
        /// [ThreadState::WaitingForPeripheral]. For DMAWFE, values 0 to 15 select event 0 to
        /// 15. For DMAWFP, values 0 to 3 select peripheral request interface 0 to 3.
        #[bits(4..=8, r)]
        wakeup_number: u5,
        /// Current operating state.
        #[bits(0..=3, r)]
        state: Option<ThreadState>,
    }

    /// Loop counter register, shared by the two loop counters of all eight DMA channels.
    /// Read-only, updated by the DMAC when the channel thread executes a DMALP or DMALPEND
    /// instruction using this counter.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct LoopCounter {
        /// Number of loop iterations remaining.
        #[bits(0..=7, r)]
        iterations: u8,
    }

    /// Endianness byte-swap applied by a channel, see [ChannelControl::swap].
    #[bitbybit::bitenum(u3, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SwapSize {
        /// No swap.
        None = 0b000,
        /// Swap bytes within 16-bit data.
        Bits16 = 0b001,
        /// Swap bytes within 32-bit data.
        Bits32 = 0b010,
        /// Swap bytes within 64-bit data.
        Bits64 = 0b011,
        /// Swap bytes within 128-bit data.
        Bits128 = 0b100,
    }

    /// Number of bytes a channel transfers per beat within a burst.
    #[bitbybit::bitenum(u3, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum BeatSize {
        /// 1 byte per beat.
        OneByte = 0b000,
        /// 2 bytes per beat.
        TwoBytes = 0b001,
        /// 4 bytes per beat.
        FourBytes = 0b010,
        /// 8 bytes per beat.
        EightBytes = 0b011,
        /// 16 bytes per beat.
        SixteenBytes = 0b100,
    }

    /// Channel control register, shared by all eight DMA channels. Read-only, reflects the
    /// operand of the last DMAMOV CCR instruction the channel thread executed. Programs the
    /// AXI attributes used for the channel's reads and writes.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ChannelControl {
        /// Byte swap applied to write data.
        #[bits(28..=30, r)]
        swap: Option<SwapSize>,
        /// AXI AWCACHE signals used for writes. Bit 2 drives AWCACHE\[3\], bit 1 drives
        /// AWCACHE\[1\], bit 0 drives AWCACHE\[0\]. AWCACHE\[2\] is always low.
        #[bits(25..=27, r)]
        dst_cache: u3,
        /// AXI AWPROT signals used for writes. Bit 2/1/0 drives AWPROT\[2\]/\[1\]/\[0\]. Only a
        /// channel in the Secure state can set AWPROT\[1\] low.
        #[bits(22..=24, r)]
        dst_prot: u3,
        /// Number of transfers per write burst, minus one.
        #[bits(18..=21, r)]
        dst_burst_len: u4,
        /// Number of bytes transferred per write beat.
        #[bits(15..=17, r)]
        dst_burst_size: Option<BeatSize>,
        /// Write burst type. 0: fixed address (AWBURST\[0\] low). 1: incrementing address
        /// (AWBURST\[0\] high).
        #[bit(14, r)]
        dst_increment: bool,
        /// AXI ARCACHE signals used for reads. Bit 2/1/0 drives ARCACHE\[2\]/\[1\]/\[0\].
        /// ARCACHE\[3\] is always low.
        #[bits(11..=13, r)]
        src_cache: u3,
        /// AXI ARPROT signals used for reads. Bit 2/1/0 drives ARPROT\[2\]/\[1\]/\[0\]. Only a
        /// channel in the Secure state can set ARPROT\[1\] low.
        #[bits(8..=10, r)]
        src_prot: u3,
        /// Number of transfers per read burst, minus one.
        #[bits(4..=7, r)]
        src_burst_len: u4,
        /// Number of bytes transferred per read beat.
        #[bits(1..=3, r)]
        src_burst_size: Option<BeatSize>,
        /// Read burst type. 0: fixed address (ARBURST\[0\] low). 1: incrementing address
        /// (ARBURST\[0\] high).
        #[bit(0, r)]
        src_increment: bool,
    }

    /// Debug execution status register. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct DbgStatus {
        /// The DMA manager is executing a debug instruction submitted through [DbgCommand].
        #[bit(0, r)]
        busy: bool,
    }

    /// Debug command register. Write-only, submits the instruction staged in [DbgInst0] and
    /// [DbgInst1] for execution. Wait for [DbgStatus::busy] to clear before writing.
    #[bitbybit::bitfield(u32, default = 0, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(Debug)]
    pub struct DbgCommand {
        /// Must be written as 0 to execute the staged instruction. All other values are
        /// reserved.
        #[bits(0..=1, w)]
        execute: u2,
    }

    /// Selects which thread a debug instruction targets, see [DbgInst0::with_thread].
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DebugThread {
        /// The DMA manager thread.
        Manager = 0,
        /// A DMA channel thread, selected by [DbgInst0::with_channel].
        Channel = 1,
    }

    /// First word of a staged debug instruction, holding its first two bytes and target
    /// thread. Write-only.
    #[bitbybit::bitfield(u32, default = 0, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(Debug)]
    pub struct DbgInst0 {
        /// Instruction byte 1.
        #[bits(24..=31, w)]
        instruction_byte_1: u8,
        /// Instruction byte 0, the opcode.
        #[bits(16..=23, w)]
        instruction_byte_0: u8,
        /// DMA channel targeted when [Self::with_thread] is [DebugThread::Channel].
        #[bits(8..=10, w)]
        channel: u3,
        /// Target thread for the instruction.
        #[bit(0, w)]
        thread: DebugThread,
    }

    /// Second word of a staged debug instruction, holding its remaining four bytes.
    /// Write-only.
    #[bitbybit::bitfield(u32, default = 0, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(Debug)]
    pub struct DbgInst1 {
        /// Instruction byte 5.
        #[bits(24..=31, w)]
        instruction_byte_5: u8,
        /// Instruction byte 4.
        #[bits(16..=23, w)]
        instruction_byte_4: u8,
        /// Instruction byte 3.
        #[bits(8..=15, w)]
        instruction_byte_3: u8,
        /// Instruction byte 2.
        #[bits(0..=7, w)]
        instruction_byte_2: u8,
    }

    /// Configuration register 0: number of events, peripheral request interfaces and channel
    /// threads implemented, plus the boot configuration latched at reset. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct Cr0 {
        /// Number of events implemented, minus one. Always 15 (16 events).
        #[bits(17..=21, r)]
        max_events: u5,
        /// Number of peripheral request interfaces implemented, minus one. Always 3
        /// (4 interfaces).
        #[bits(12..=16, r)]
        max_peripherals: u5,
        /// Number of channel threads implemented, minus one. Always 7 (8 channels).
        #[bits(4..=6, r)]
        max_channels: u3,
        /// Security state of the DMA manager thread when the DMAC exited reset, latched from
        /// the SLCR non-secure configuration bit.
        #[bit(2, r)]
        manager_non_secure_at_reset: bool,
        /// The boot_from_pc input was high when the DMAC exited reset, meaning the manager
        /// booted by executing a program at the manager boot address register instead of
        /// waiting for a DMAGO instruction.
        #[bit(1, r)]
        booted_from_pc: bool,
        /// The DMAC provides the four peripheral request interfaces.
        #[bit(0, r)]
        has_peripheral_interfaces: bool,
    }

    /// Configuration register 1: instruction cache geometry. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct Cr1 {
        /// Number of instruction cache lines. Always 8.
        #[bits(4..=7, r)]
        icache_lines: u4,
        /// Length of an instruction cache line in bytes. Always 16.
        #[bits(0..=2, r)]
        icache_line_len: u3,
    }

    /// Configuration register D: MFIFO and AXI queue depths, and the AXI data bus width.
    /// Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct ConfigD {
        /// Depth of the MFIFO in 64-bit double words.
        #[bits(20..=29, r)]
        fifo_depth: u10,
        /// Depth of the read queue.
        #[bits(16..=19, r)]
        read_queue_depth: u4,
        /// Number of outstanding read transactions supported.
        #[bits(12..=14, r)]
        outstanding_reads: u3,
        /// Depth of the write queue.
        #[bits(8..=11, r)]
        write_queue_depth: u4,
        /// Number of outstanding write transactions supported.
        #[bits(4..=6, r)]
        outstanding_writes: u3,
        /// Width of the AXI master interface data bus, log2(bits) minus 3. 3 means 64 bits.
        #[bits(0..=2, r)]
        data_width: u3,
    }

    /// Watchdog register. Read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct Watchdog {
        /// When a channel lock-up is detected, the DMAC aborts the channel thread and
        /// asserts its Abort interrupt.
        #[bit(0, r)]
        irq_only: bool,
    }

    /// Status and program counter of one DMA channel thread. Read-only.
    #[derive(derive_mmio::Mmio)]
    #[repr(C)]
    pub struct ChannelStatusRegisters {
        #[mmio(PureRead)]
        status: ChannelStatus,
        /// Program counter of the channel thread.
        #[mmio(PureRead)]
        pc: u32,
    }

    /// Source, destination, control and loop counter registers of one DMA channel. Read-only,
    /// reflects the operands of the DMA instructions the channel thread has executed. Padded
    /// to the 0x20-byte channel-to-channel stride used by the DMAC.
    #[derive(derive_mmio::Mmio)]
    #[repr(C)]
    pub struct ChannelRegisters {
        /// Source address.
        #[mmio(PureRead)]
        source_addr: u32,
        /// Destination address.
        #[mmio(PureRead)]
        dest_addr: u32,
        #[mmio(PureRead)]
        control: ChannelControl,
        #[mmio(PureRead)]
        loop_counter_a: LoopCounter,
        #[mmio(PureRead)]
        loop_counter_b: LoopCounter,
        _gap: [u32; 3],
    }
}

pub use types::*;

/// DMA controller register block.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    #[mmio(PureRead)]
    manager_status: DmaManagerStatus,
    /// Program counter of the DMA manager thread. Read-only.
    #[mmio(PureRead)]
    manager_pc: u32,
    _gap0: [u32; 6],

    interrupt_enable: InterruptEnable,
    #[mmio(PureRead)]
    event_status: EventStatus,
    #[mmio(PureRead)]
    interrupt_status: InterruptStatus,
    #[mmio(Write)]
    interrupt_clear: InterruptClear,
    #[mmio(PureRead)]
    manager_fault_status: ManagerFaultStatus,
    #[mmio(PureRead)]
    channel_fault_status: ChannelFaultStatus,
    #[mmio(PureRead)]
    manager_fault_type: ManagerFaultType,
    _gap1: [u32; 1],

    #[mmio(PureRead)]
    channel_fault_type: [ChannelFaultType; 8],
    _gap2: [u32; 40],

    /// Status and program counter of DMA channels 0 to 7.
    #[mmio(Inner)]
    channel_status: [ChannelStatusRegisters; 8],
    _gap3: [u32; 176],

    /// Source, destination, control and loop counter registers of DMA channels 0 to 7.
    #[mmio(Inner)]
    channels: [ChannelRegisters; 8],
    _gap4: [u32; 512],

    #[mmio(PureRead)]
    debug_status: DbgStatus,
    #[mmio(Write)]
    debug_command: DbgCommand,
    #[mmio(Write)]
    debug_instruction_0: DbgInst0,
    #[mmio(Write)]
    debug_instruction_1: DbgInst1,
    _gap5: [u32; 60],

    #[mmio(PureRead)]
    config_0: Cr0,
    #[mmio(PureRead)]
    config_1: Cr1,
    /// Boot address of the DMA manager thread, always 0 since only system memory boot is
    /// supported. Read-only.
    #[mmio(PureRead)]
    manager_boot_addr: u32,
    /// Security state, one bit per IRQ line, latched from the SLCR at reset. Read-only.
    #[mmio(PureRead)]
    irq_security: u32,
    /// Security state, one bit per peripheral request interface, latched from the SLCR at
    /// reset. Read-only.
    #[mmio(PureRead)]
    peripheral_security: u32,
    #[mmio(PureRead)]
    config_d: ConfigD,
    _gap6: [u32; 26],

    #[mmio(PureRead)]
    watchdog: Watchdog,
    _gap7: [u32; 87],

    /// Peripheral identification registers 0 to 3. Read-only.
    #[mmio(PureRead)]
    peripheral_id: [u32; 4],
    /// Component identification registers 0 to 3. Read-only.
    #[mmio(PureRead)]
    component_id: [u32; 4],
}

static_assertions::const_assert_eq!(core::mem::size_of::<ChannelStatusRegisters>(), 0x8);
static_assertions::const_assert_eq!(core::mem::size_of::<ChannelRegisters>(), 0x20);

static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, channel_fault_type), 0x40);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, channel_status), 0x100);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, channels), 0x400);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, debug_status), 0xD00);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, config_0), 0xE00);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, watchdog), 0xE80);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, peripheral_id), 0xFE0);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, component_id), 0xFF0);

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x1000);

impl Registers {
    /// Create a new DMAC MMIO instance for the non-secure register view at
    /// [BASE_ADDR_NS].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do
    /// not interfere with each other.
    pub const unsafe fn new_mmio_fixed_ns() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(BASE_ADDR_NS) }
    }

    /// Create a new DMAC MMIO instance for the secure register view at [BASE_ADDR_S].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do
    /// not interfere with each other.
    pub const unsafe fn new_mmio_fixed_s() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(BASE_ADDR_S) }
    }
}
