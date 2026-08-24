use arbitrary_int::{u2, u3};

pub use crate::{SpiClockPhase, SpiClockPolarity};

/// Base address of the QSPI controller register block.
pub const QSPI_BASE_ADDR: usize = 0xE000D000;

/// Selects between legacy SPI and the QSPI flash memory interface.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterfaceMode {
    /// Legacy SPI interface, compatible with the SPI controller.
    LegacySpi = 0,
    /// QSPI flash memory interface, supporting dual and quad modes.
    FlashMemoryInterface = 1,
}

/// Byte order used for data transfers.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Endianness {
    /// Least significant byte first.
    Little = 0,
    /// Most significant byte first.
    Big = 1,
}

/// Baud rate divisor register values.
#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BaudRateDivisor {
    /// Reference clock divided by 2.
    _2 = 0b000,
    /// Reference clock divided by 4.
    _4 = 0b001,
    /// Reference clock divided by 8.
    _8 = 0b010,
    /// Reference clock divided by 16.
    _16 = 0b011,
    /// Reference clock divided by 32.
    _32 = 0b100,
    /// Reference clock divided by 64.
    _64 = 0b101,
    /// Reference clock divided by 128.
    _128 = 0b110,
    /// Reference clock divided by 256.
    _256 = 0b111,
}

impl BaudRateDivisor {
    /// Actual divisor value.
    pub fn divisor(&self) -> usize {
        match self {
            BaudRateDivisor::_2 => 2,
            BaudRateDivisor::_4 => 4,
            BaudRateDivisor::_8 => 8,
            BaudRateDivisor::_16 => 16,
            BaudRateDivisor::_32 => 32,
            BaudRateDivisor::_64 => 64,
            BaudRateDivisor::_128 => 128,
            BaudRateDivisor::_256 => 256,
        }
    }
}

/// Configuration register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct Config {
    /// Selects between legacy SPI and the QSPI flash memory interface.
    #[bit(31, rw)]
    interface_mode: InterfaceMode,
    /// Byte order of the transmitted and received data.
    #[bit(26, rw)]
    edianness: Endianness,
    /// Drives the HOLD_B/WP_B pin high while in the reset state.
    #[bit(19, rw)]
    holdb_dr: bool,
    /// Triggers a manual data transfer when manual start mode is enabled.
    #[bit(16, w)]
    manual_start_command: bool,
    /// Requires a manual start command to begin a data transfer.
    #[bit(15, rw)]
    manual_start_enable: bool,
    /// Enables manual chip select control instead of automatic assertion.
    #[bit(14, rw)]
    manual_cs: bool,
    /// Directly drives the chip select line when CS is driven manually (bit 14 is set)
    #[bit(10, rw)]
    peripheral_chip_select: bool,
    /// The only valid value is 0b11 (32 bits)
    #[bits(6..=7, rw)]
    fifo_width: u2,
    /// Reference clock divisor used to derive the SPI clock.
    #[bits(3..=5, rw)]
    baud_rate_div: BaudRateDivisor,
    /// Clock phase. 1: The SPI clock is inactive outside the word.
    #[bit(2, rw)]
    clock_phase: SpiClockPhase,
    /// Clock polarity. 1: The SPI clock is quiescent high.
    #[bit(1, rw)]
    clock_polarity: SpiClockPolarity,
    /// Must be set to 1 before using QSPI, 0 is a reserved value.
    #[bit(0, rw)]
    mode_select: bool,
}

/// Interrupt status register.
#[bitbybit::bitfield(u32, debug, forbid_overlaps)]
pub struct InterruptStatus {
    /// Write-to-clear bit.
    #[bit(6, rw)]
    tx_underflow: bool,
    /// RX FIFO is full.
    #[bit(5, r)]
    rx_full: bool,
    /// RX FIFO level is above the threshold.
    #[bit(4, r)]
    rx_above_threshold: bool,
    /// TX FIFO is full.
    #[bit(3, r)]
    tx_full: bool,
    /// TX FIFO level is below the threshold.
    #[bit(2, r)]
    tx_below_threshold: bool,
    /// Write-to-clear bit.
    #[bit(0, rw)]
    rx_overrun: bool,
}

/// Interrupt enable and disable register.
#[bitbybit::bitfield(u32, forbid_overlaps)]
pub struct InterruptControl {
    /// TX FIFO underflow interrupt.
    #[bit(6, w)]
    tx_underflow: bool,
    /// RX FIFO full interrupt.
    #[bit(5, w)]
    rx_full: bool,
    /// RX FIFO not empty interrupt.
    #[bit(4, w)]
    rx_not_empty: bool,
    /// TX FIFO full interrupt.
    #[bit(3, w)]
    tx_full: bool,
    /// TX FIFO not full interrupt.
    #[bit(2, w)]
    tx_not_full: bool,
    /// RX FIFO overrun interrupt.
    #[bit(0, w)]
    rx_overrun: bool,
}

/// Interrupt mask register.
#[bitbybit::bitfield(u32, debug, forbid_overlaps)]
pub struct InterruptMask {
    /// TX FIFO underflow interrupt is masked.
    #[bit(6, r)]
    tx_underflow: bool,
    /// RX FIFO full interrupt is masked.
    #[bit(5, r)]
    rx_full: bool,
    /// RX FIFO not empty interrupt is masked.
    #[bit(4, r)]
    rx_not_empty: bool,
    /// TX FIFO full interrupt is masked.
    #[bit(3, r)]
    tx_full: bool,
    /// TX FIFO not full interrupt is masked.
    #[bit(2, r)]
    tx_not_full: bool,
    /// RX FIFO overrun interrupt is masked.
    #[bit(0, r)]
    rx_overrun: bool,
}

/// SPI enable register.
#[bitbybit::bitfield(u32, default = 0x0, debug, forbid_overlaps)]
pub struct SpiEnable {
    /// Enables the SPI/QSPI controller.
    #[bit(0, rw)]
    enable: bool,
}

/// All the delays are in SPI reference block or external clock cycles.
#[bitbybit::bitfield(u32, debug, forbid_overlaps)]
pub struct Delay {
    /// Length of the master mode chip select output de-asserts between words when CPHA = 0.
    #[bits(24..=31, rw)]
    deassert: u8,
    /// Delay between one chip select being de-activated and another being activated.
    #[bits(16..=23, rw)]
    between: u8,
    /// Length between last bit of current word and first bit of next word.
    #[bits(8..=15, rw)]
    after: u8,
    /// Delay between setting chip select low and first bit transfer.
    #[bits(0..=7, rw)]
    init: u8,
}

/// GPIO register controlling the flash write-protect pin.
#[bitbybit::bitfield(u32, forbid_overlaps)]
pub struct Gpio {
    /// Active low write-protect bit.
    #[bit(0, rw)]
    write_protect_n: bool,
}

/// Loopback master clock delay register, used to tune read data capture timing.
#[bitbybit::bitfield(u32, default = 0x0, debug, forbid_overlaps)]
pub struct LoopbackMasterClockDelay {
    /// Use internal loopback master clock for read data capturing when the baud rate divisor
    /// is 2.
    #[bit(5, rw)]
    use_loopback: bool,
    /// Upper delay element for the loopback master clock.
    #[bits(3..=4,rw)]
    delay_1: u2,
    /// Lower delay element for the loopback master clock.
    #[bits(0..=2 ,rw)]
    delay_0: u3,
}

/// Flash read instruction sent in linear addressing mode.
#[bitbybit::bitenum(u8, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
pub enum InstructionCode {
    /// Single output read.
    Read = 0x03,
    /// Single output fast read.
    FastRead = 0x0B,
    /// Dual output fast read.
    FastReadDualOutput = 0x3B,
    /// Quad output fast read.
    FastReadQuadOutput = 0x6B,
    /// Dual I/O fast read.
    FastReadDualIo = 0xBB,
    /// Quad I/O fast read.
    FastReadQuadIo = 0xEB,
}

/// Linear QSPI configuration register, controlling AXI memory-mapped flash reads.
#[bitbybit::bitfield(u32, default = 0x0, debug, forbid_overlaps)]
pub struct LinearQspiConfig {
    /// Enables the linear addressing mode, exposing the flash as memory-mapped AXI space.
    #[bit(31, rw)]
    enable_linear_mode: bool,
    /// Both attached memories are used together to widen the data bus.
    #[bit(30, rw)]
    both_memories: bool,
    /// Only has a meaning is bit 30 is set (both memories).
    #[bit(29, rw)]
    separate_memory_bus: bool,
    /// Upper memory page, if set. Only has a meaning if bit 30 is set and bit 29 / bit 31 are
    /// cleared.
    ///
    /// In LQSPI mode, address bit 25 will indicate the lower (0) or upper (1) page.
    /// In IO mode, this bit selects the lower or upper memory.
    #[bit(28, rw)]
    upper_memory_page: bool,
    /// Enables sending mode bits after the address in the read command.
    #[bit(25, rw)]
    mode_enable: bool,
    /// Selects whether the mode bits indicate the next access does not need an instruction byte.
    #[bit(24, rw)]
    mode_on: bool,
    /// Mode bits sent after the address when mode bit output is enabled.
    #[bits(16..=23, rw)]
    mode_bits: u8,
    /// Number of dummy bytes inserted between the address and the returned data.
    #[bits(8..=10, rw)]
    num_dummy_bytes: u3,
    /// Read instruction sent to the flash device for linear reads.
    #[bits(0..=7, rw)]
    instruction_code: Option<InstructionCode>,
}

/// Linear QSPI status register.
#[bitbybit::bitfield(u32, debug, forbid_overlaps)]
pub struct LinearQspiStatus {
    /// The AXI read data state machine encountered an error.
    #[bit(2, rw)]
    data_fsm_error: bool,
    /// An AXI write command was received while in linear mode, which is not supported.
    #[bit(1, rw)]
    axi_write_command_received: bool,
}

/// QSPI register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Configuration register.
    config: Config,
    /// Interrupt status register.
    interrupt_status: InterruptStatus,
    /// Interrupt enable register.
    #[mmio(Write)]
    interrupt_enable: InterruptControl,
    /// Interrupt disable register.
    #[mmio(Write)]
    interrupt_disable: InterruptControl,
    /// Interrupt mask register.
    #[mmio(PureRead)]
    interupt_mask: InterruptMask,
    /// SPI enable register.
    spi_enable: SpiEnable,
    /// Delay register.
    delay: Delay,
    /// Transmits 1-byte command and 3-byte data OR 4-byte data.
    #[mmio(Write)]
    tx_data_00: u32,
    /// Receives data from the RX FIFO.
    #[mmio(Read)]
    rx_data: u32,
    /// Minimum number of clock cycles the slave select stays idle between transfers.
    slave_idle_count: u32,
    /// Defines the level at which the TX FIFO not full interrupt is generated.
    tx_fifo_threshold: u32,
    /// Defines the level at which the RX FIFO not empty interrupt is generated.
    rx_fifo_threshold: u32,
    /// GPIO register controlling the flash write-protect pin.
    gpio: Gpio,
    _reserved0: u32,
    /// Loopback master clock delay register, used to tune read data capture timing.
    loopback_master_clock_delay: LoopbackMasterClockDelay,
    _reserved1: [u32; 0x11],
    /// Transmits 1-byte command.
    #[mmio(Write)]
    tx_data_01: u32,
    /// Transmits 1-byte command and 1-byte data.
    #[mmio(Write)]
    tx_data_10: u32,
    /// Transmits 1-byte command and 2-byte data.
    #[mmio(Write)]
    tx_data_11: u32,
    _reserved2: [u32; 0x5],
    /// Linear QSPI configuration register, controlling AXI memory-mapped flash reads.
    linear_qspi_config: LinearQspiConfig,
    /// Linear QSPI status register.
    linear_qspi_status: LinearQspiStatus,
    _reserved3: [u32; 0x15],
    /// Module ID value with reset value 0x1090101.
    module_id: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x100);

impl Registers {
    /// Create a new QSPI MMIO instance for for QSPI controller at address [QSPI_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(QSPI_BASE_ADDR) }
    }
}
