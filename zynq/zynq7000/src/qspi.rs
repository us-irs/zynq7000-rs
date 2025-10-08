use arbitrary_int::{u2, u3};

pub use crate::{SpiClockPhase, SpiClockPolarity};

pub const QSPI_BASE_ADDR: usize = 0xE000D000;

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum InterfaceMode {
    LegacySpi = 0,
    FlashMemoryInterface = 1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum Endianness {
    Little = 0,
    Big = 1,
}

/// Baud rate divisor register values.
#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum BaudRateDivisor {
    _2 = 0b000,
    _4 = 0b001,
    _8 = 0b010,
    _16 = 0b011,
    _32 = 0b100,
    _64 = 0b101,
    _128 = 0b110,
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

// TODO: Use bitbybit debug support as soon as support for write fields has been implemented.
#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct Config {
    #[bit(31, rw)]
    interface_mode: InterfaceMode,
    #[bit(26, rw)]
    edianness: Endianness,
    #[bit(19, rw)]
    holdb_dr: bool,
    #[bit(16, w)]
    manual_start_command: bool,
    #[bit(15, rw)]
    manual_start_enable: bool,
    #[bit(14, rw)]
    manual_cs: bool,
    /// Directly drives the chip select line when CS is driven manually (bit 14 is set)
    #[bit(10, rw)]
    peripheral_chip_select: bool,
    /// The only valid value is 0b11 (32 bits)
    #[bits(6..=7, rw)]
    fifo_width: u2,
    #[bits(3..=5, rw)]
    baud_rate_div: BaudRateDivisor,
    #[bit(2, rw)]
    clock_phase: SpiClockPhase,
    #[bit(1, rw)]
    clock_polarity: SpiClockPolarity,
    /// Must be set to 1 before using QSPI, 0 is a reserved value.
    #[bit(0, rw)]
    mode_select: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptStatus {
    /// Write-to-clear bit.
    #[bit(6, rw)]
    tx_underflow: bool,
    #[bit(5, r)]
    rx_full: bool,
    #[bit(4, r)]
    rx_above_threshold: bool,
    #[bit(3, r)]
    tx_full: bool,
    #[bit(2, r)]
    tx_below_threshold: bool,
    /// Write-to-clear bit.
    #[bit(0, rw)]
    rx_overrun: bool,
}

#[bitbybit::bitfield(u32)]
pub struct InterruptControl {
    #[bit(6, w)]
    tx_underflow: bool,
    #[bit(5, w)]
    rx_full: bool,
    #[bit(4, w)]
    rx_not_empty: bool,
    #[bit(3, w)]
    tx_full: bool,
    #[bit(2, w)]
    tx_not_full: bool,
    #[bit(0, w)]
    rx_overrun: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptMask {
    #[bit(6, r)]
    tx_underflow: bool,
    #[bit(5, r)]
    rx_full: bool,
    #[bit(4, r)]
    rx_not_empty: bool,
    #[bit(3, r)]
    tx_full: bool,
    #[bit(2, r)]
    tx_not_full: bool,
    #[bit(0, r)]
    rx_overrun: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct SpiEnable {
    #[bit(0, rw)]
    enable: bool,
}

/// All the delays are in SPI reference block or external clock cycles.
#[bitbybit::bitfield(u32, debug)]
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

#[bitbybit::bitfield(u32)]
pub struct Gpio {
    /// Active low write-protect bit.
    #[bit(0, rw)]
    write_protect_n: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct LoopbackMasterClockDelay {
    /// Use internal loopback master clock for read data capturing when the baud rate divisor
    /// is 2.
    #[bit(5, rw)]
    use_loopback: bool,
    #[bits(3..=4,rw)]
    delay_1: u2,
    #[bits(0..=2 ,rw)]
    delay_0: u3,
}

#[bitbybit::bitenum(u8, exhaustive = false)]
#[derive(Debug, PartialEq, Eq)]
pub enum InstructionCode {
    Read = 0x03,
    FastRead = 0x0B,
    FastReadDualOutput = 0x3B,
    FastReadQuadOutput = 0x6B,
    FastReadDualIo = 0xBB,
    FastReadQuadIo = 0xEB,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct LinearQspiConfig {
    #[bit(31, rw)]
    enable_linear_mode: bool,
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
    #[bit(25, rw)]
    mode_enable: bool,
    #[bit(24, rw)]
    mode_on: bool,
    #[bits(16..=23, rw)]
    mode_bits: u8,
    #[bits(8..=10, rw)]
    num_dummy_bytes: u3,
    #[bits(0..=7, rw)]
    instruction_code: Option<InstructionCode>,
}

#[bitbybit::bitfield(u32, debug)]
pub struct LinearQspiStatus {
    #[bit(2, rw)]
    data_fsm_error: bool,
    #[bit(1, rw)]
    axi_write_command_received: bool,
}

/// QSPI register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Qspi {
    config: Config,
    interrupt_status: InterruptStatus,
    #[mmio(Write)]
    interrupt_enable: InterruptControl,
    #[mmio(Write)]
    interrupt_disable: InterruptControl,
    #[mmio(PureRead)]
    interupt_mask: InterruptMask,
    spi_enable: SpiEnable,
    delay: Delay,
    /// Transmits 1-byte command and 3-byte data OR 4-byte data.
    #[mmio(Write)]
    tx_data_00: u32,
    #[mmio(PureRead)]
    rx_data: u32,
    slave_idle_count: u32,
    /// Defines the level at which the TX FIFO not full interrupt is generated.
    tx_fifo_threshold: u32,
    /// Defines the level at which the RX FIFO not empty interrupt is generated.
    rx_fifo_threshold: u32,
    gpio: Gpio,
    _reserved0: u32,
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
    linear_qspi_config: LinearQspiConfig,
    linear_qspi_status: LinearQspiStatus,
    _reserved3: [u32; 0x15],
    /// Module ID value with reset value 0x1090101.
    module_id: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Qspi>(), 0x100);

impl Qspi {
    /// Create a new QSPI MMIO instance for for QSPI controller at address [QSPI_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed() -> MmioQspi<'static> {
        unsafe { Self::new_mmio_at(QSPI_BASE_ADDR) }
    }
}
