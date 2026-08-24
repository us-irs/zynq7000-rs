//! I2C register module.
use arbitrary_int::{u2, u6, u10};

/// Base address of the I2C0 register block.
pub const I2C_0_BASE_ADDR: usize = 0xE000_4000;
/// Base address of the I2C1 register block.
pub const I2C_1_BASE_ADDR: usize = 0xE000_5000;

/// Selects the transfer direction in master mode.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// Master receives data.
    Receiver = 0b1,
    /// Master transmits data.
    Transmitter = 0b0,
}

/// Selects master or slave mode.
#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Slave mode.
    Slave = 0b0,
    /// Master mode.
    Master = 0b1,
}

/// I2C control register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_fields(feature = "defmt"),
    forbid_overlaps
)]
pub struct Control {
    /// Divides the input PCLK frequency by this value + 1
    #[bits(14..=15, rw)]
    div_a: u2,
    /// Divides the output from divisor A by this value + 1
    #[bits(8..=13, rw)]
    div_b: u6,
    /// Clears the FIFO, automatically cleared once the FIFO has been reset.
    #[bit(6, rw)]
    clear_fifo: bool,
    /// Enables slave monitor mode.
    #[bit(5, rw)]
    slv_mon: bool,
    /// 0: Allow transfer to terminate as soon as all data has been transmitted or received.
    /// 1: When no more data is avilable for transmit or no more data can be received, hold
    ///    the SCK line low until services by the host.
    #[bit(4, rw)]
    hold_bus: bool,
    /// Should be set to 1. 0: Disabled, NACK transmitted. 1: Enabled, ACK transmitted.
    #[bit(3, rw)]
    acken: bool,
    /// Only used in master mode. 0: Reserved. 1: Normal 7-bit address.
    #[bit(2, rw)]
    addressing: bool,
    /// Selects master or slave mode.
    #[bit(1, rw)]
    mode: Mode,
    /// Selects the transfer direction.
    #[bit(0, rw)]
    dir: Direction,
}

/// I2C status register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct Status {
    /// I2C bus is currently active, a start condition has been observed but no stop yet.
    #[bit(8, r)]
    bus_active: bool,
    /// FIFO is full and new byte was received. The new byte is not acknowledged and the contents
    /// of the FIFO remain unchanged.
    #[bit(7, r)]
    rx_overflow: bool,
    /// 1: There is still a byte of data to be transmitted by the interface.
    #[bit(6, r)]
    tx_busy: bool,
    /// Receiver data valid, ca be read from the interface.
    #[bit(5, r)]
    rx_valid: bool,
    /// R/W bit of the address received in slave mode.
    #[bit(3, r)]
    rx_rw: bool,
}

/// I2C slave address register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct Address {
    /// Slave address, 7-bit or 10-bit depending on the addressing mode.
    #[bits(0..=9, rw)]
    addr: u10,
}

/// I2C FIFO data register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct Fifo {
    /// Data byte transmitted to or received from the FIFO.
    #[bits(0..=7, rw)]
    data: u8,
}

/// Interrupt status register, cleared by writing a 1.
#[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
pub struct InterruptStatus {
    /// Master lost arbitration during a transfer.
    #[bit(9, rw)]
    arbitration_lost: bool,
    /// FIFO receive underflow, a read was attempted from an empty FIFO.
    #[bit(7, rw)]
    rx_underflow: bool,
    /// FIFO transmit overflow, a write was attempted to a full FIFO.
    #[bit(6, rw)]
    tx_overflow: bool,
    /// FIFO receive overflow, a byte was received while the FIFO was full.
    #[bit(5, rw)]
    rx_overflow: bool,
    /// Slave state cleared and ready to respond, monitored only in slave monitor mode.
    #[bit(4, rw)]
    slave_ready: bool,
    /// Transfer timeout occurred, SCL was held low for too long.
    #[bit(3, rw)]
    timeout: bool,
    /// Transfer was not acknowledged by the receiver.
    #[bit(2, rw)]
    nack: bool,
    /// FIFO is at or below the configured transfer size threshold.
    #[bit(1, rw)]
    data: bool,
    /// Transfer has completed.
    #[bit(0, rw)]
    complete: bool,
}

/// Interrupt mask register, shows the currently enabled interrupts.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct InterruptMask {
    /// Arbitration lost interrupt is enabled.
    #[bit(9, r)]
    arbitration_lost: bool,
    /// Receive FIFO underflow interrupt is enabled.
    #[bit(7, r)]
    rx_underflow: bool,
    /// Transmit FIFO overflow interrupt is enabled.
    #[bit(6, r)]
    tx_overflow: bool,
    /// Receive FIFO overflow interrupt is enabled.
    #[bit(5, r)]
    rx_overflow: bool,
    /// Slave ready interrupt is enabled.
    #[bit(4, r)]
    slave_ready: bool,
    /// Transfer timeout interrupt is enabled.
    #[bit(3, r)]
    timeout: bool,
    /// Transfer not acknowledged interrupt is enabled.
    #[bit(2, r)]
    nack: bool,
    /// FIFO data interrupt is enabled.
    #[bit(1, r)]
    data: bool,
    /// Transfer complete interrupt is enabled.
    #[bit(0, r)]
    complete: bool,
}

/// Interrupt enable or disable register, depending on which register instance is written.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct InterruptControl {
    /// Arbitration lost interrupt.
    #[bit(9, w)]
    arbitration_lost: bool,
    /// Receive FIFO underflow interrupt.
    #[bit(7, w)]
    rx_underflow: bool,
    /// Transmit FIFO overflow interrupt.
    #[bit(6, w)]
    tx_overflow: bool,
    /// Receive FIFO overflow interrupt.
    #[bit(5, w)]
    rx_overflow: bool,
    /// Slave ready interrupt.
    #[bit(4, w)]
    slave_ready: bool,
    /// Transfer timeout interrupt.
    #[bit(3, w)]
    timeout: bool,
    /// Transfer not acknowledged interrupt.
    #[bit(2, w)]
    nack: bool,
    /// FIFO data interrupt.
    #[bit(1, w)]
    data: bool,
    /// Transfer complete interrupt.
    #[bit(0, w)]
    complete: bool,
}

/// I2C SCL timeout register.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct Timeout {
    /// Reset value: 0x1F.
    #[bits(0..=7, rw)]
    timeout: u8,
}

/// Transfer size register, holds the number of bytes to transfer or the FIFO threshold
/// depending on mode.
#[bitbybit::bitfield(
    u32,
    default = 0x0,
    debug,
    defmt_bitfields(feature = "defmt"),
    forbid_overlaps
)]
pub struct TransferSize {
    /// Number of bytes to transfer, or FIFO fill threshold in slave monitor mode.
    #[bits(0..=7, rw)]
    size: u8,
}

/// I2C register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Control register.
    control: Control,
    /// Status register.
    #[mmio(PureRead)]
    status: Status,
    /// Slave address register.
    addr: Address,
    /// FIFO data register.
    #[mmio(Read, Write)]
    data: Fifo,
    /// Interrupt status register.
    #[mmio(PureRead, Write, Modify)]
    interrupt_status: InterruptStatus,
    /// Transfer size register.
    transfer_size: TransferSize,
    /// Slave monitor mode pause interval between successive address polls.
    slave_pause: u32,
    /// SCL timeout register.
    timeout: Timeout,
    /// Interrupt mask register, shows the currently enabled interrupts.
    #[mmio(PureRead)]
    enabled_interrupts: InterruptMask,
    /// Interrupt enable register, write 1 to enable an interrupt.
    #[mmio(Write)]
    interrupt_enable: InterruptControl,
    /// Interrupt disable register, write 1 to disable an interrupt.
    #[mmio(Write)]
    interrupt_disable: InterruptControl,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x2C);

impl Registers {
    /// Create a new I2C MMIO instance for I2C0 at address [I2C_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(I2C_0_BASE_ADDR) }
    }

    /// Create a new I2C MMIO instance for I2C1 at address [I2C_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(I2C_1_BASE_ADDR) }
    }
}
