//! SPI register module.
use arbitrary_int::{u2, u6, u10};

pub const I2C_0_BASE_ADDR: usize = 0xE000_4000;
pub const I2C_1_BASE_ADDR: usize = 0xE000_5000;

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum Direction {
    Receiver = 0b1,
    Transmitter = 0b0,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum Mode {
    Slave = 0b0,
    Master = 0b1,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct Control {
    /// Divides the input PCLK frequency by this value + 1
    #[bits(14..=15, rw)]
    div_a: u2,
    /// Divides the output from divisor A by this value + 1
    #[bits(8..=13, rw)]
    div_b: u6,
    #[bit(6, rw)]
    clear_fifo: bool,
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
    #[bit(1, rw)]
    mode: Mode,
    #[bit(0, rw)]
    dir: Direction,
}

#[bitbybit::bitfield(u32)]
pub struct Status {
    #[bit(8, r)]
    bus_active: bool,
    /// FIFO is full and new byte was received. The new byte is not acknowledged and the contents
    /// of the FIFO remain unchanged.
    #[bit(6, r)]
    rx_overflow: bool,
    /// 1: There is still a byte of data to be transmitted by the interface.
    #[bit(6, r)]
    tx_busy: bool,
    /// Receiver data valid, ca be read from the interface.
    #[bit(5, r)]
    rx_valid: bool,
    #[bit(3, r)]
    rx_rw: bool,
}

#[bitbybit::bitfield(u32)]
pub struct Addr {
    #[bits(0..=9, rw)]
    addr: u10,
}

#[bitbybit::bitfield(u32)]
pub struct Fifo {
    #[bits(0..=7, rw)]
    data: u8,
}

#[bitbybit::bitfield(u32)]
pub struct InterruptStatus {
    #[bit(9, rw)]
    arbitration_lost: bool,
    #[bit(7, rw)]
    rx_underflow: bool,
    #[bit(6, rw)]
    tx_overflow: bool,
    #[bit(5, rw)]
    rx_overflow: bool,
    #[bit(4, rw)]
    slave_ready: bool,
    #[bit(3, rw)]
    timeout: bool,
    #[bit(2, rw)]
    nack: bool,
    #[bit(1, rw)]
    data: bool,
    #[bit(0, rw)]
    complete: bool,
}

#[bitbybit::bitfield(u32)]
pub struct InterruptMask {
    #[bit(9, r)]
    arbitration_lost: bool,
    #[bit(7, r)]
    rx_underflow: bool,
    #[bit(6, r)]
    tx_overflow: bool,
    #[bit(5, r)]
    rx_overflow: bool,
    #[bit(4, r)]
    slave_ready: bool,
    #[bit(3, r)]
    timeout: bool,
    #[bit(2, r)]
    nack: bool,
    #[bit(1, r)]
    data: bool,
    #[bit(0, r)]
    complete: bool,
}

#[bitbybit::bitfield(u32)]
pub struct InterruptControl {
    #[bit(9, w)]
    arbitration_lost: bool,
    #[bit(7, w)]
    rx_underflow: bool,
    #[bit(6, w)]
    tx_overflow: bool,
    #[bit(5, w)]
    rx_overflow: bool,
    #[bit(4, w)]
    slave_ready: bool,
    #[bit(3, w)]
    timeout: bool,
    #[bit(2, w)]
    nack: bool,
    #[bit(1, w)]
    data: bool,
    #[bit(0, w)]
    complete: bool,
}

#[bitbybit::bitfield(u32)]
pub struct Timeout {
    /// Reset value: 0x1F.
    #[bits(0..=7, rw)]
    timeout: u8,
}

#[bitbybit::bitfield(u32, default = 0x0)]
pub struct TransferSize {
    #[bits(0..=7, rw)]
    size: u8,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct I2c {
    cr: Control,
    #[mmio(PureRead)]
    sr: Status,
    addr: Addr,
    #[mmio(Read, Write)]
    data: Fifo,
    #[mmio(PureRead, Write, Modify)]
    isr: InterruptStatus,
    transfer_size: TransferSize,
    slave_pause: u32,
    timeout: Timeout,
    #[mmio(PureRead)]
    imr: InterruptMask,
    #[mmio(Write)]
    ier: InterruptControl,
    #[mmio(Write)]
    idr: InterruptControl,
}

impl I2c {
    /// Create a new I2C MMIO instance for I2C0 at address [I2C_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioI2c<'static> {
        unsafe { Self::new_mmio_at(I2C_0_BASE_ADDR) }
    }

    /// Create a new I2C MMIO instance for I2C1 at address [I2C_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioI2c<'static> {
        unsafe { Self::new_mmio_at(I2C_1_BASE_ADDR) }
    }
}
