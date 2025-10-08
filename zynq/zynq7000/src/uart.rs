//! PS UART register module.
use arbitrary_int::u6;

pub const UART_0_BASE: usize = 0xE000_0000;
pub const UART_1_BASE: usize = 0xE000_1000;

#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug)]
pub enum Parity {
    Even = 0b000,
    Odd = 0b001,
    /// Forced to 0 (Space)
    ForcedTo0 = 0b010,
    /// Forced to 1 (Mark)
    ForcedTo1 = 0b011,
    NoParity = 0b100,
    NoParityAlt0 = 0b101,
    NoParityAlt1 = 0b110,
    NoParityAlt2 = 0b111,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Default, Debug, PartialEq, Eq)]
pub enum CharLen {
    SixBits = 0b11,
    SevenBits = 0b10,
    #[default]
    EightBits = 0b00,
    EightBitsAlt = 0b01,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Default, Debug, PartialEq, Eq)]
pub enum ClockSelect {
    #[default]
    UartRefClk = 0b0,
    UartRefClkDiv8 = 0b1,
}

#[bitbybit::bitenum(u2)]
#[derive(Default, Debug, PartialEq, Eq)]
pub enum Stopbits {
    #[default]
    One = 0b00,
    OnePointFive = 0b01,
    Two = 0b10,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, Default)]
pub enum ChMode {
    #[default]
    Normal = 0b00,
    AutoEcho = 0b01,
    LocalLoopback = 0b10,
    RemoteLoopback = 0b11,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Control {
    /// Stop transmitter break.
    #[bit(8, rw)]
    stopbrk: bool,
    /// Start transmitter break.
    #[bit(7, rw)]
    startbrk: bool,
    /// Restart receiver timeout counter.
    #[bit(6, rw)]
    rstto: bool,
    /// TX disable. If this is 1, TX is disabled, regardless of TXEN.
    #[bit(5, rw)]
    tx_dis: bool,
    /// TX enable. TX will be enabled if this bit is 1 and the TXDIS is 0.
    #[bit(4, rw)]
    tx_en: bool,
    /// RX disable. If this is 1, RX is disabled, regardless of RXEN.
    #[bit(3, rw)]
    rx_dis: bool,
    /// RX enable. RX will be enabled if this bit is 1 and the RXDIS is 0.
    #[bit(2, rw)]
    rx_en: bool,
    /// TX soft reset.
    #[bit(1, rw)]
    tx_rst: bool,
    /// RX soft reset.
    #[bit(0, rw)]
    rx_rst: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct Mode {
    #[bits(8..=9, rw)]
    chmode: ChMode,
    #[bits(6..=7, rw)]
    nbstop: Option<Stopbits>,
    #[bits(3..=5, rw)]
    par: Parity,
    /// Char length.
    #[bits(1..=2, rw)]
    chrl: CharLen,
    #[bit(0, rw)]
    clksel: ClockSelect,
}

#[bitbybit::bitfield(u32, default = 0, debug)]
pub struct Baudgen {
    #[bits(0..=15, rw)]
    cd: u16,
}

#[bitbybit::bitfield(u32, default = 0, debug)]
pub struct BaudRateDivisor {
    #[bits(0..=7, rw)]
    bdiv: u8,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Fifo {
    #[bits(0..=7, rw)]
    fifo: u8,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum Ttrig {
    LessThanTTrig = 0b0,
    GreaterEqualTTrig = 0b1,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Status {
    #[bit(14, r)]
    tx_near_full: bool,
    #[bit(13, r)]
    tx_trig: Ttrig,
    #[bit(12, r)]
    flowdel: bool,
    /// Transmitter state machine active.
    #[bit(11, r)]
    tx_active: bool,
    /// Receiver state machine active.
    #[bit(10, r)]
    rx_active: bool,
    #[bit(4, r)]
    tx_full: bool,
    #[bit(3, r)]
    tx_empty: bool,
    #[bit(2, r)]
    rx_full: bool,
    #[bit(1, r)]
    rx_empty: bool,
    /// RX FIFO trigger level was reached.
    #[bit(0, r)]
    rx_trg: bool,
}

#[bitbybit::bitfield(u32, default = 0x0)]
#[derive(Debug)]
pub struct InterruptControl {
    #[bit(12, w)]
    tx_over: bool,
    #[bit(11, w)]
    tx_near_full: bool,
    #[bit(10, w)]
    tx_trig: bool,
    #[bit(9, w)]
    rx_dms: bool,
    /// Receiver timeout error interrupt.
    #[bit(8, w)]
    rx_timeout: bool,
    #[bit(7, w)]
    rx_parity: bool,
    #[bit(6, w)]
    rx_framing: bool,
    #[bit(5, w)]
    rx_over: bool,
    #[bit(4, w)]
    tx_full: bool,
    #[bit(3, w)]
    tx_empty: bool,
    #[bit(2, w)]
    rx_full: bool,
    #[bit(1, w)]
    rx_empty: bool,
    #[bit(0, w)]
    rx_trg: bool,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct FifoTrigger {
    #[bits(0..=5, rw)]
    trig: u6,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct InterruptMask {
    #[bit(12, r)]
    tx_over: bool,
    #[bit(11, r)]
    tx_near_full: bool,
    #[bit(10, r)]
    tx_trig: bool,
    #[bit(9, r)]
    rx_dms: bool,
    /// Receiver timeout error interrupt.
    #[bit(8, r)]
    rx_timeout: bool,
    #[bit(7, r)]
    rx_parity: bool,
    #[bit(6, r)]
    rx_framing: bool,
    #[bit(5, r)]
    rx_over: bool,
    #[bit(4, r)]
    tx_full: bool,
    #[bit(3, r)]
    tx_empty: bool,
    #[bit(2, r)]
    rx_full: bool,
    #[bit(1, r)]
    rx_empty: bool,
    /// RX FIFO trigger level reached.
    #[bit(0, r)]
    rx_trg: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct InterruptStatus {
    #[bit(12, rw)]
    tx_over: bool,
    #[bit(11, rw)]
    tx_near_full: bool,
    #[bit(10, rw)]
    tx_trig: bool,
    #[bit(9, rw)]
    rx_dms: bool,
    /// Receiver timeout error interrupt.
    #[bit(8, rw)]
    rx_timeout: bool,
    #[bit(7, rw)]
    rx_parity: bool,
    #[bit(6, rw)]
    rx_framing: bool,
    #[bit(5, rw)]
    rx_over: bool,
    #[bit(4, rw)]
    tx_full: bool,
    #[bit(3, rw)]
    tx_empty: bool,
    #[bit(2, rw)]
    rx_full: bool,
    #[bit(1, rw)]
    rx_empty: bool,
    /// RX FIFO trigger level reached.
    #[bit(0, rw)]
    rx_trg: bool,
}

impl InterruptStatus {
    pub fn new_for_clearing_rx_errors() -> Self {
        Self::builder()
            .with_tx_over(false)
            .with_tx_near_full(false)
            .with_tx_trig(false)
            .with_rx_dms(false)
            .with_rx_timeout(false)
            .with_rx_parity(true)
            .with_rx_framing(true)
            .with_rx_over(true)
            .with_tx_full(false)
            .with_tx_empty(false)
            .with_rx_full(false)
            .with_rx_empty(false)
            .with_rx_trg(false)
            .build()
    }
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Uart {
    /// Control Register
    cr: Control,
    /// Mode register
    mr: Mode,
    /// Interrupt enable register
    #[mmio(Write)]
    ier: InterruptControl,
    /// Interrupt disable register
    #[mmio(Write)]
    idr: InterruptControl,
    /// Interrupt mask register, showing enabled interrupts.
    #[mmio(PureRead)]
    imr: InterruptMask,
    /// Interrupt status register
    #[mmio(PureRead, Write)]
    isr: InterruptStatus,
    /// Baudgen register
    baudgen: Baudgen,
    /// RX timeout register
    rx_tout: u32,
    /// RX FIFO trigger level register
    rx_fifo_trigger: FifoTrigger,
    /// Modem control register
    modem_cr: u32,
    /// Modem status register
    modem_sr: u32,
    /// Channel status register
    #[mmio(PureRead)]
    sr: Status,
    /// FIFO register
    #[mmio(Read, Write)]
    fifo: Fifo,
    /// Baud rate divider register
    baud_rate_div: BaudRateDivisor,
    /// Flow control delay register
    flow_delay: u32,

    _reserved: [u32; 2],

    /// TX fifo trigger level
    tx_fifo_trigger: FifoTrigger,
}

static_assertions::const_assert_eq!(core::mem::size_of::<Uart>(), 0x48);

impl Uart {
    /// Create a new UART MMIO instance for uart0 at address 0xE000_0000.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioUart<'static> {
        unsafe { Self::new_mmio_at(UART_0_BASE) }
    }

    /// Create a new UART MMIO instance for uart1 at address 0xE000_1000.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioUart<'static> {
        unsafe { Self::new_mmio_at(UART_1_BASE) }
    }
}
