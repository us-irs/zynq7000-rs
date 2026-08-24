//! PS UART register module.

/// Base address of the UART 0 register block.
pub const UART_0_BASE: usize = 0xE000_0000;
/// Base address of the UART 1 register block.
pub const UART_1_BASE: usize = 0xE000_1000;

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::u6;

    /// Parity mode.
    #[bitbybit::bitenum(u3, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Parity {
        /// Even parity.
        Even = 0b000,
        /// Odd parity.
        Odd = 0b001,
        /// Forced to 0 (Space)
        ForcedTo0 = 0b010,
        /// Forced to 1 (Mark)
        ForcedTo1 = 0b011,
        /// No parity.
        NoParity = 0b100,
        /// No parity, alternate encoding.
        NoParityAlt0 = 0b101,
        /// No parity, alternate encoding.
        NoParityAlt1 = 0b110,
        /// No parity, alternate encoding.
        NoParityAlt2 = 0b111,
    }

    /// Character length.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Default, Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum CharLen {
        /// 6 data bits.
        SixBits = 0b11,
        /// 7 data bits.
        SevenBits = 0b10,
        /// 8 data bits.
        #[default]
        EightBits = 0b00,
        /// 8 data bits, alternate encoding.
        EightBitsAlt = 0b01,
    }

    /// Baud rate generator input clock selection.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Default, Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ClockSelect {
        /// Use the UART reference clock directly.
        #[default]
        UartRefClk = 0b0,
        /// Use the UART reference clock divided by 8.
        UartRefClkDiv8 = 0b1,
    }

    /// Number of stop bits.
    #[bitbybit::bitenum(u2)]
    #[derive(Default, Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Stopbits {
        /// One stop bit.
        #[default]
        One = 0b00,
        /// 1.5 stop bits.
        OnePointFive = 0b01,
        /// Two stop bits.
        Two = 0b10,
    }

    /// Channel mode, controlling normal operation, echo and loopback.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ChMode {
        /// Normal operation.
        #[default]
        Normal = 0b00,
        /// Automatic echo, received data is retransmitted on the TX line.
        AutoEcho = 0b01,
        /// Local loopback, TX is internally connected to RX.
        LocalLoopback = 0b10,
        /// Remote loopback, RX data is looped back onto TX.
        RemoteLoopback = 0b11,
    }

    /// UART control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        forbid_overlaps,
        defmt_bitfields(feature = "defmt")
    )]
    pub struct Control {
        /// Stop transmitter break.
        #[bit(8, rw)]
        stopbrk: bool,
        /// Start transmitter break.
        #[bit(7, rw)]
        startbrk: bool,
        /// Restart receiver timeout counter.
        #[bit(6, rw)]
        restart_timeout: bool,
        /// TX disable. If this is 1, TX is disabled, regardless of TXEN.
        #[bit(5, rw)]
        tx_disable: bool,
        /// TX enable. TX will be enabled if this bit is 1 and the TXDIS is 0.
        #[bit(4, rw)]
        tx_enable: bool,
        /// RX disable. If this is 1, RX is disabled, regardless of RXEN.
        #[bit(3, rw)]
        rx_disable: bool,
        /// RX enable. RX will be enabled if this bit is 1 and the RXDIS is 0.
        #[bit(2, rw)]
        rx_enable: bool,
        /// TX soft reset.
        #[bit(1, rw)]
        tx_reset: bool,
        /// RX soft reset.
        #[bit(0, rw)]
        rx_reset: bool,
    }

    /// UART mode register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        forbid_overlaps,
        defmt_fields(feature = "defmt")
    )]
    pub struct Mode {
        /// Channel mode.
        #[bits(8..=9, rw)]
        chmode: ChMode,
        /// Number of stop bits.
        #[bits(6..=7, rw)]
        stopbits: Option<Stopbits>,
        /// Parity mode.
        #[bits(3..=5, rw)]
        parity: Parity,
        /// Char length.
        #[bits(1..=2, rw)]
        charlen: CharLen,
        /// Baud rate generator input clock selection.
        #[bit(0, rw)]
        clock_select: ClockSelect,
    }

    /// Baud rate generator register.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        forbid_overlaps,
        defmt_fields(feature = "defmt")
    )]
    pub struct Baudgen {
        /// Baud rate clock divisor.
        #[bits(0..=15, rw)]
        cd: u16,
    }

    /// Baud rate divider register.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        forbid_overlaps,
        defmt_fields(feature = "defmt")
    )]
    pub struct BaudRateDivisor {
        /// Baud rate divisor.
        #[bits(0..=7, rw)]
        bdiv: u8,
    }

    /// TX/RX FIFO data register.
    #[bitbybit::bitfield(u32, debug, forbid_overlaps, defmt_fields(feature = "defmt"))]
    pub struct Fifo {
        /// FIFO data byte.
        #[bits(0..=7, rw)]
        fifo: u8,
    }

    /// TX FIFO fill level relative to the trigger threshold.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Ttrig {
        /// TX FIFO fill level is below the trigger threshold.
        LessThanTTrig = 0b0,
        /// TX FIFO fill level is at or above the trigger threshold.
        GreaterEqualTTrig = 0b1,
    }

    /// Channel status register.
    #[bitbybit::bitfield(u32, debug, forbid_overlaps, defmt_bitfields(feature = "defmt"))]
    pub struct Status {
        /// TX FIFO fill level is above the TX empty trigger level.
        #[bit(14, r)]
        tx_near_full: bool,
        /// TX FIFO fill level relative to the trigger threshold.
        #[bit(13, r)]
        tx_trigger: Ttrig,
        /// RX flow delay trigger reached.
        #[bit(12, r)]
        flowdel: bool,
        /// Transmitter state machine active.
        #[bit(11, r)]
        tx_active: bool,
        /// Receiver state machine active.
        #[bit(10, r)]
        rx_active: bool,
        /// TX FIFO full.
        #[bit(4, r)]
        tx_full: bool,
        /// TX FIFO empty.
        #[bit(3, r)]
        tx_empty: bool,
        /// RX FIFO full.
        #[bit(2, r)]
        rx_full: bool,
        /// RX FIFO empty.
        #[bit(1, r)]
        rx_empty: bool,
        /// RX FIFO trigger level was reached.
        #[bit(0, r)]
        rx_trigger: bool,
    }

    /// Interrupt enable register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        forbid_overlaps,
        defmt_bitfields(feature = "defmt")
    )]
    pub struct InterruptControl {
        /// TX FIFO overflow interrupt.
        #[bit(12, w)]
        tx_over: bool,
        /// TX FIFO near full interrupt.
        #[bit(11, w)]
        tx_near_full: bool,
        /// TX FIFO trigger interrupt.
        #[bit(10, w)]
        tx_trigger: bool,
        /// Delta modem status interrupt.
        #[bit(9, w)]
        rx_dms: bool,
        /// Receiver timeout error interrupt.
        #[bit(8, w)]
        rx_timeout: bool,
        /// Receiver parity error interrupt.
        #[bit(7, w)]
        rx_parity: bool,
        /// Receiver framing error interrupt.
        #[bit(6, w)]
        rx_framing: bool,
        /// Receiver overflow error interrupt.
        #[bit(5, w)]
        rx_over: bool,
        /// TX FIFO full interrupt.
        #[bit(4, w)]
        tx_full: bool,
        /// TX FIFO empty interrupt.
        #[bit(3, w)]
        tx_empty: bool,
        /// RX FIFO full interrupt.
        #[bit(2, w)]
        rx_full: bool,
        /// RX FIFO empty interrupt.
        #[bit(1, w)]
        rx_empty: bool,
        /// RX FIFO trigger interrupt.
        #[bit(0, w)]
        rx_trigger: bool,
    }

    /// FIFO trigger level register, shared by the RX and TX FIFO trigger registers.
    #[bitbybit::bitfield(u32, default = 0x0, forbid_overlaps)]
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct FifoTrigger {
        /// FIFO trigger level.
        #[bits(0..=5, rw)]
        trigger: u6,
    }

    /// Interrupt mask register, showing which interrupts are currently enabled.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    #[derive(PartialEq, Eq)]
    pub struct InterruptMask {
        /// TX FIFO overflow interrupt enabled.
        #[bit(12, r)]
        tx_over: bool,
        /// TX FIFO near full interrupt enabled.
        #[bit(11, r)]
        tx_near_full: bool,
        /// TX FIFO trigger interrupt enabled.
        #[bit(10, r)]
        tx_trigger: bool,
        /// Delta modem status interrupt enabled.
        #[bit(9, r)]
        rx_dms: bool,
        /// Receiver timeout error interrupt.
        #[bit(8, r)]
        rx_timeout: bool,
        /// Receiver parity error interrupt enabled.
        #[bit(7, r)]
        rx_parity: bool,
        /// Receiver framing error interrupt enabled.
        #[bit(6, r)]
        rx_framing: bool,
        /// Receiver overflow error interrupt enabled.
        #[bit(5, r)]
        rx_over: bool,
        /// TX FIFO full interrupt enabled.
        #[bit(4, r)]
        tx_full: bool,
        /// TX FIFO empty interrupt enabled.
        #[bit(3, r)]
        tx_empty: bool,
        /// RX FIFO full interrupt enabled.
        #[bit(2, r)]
        rx_full: bool,
        /// RX FIFO empty interrupt enabled.
        #[bit(1, r)]
        rx_empty: bool,
        /// RX FIFO trigger level reached.
        #[bit(0, r)]
        rx_trigger: bool,
    }

    /// Interrupt status register, showing and allowing clearing of pending interrupts.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptStatus {
        /// TX FIFO overflow interrupt pending.
        #[bit(12, rw)]
        tx_over: bool,
        /// TX FIFO near full interrupt pending.
        #[bit(11, rw)]
        tx_near_full: bool,
        /// TX FIFO trigger interrupt pending.
        #[bit(10, rw)]
        tx_trig: bool,
        /// Delta modem status interrupt pending.
        #[bit(9, rw)]
        rx_dms: bool,
        /// Receiver timeout error interrupt.
        #[bit(8, rw)]
        rx_timeout: bool,
        /// Receiver parity error interrupt pending.
        #[bit(7, rw)]
        rx_parity: bool,
        /// Receiver framing error interrupt pending.
        #[bit(6, rw)]
        rx_framing: bool,
        /// Receiver overflow error interrupt pending.
        #[bit(5, rw)]
        rx_over: bool,
        /// TX FIFO full interrupt pending.
        #[bit(4, rw)]
        tx_full: bool,
        /// TX FIFO empty interrupt pending.
        #[bit(3, rw)]
        tx_empty: bool,
        /// RX FIFO full interrupt pending.
        #[bit(2, rw)]
        rx_full: bool,
        /// RX FIFO empty interrupt pending.
        #[bit(1, rw)]
        rx_empty: bool,
        /// RX FIFO trigger level reached.
        #[bit(0, rw)]
        rx_trigger: bool,
    }

    impl InterruptStatus {
        /// Build a status value clearing the RX parity, framing and overflow errors.
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
                .with_rx_trigger(false)
                .build()
        }
    }
}

/// UART register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Control Register
    control: Control,
    /// Mode register
    mode: Mode,
    /// Interrupt enable register
    #[mmio(Write)]
    interrupt_enable: InterruptControl,
    /// Interrupt disable register
    #[mmio(Write)]
    interrupt_disable: InterruptControl,
    /// Interrupt mask register, showing enabled interrupts.
    #[mmio(PureRead)]
    enabled_interrupts: InterruptMask,
    /// Interrupt status register
    #[mmio(PureRead, Write)]
    interrupt_status: InterruptStatus,
    /// Baudgen register
    baudgen: Baudgen,
    /// RX timeout register
    rx_timeout: u32,
    /// RX FIFO trigger level register
    rx_fifo_trigger: FifoTrigger,
    /// Modem control register
    modem_control: u32,
    /// Modem status register
    modem_status: u32,
    /// Channel status register
    #[mmio(PureRead)]
    status: Status,
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

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x48);

impl Registers {
    /// Create a new UART MMIO instance for uart0 at address 0xE000_0000.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(UART_0_BASE) }
    }

    /// Create a new UART MMIO instance for uart1 at address 0xE000_1000.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(UART_1_BASE) }
    }
}
