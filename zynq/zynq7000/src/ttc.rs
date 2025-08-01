//! Triple-timer counter (TTC) register module.
use arbitrary_int::u4;

pub const TTC_0_BASE_ADDR: usize = 0xF800_1000;
pub const TTC_1_BASE_ADDR: usize = 0xF800_2000;

#[derive(Debug, Default)]
#[bitbybit::bitenum(u1, exhaustive = true)]
pub enum ClockSource {
    /// PS internal bus clock.
    #[default]
    Pclk = 0b0,
    External = 0b1,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct ClockControl {
    /// When this bit is set and the external clock is selected, the counter clocks on the
    /// negative edge of the external clock input.
    #[bit(6, rw)]
    ext_clk_edge: bool,
    #[bit(5, rw)]
    clk_src: ClockSource,
    #[bits(1..=4, rw)]
    prescaler: u4,
    #[bit(0, rw)]
    prescale_enable: bool,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum Mode {
    Overflow = 0b0,
    Interval = 0b1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug, Default)]
pub enum WavePolarity {
    /// The waveform output goes from high to low on a match 0 interrupt and returns high on
    /// overflow or interval interrupt.
    #[default]
    HighToLowOnMatch1 = 0b0,
    /// The waveform output goes from low to high on a match 0 interrupt and returns low on
    /// overflow or interval interrupt.
    LowToHighOnMatch1 = 0b1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum WaveEnable {
    Enable = 0b0,
    Disable = 0b1,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct CounterControl {
    #[bit(6, rw)]
    wave_polarity: WavePolarity,
    /// Output waveform enable, active low. Reset value 1.
    #[bit(5, rw)]
    wave_enable_n: WaveEnable,
    /// Resets the counter and restarts counting. Automatically cleared on restart.
    #[bit(4, rw)]
    reset: bool,
    /// When this bit is set, an interrupt is generated when the count value matches one of the
    /// three match registers and the corresponding bit is set in the IER register.
    #[bit(3, rw)]
    match_enable: bool,
    /// When this bit is high, the timer counts down.
    #[bit(2, rw)]
    decrementing: bool,
    #[bit(1, rw)]
    mode: Mode,
    #[bit(0, rw)]
    disable: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct Counter {
    #[bits(0..=15, r)]
    count: u16,
}

#[bitbybit::bitfield(u32, debug)]
pub struct RwValue {
    #[bits(0..=15, rw)]
    value: u16,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptStatus {
    /// Even timer overflow interrupt.
    #[bit(5, r)]
    event: bool,
    #[bit(4, r)]
    counter_overflow: bool,
    #[bit(3, r)]
    match_2: bool,
    #[bit(2, r)]
    match_1: bool,
    #[bit(1, r)]
    match_0: bool,
    #[bit(0, r)]
    interval: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptControl {
    /// Even timer overflow interrupt.
    #[bit(5, rw)]
    event: bool,
    #[bit(4, rw)]
    counter_overflow: bool,
    #[bit(3, rw)]
    match_2: bool,
    #[bit(2, rw)]
    match_1: bool,
    #[bit(1, rw)]
    match_0: bool,
    #[bit(0, rw)]
    interval: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct EventControl {
    /// E_Ov bit. When set to 0, the event timer is disabled and set to 0 when an event timer
    /// register overflow occurs. Otherwise, continue counting on overflow.
    #[bit(2, rw)]
    continuous_mode: bool,
    /// E_Lo bit. When set to 1, counts PCLK cycles during low level duration of the external
    /// clock. Otherwise, counts it during high level duration.
    #[bit(1, rw)]
    count_low_level_of_ext_clk: bool,
    #[bit(0, rw)]
    enable: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct EventCount {
    #[bits(0..=15, r)]
    count: u16,
}

/// Triple-timer counter
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Ttc {
    clk_cntr: [ClockControl; 3],
    cnt_ctrl: [CounterControl; 3],
    #[mmio(PureRead)]
    current_counter: [Counter; 3],
    interval_value: [RwValue; 3],
    match_value_0: [RwValue; 3],
    match_value_1: [RwValue; 3],
    match_value_2: [RwValue; 3],
    #[mmio(Read)]
    isr: [InterruptStatus; 3],
    ier: [InterruptControl; 3],
    event_cntrl: [EventControl; 3],
    #[mmio(PureRead)]
    event_reg: [EventCount; 3],
}

static_assertions::const_assert_eq!(core::mem::size_of::<Ttc>(), 0x84);

impl Ttc {
    /// Create a new TTC MMIO instance for TTC0 at address [TTC_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioTtc<'static> {
        unsafe { Self::new_mmio_at(TTC_0_BASE_ADDR) }
    }

    /// Create a new TTC MMIO instance for TTC1 at address [TTC_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioTtc<'static> {
        unsafe { Self::new_mmio_at(TTC_1_BASE_ADDR) }
    }
}
