//! Triple-timer counter (TTC) register module.

/// Base address of the TTC0 register block.
pub const TTC_0_BASE_ADDR: usize = 0xF800_1000;
/// Base address of the TTC1 register block.
pub const TTC_1_BASE_ADDR: usize = 0xF800_2000;

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::u4;

    /// Selects the clock source for a counter.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, Default, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ClockSource {
        /// PS internal bus clock.
        #[default]
        Pclk = 0b0,
        /// External clock input.
        External = 0b1,
    }

    /// Clock control register, selects clock source, edge and prescaler for a counter.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ClockControl {
        /// When this bit is set and the external clock is selected, the counter clocks on the
        /// negative edge of the external clock input.
        #[bit(6, rw)]
        ext_clk_edge: bool,
        /// Selects the clock source.
        #[bit(5, rw)]
        clk_src: ClockSource,
        /// Prescaler value, the clock is divided by 2^(prescaler + 1) when enabled.
        #[bits(1..=4, rw)]
        prescaler: u4,
        /// Enables the prescaler.
        #[bit(0, rw)]
        prescale_enable: bool,
    }

    /// Selects between overflow mode and interval mode for a counter.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Mode {
        /// Counter runs continuously and overflows at 0xFFFF.
        Overflow = 0b0,
        /// Counter resets to zero after reaching the interval value.
        Interval = 0b1,
    }

    /// Selects the polarity of the waveform output on a match or overflow event.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, Default, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum WavePolarity {
        /// The waveform output goes from high to low on a match 0 interrupt and returns high on
        /// overflow or interval interrupt.
        #[default]
        HighToLowOnMatch1 = 0b0,
        /// The waveform output goes from low to high on a match 0 interrupt and returns low on
        /// overflow or interval interrupt.
        LowToHighOnMatch1 = 0b1,
    }

    /// Enables or disables the waveform output.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum WaveEnable {
        /// Waveform output is enabled.
        Enable = 0b0,
        /// Waveform output is disabled.
        Disable = 0b1,
    }

    /// Counter control register, configures reset, mode, direction and waveform output of a counter.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CounterControl {
        /// Selects the waveform output polarity.
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
        /// Selects overflow or interval mode.
        #[bit(1, rw)]
        mode: Mode,
        /// Disables the counter.
        #[bit(0, rw)]
        disable: bool,
    }

    /// Current counter value register.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct Counter {
        /// Current counter value.
        #[bits(0..=15, r)]
        count: u16,
    }

    /// Generic 16-bit read-write register used for the interval and match value registers.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct RwValue {
        /// Register value.
        #[bits(0..=15, rw)]
        value: u16,
    }

    /// Interrupt status register, cleared on read.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct InterruptStatus {
        /// Even timer overflow interrupt.
        #[bit(5, r)]
        event: bool,
        /// Counter overflow occurred.
        #[bit(4, r)]
        counter_overflow: bool,
        /// Counter value matched match register 2.
        #[bit(3, r)]
        match_2: bool,
        /// Counter value matched match register 1.
        #[bit(2, r)]
        match_1: bool,
        /// Counter value matched match register 0.
        #[bit(1, r)]
        match_0: bool,
        /// Interval interrupt occurred.
        #[bit(0, r)]
        interval: bool,
    }

    /// Interrupt enable register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct InterruptControl {
        /// Even timer overflow interrupt.
        #[bit(5, rw)]
        event: bool,
        /// Enables the counter overflow interrupt.
        #[bit(4, rw)]
        counter_overflow: bool,
        /// Enables the match register 2 interrupt.
        #[bit(3, rw)]
        match_2: bool,
        /// Enables the match register 1 interrupt.
        #[bit(2, rw)]
        match_1: bool,
        /// Enables the match register 0 interrupt.
        #[bit(1, rw)]
        match_0: bool,
        /// Enables the interval interrupt.
        #[bit(0, rw)]
        interval: bool,
    }

    /// Event timer control register, configures the external event timer.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct EventControl {
        /// E_Ov bit. When set to 0, the event timer is disabled and set to 0 when an event timer
        /// register overflow occurs. Otherwise, continue counting on overflow.
        #[bit(2, rw)]
        continuous_mode: bool,
        /// E_Lo bit. When set to 1, counts PCLK cycles during low level duration of the external
        /// clock. Otherwise, counts it during high level duration.
        #[bit(1, rw)]
        count_low_level_of_ext_clk: bool,
        /// Enables the event timer.
        #[bit(0, rw)]
        enable: bool,
    }

    /// Event timer counter register, holds the count of external clock edges.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct EventCount {
        /// Event counter value.
        #[bits(0..=15, r)]
        count: u16,
    }
}

/// Triple-timer counter register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    /// Clock control registers for the three counters.
    clk_cntr: [ClockControl; 3],
    /// Counter control registers for the three counters.
    cnt_ctrl: [CounterControl; 3],
    /// Current counter value registers.
    #[mmio(PureRead)]
    current_counter: [Counter; 3],
    /// Interval value registers.
    interval_value: [RwValue; 3],
    /// Match register 0 for each counter.
    match_value_0: [RwValue; 3],
    /// Match register 1 for each counter.
    match_value_1: [RwValue; 3],
    /// Match register 2 for each counter.
    match_value_2: [RwValue; 3],
    /// Interrupt status registers, cleared on read.
    #[mmio(Read)]
    interrupt_status: [InterruptStatus; 3],
    /// Interrupt enable registers.
    interrupt_enable: [InterruptControl; 3],
    /// Event timer control registers.
    event_control: [EventControl; 3],
    /// Event timer counter registers.
    #[mmio(PureRead)]
    event_reg: [EventCount; 3],
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x84);

impl Registers {
    /// Create a new TTC MMIO instance for TTC0 at address [TTC_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(TTC_0_BASE_ADDR) }
    }

    /// Create a new TTC MMIO instance for TTC1 at address [TTC_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(TTC_1_BASE_ADDR) }
    }
}
