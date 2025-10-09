//! # Triple-timer counter (TTC) high-level driver
//!
//! This module also contains support for PWM and output waveform generation.
//!
//! ## Examples
//!
//! - [PWM](https://egit.irs.uni-stuttgart.de/rust/zynq7000-rs/src/branch/main/zynq/examples/embassy/src/bin/pwm.rs)

use core::convert::Infallible;

use arbitrary_int::{prelude::*, u3, u4};
use zynq7000::ttc::{MmioTtc, TTC_0_BASE_ADDR, TTC_1_BASE_ADDR};

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{Mio16, Mio17, Mio18, Mio19, Mio40, Mio41, Mio42, Mio43};
use crate::{
    clocks::ArmClocks,
    gpio::{
        IoPeriphPin,
        mio::{Mio28, Mio29, Mio30, Mio31, MioPin, MuxConfig, Pin},
    },
    time::Hertz,
};

/// Each TTC consists of three independent timers/counters.
#[derive(Debug, Copy, Clone)]
pub enum TtcId {
    Ttc0 = 0,
    Ttc1 = 1,
}

#[derive(Debug, Copy, Clone)]
pub enum ChannelId {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
}

pub trait PsTtc {
    fn reg_block(&self) -> MmioTtc<'static>;
    fn id(&self) -> Option<TtcId>;
}

impl PsTtc for MmioTtc<'static> {
    #[inline]
    fn reg_block(&self) -> MmioTtc<'static> {
        unsafe { self.clone() }
    }

    #[inline]
    fn id(&self) -> Option<TtcId> {
        let base_addr = unsafe { self.ptr() } as usize;
        if base_addr == TTC_0_BASE_ADDR {
            return Some(TtcId::Ttc0);
        } else if base_addr == TTC_1_BASE_ADDR {
            return Some(TtcId::Ttc1);
        }
        None
    }
}

pub const TTC_MUX_CONF: MuxConfig = MuxConfig::new_with_l3(u3::new(0b110));

pub trait ClockInPin: MioPin {
    const ID: TtcId;
}

pub trait WaveOutPin: MioPin {
    const ID: TtcId;
}

// TTC0 pin trait implementations.

impl ClockInPin for Pin<Mio19> {
    const ID: TtcId = TtcId::Ttc0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl ClockInPin for Pin<Mio31> {
    const ID: TtcId = TtcId::Ttc0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl ClockInPin for Pin<Mio43> {
    const ID: TtcId = TtcId::Ttc0;
}

impl WaveOutPin for Pin<Mio18> {
    const ID: TtcId = TtcId::Ttc0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl WaveOutPin for Pin<Mio30> {
    const ID: TtcId = TtcId::Ttc0;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl WaveOutPin for Pin<Mio42> {
    const ID: TtcId = TtcId::Ttc0;
}

// TTC1 pin trait implementations.

impl ClockInPin for Pin<Mio17> {
    const ID: TtcId = TtcId::Ttc1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl ClockInPin for Pin<Mio29> {
    const ID: TtcId = TtcId::Ttc1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl ClockInPin for Pin<Mio41> {
    const ID: TtcId = TtcId::Ttc1;
}

impl WaveOutPin for Pin<Mio16> {
    const ID: TtcId = TtcId::Ttc1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl WaveOutPin for Pin<Mio28> {
    const ID: TtcId = TtcId::Ttc1;
}
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl WaveOutPin for Pin<Mio40> {
    const ID: TtcId = TtcId::Ttc1;
}

pub struct Ttc {
    pub ch0: TtcChannel,
    pub ch1: TtcChannel,
    pub ch2: TtcChannel,
}

impl Ttc {
    /// Create a new TTC instance. The passed TTC peripheral instance MUST point to a valid
    /// processing system TTC peripheral.
    ///
    /// Returns [None] if the passed peripheral block does not have a valid PS TTC address.
    pub fn new(ps_ttc: impl PsTtc) -> Option<Self> {
        ps_ttc.id()?;
        let regs = ps_ttc.reg_block();
        let ch0 = TtcChannel {
            regs: unsafe { regs.clone() },
            id: ChannelId::Ch0,
        };
        let ch1 = TtcChannel {
            regs: unsafe { regs.clone() },
            id: ChannelId::Ch1,
        };
        let ch2 = TtcChannel {
            regs,
            id: ChannelId::Ch2,
        };
        Some(Self { ch0, ch1, ch2 })
    }
}

pub struct TtcChannel {
    regs: MmioTtc<'static>,
    id: ChannelId,
}

impl TtcChannel {
    pub fn regs_mut(&mut self) -> &mut MmioTtc<'static> {
        &mut self.regs
    }

    #[inline]
    pub fn read_counter(&self) -> u16 {
        self.regs
            .read_current_counter(self.id as usize)
            .unwrap()
            .count()
    }

    pub fn id(&self) -> ChannelId {
        self.id
    }
}

#[derive(Debug, thiserror::Error)]
#[error("invalid TTC pin configuration")]
pub struct InvalidTtcPinConfigError(pub MuxConfig);

#[derive(Debug, thiserror::Error)]
#[error("frequency is zero")]
pub struct FrequencyIsZeroError;

#[derive(Debug, thiserror::Error)]
pub enum TtcConstructionError {
    #[error("invalid TTC pin configuration")]
    InvalidTtcPinConfig(#[from] InvalidTtcPinConfigError),
    #[error("frequency is zero")]
    FrequencyIsZero(#[from] FrequencyIsZeroError),
}

pub fn calc_prescaler_reg_and_interval_ticks(mut ref_clk: Hertz, freq: Hertz) -> (Option<u4>, u16) {
    // TODO: Can this be optimized?
    let mut prescaler: Option<u32> = None;
    let mut tick_val = ref_clk / freq;
    while tick_val > u16::MAX as u32 {
        ref_clk /= 2;
        match prescaler {
            Some(val) => {
                if val == u4::MAX.as_u32() {
                    break;
                }
                prescaler = Some(val + 1);
            }
            None => prescaler = Some(0),
        }
        tick_val = ref_clk / freq;
    }
    (prescaler.map(|v| u4::new(v as u8)), tick_val as u16)
}

pub struct Pwm {
    channel: TtcChannel,
    ref_clk: Hertz,
}

impl Pwm {
    /// Create a new PWM instance which uses the CPU 1x clock as the clock source and also uses
    /// a MIO output pin for the waveform output.
    pub fn new_with_cpu_clk_and_mio_waveout(
        channel: TtcChannel,
        arm_clocks: &ArmClocks,
        freq: Hertz,
        wave_out: impl WaveOutPin,
    ) -> Result<Self, TtcConstructionError> {
        IoPeriphPin::new(wave_out, TTC_MUX_CONF, None);
        Ok(Self::new_with_cpu_clk(channel, arm_clocks, freq)?)
    }

    /// Create a new PWM instance which uses the CPU 1x clock as the clock source.
    pub fn new_with_cpu_clk(
        channel: TtcChannel,
        arm_clocks: &ArmClocks,
        freq: Hertz,
    ) -> Result<Self, FrequencyIsZeroError> {
        Self::new_generic(channel, arm_clocks.cpu_1x_clk(), freq)
    }

    /// Create a new PWM instance based on a reference clock source.
    pub fn new_generic(
        channel: TtcChannel,
        ref_clk: Hertz,
        freq: Hertz,
    ) -> Result<Self, FrequencyIsZeroError> {
        if freq.raw() == 0 {
            return Err(FrequencyIsZeroError);
        }
        let (prescaler_reg, tick_val) = calc_prescaler_reg_and_interval_ticks(ref_clk, freq);
        let id = channel.id() as usize;
        let mut pwm = Self { channel, ref_clk };
        pwm.set_up_and_configure_pwm(id, prescaler_reg, tick_val);
        Ok(pwm)
    }

    /// Set a new frequency for the PWM cycle.
    ///
    /// This resets the duty cycle to 0%.
    pub fn set_frequency(&mut self, freq: Hertz) -> Result<(), FrequencyIsZeroError> {
        if freq.raw() == 0 {
            return Err(FrequencyIsZeroError);
        }
        let id = self.channel.id() as usize;
        let (prescaler_reg, tick_val) = calc_prescaler_reg_and_interval_ticks(self.ref_clk, freq);
        self.set_up_and_configure_pwm(id, prescaler_reg, tick_val);
        Ok(())
    }

    #[inline]
    pub fn ttc_channel_mut(&mut self) -> &mut TtcChannel {
        &mut self.channel
    }

    #[inline]
    pub fn max_duty_cycle(&self) -> u16 {
        self.channel
            .regs
            .read_interval_value(self.channel.id() as usize)
            .unwrap()
            .value()
    }

    #[inline]
    pub fn set_duty_cycle(&mut self, duty: u16) {
        let id = self.channel.id() as usize;
        self.channel
            .regs
            .modify_cnt_ctrl(id, |mut val| {
                val.set_disable(true);
                val
            })
            .unwrap();
        self.channel
            .regs
            .write_match_value_0(
                self.channel.id() as usize,
                zynq7000::ttc::RwValue::new_with_raw_value(duty as u32),
            )
            .unwrap();
        self.channel
            .regs
            .modify_cnt_ctrl(id, |mut val| {
                val.set_disable(false);
                val.set_reset(true);
                val
            })
            .unwrap();
    }

    fn set_up_and_configure_pwm(&mut self, id: usize, prescaler_reg: Option<u4>, tick_val: u16) {
        // Disable the counter first.
        self.channel
            .regs
            .write_cnt_ctrl(id, zynq7000::ttc::CounterControl::new_with_raw_value(1))
            .unwrap();

        // Clock configuration
        self.channel
            .regs
            .write_clk_cntr(
                id,
                zynq7000::ttc::ClockControl::builder()
                    .with_ext_clk_edge(false)
                    .with_clk_src(zynq7000::ttc::ClockSource::Pclk)
                    .with_prescaler(prescaler_reg.unwrap_or(u4::new(0)))
                    .with_prescale_enable(prescaler_reg.is_some())
                    .build(),
            )
            .unwrap();
        self.channel
            .regs
            .write_interval_value(
                id,
                zynq7000::ttc::RwValue::new_with_raw_value(tick_val as u32),
            )
            .unwrap();
        // Corresponds to duty cycle 0.
        self.channel
            .regs
            .write_match_value_0(id, zynq7000::ttc::RwValue::new_with_raw_value(0))
            .unwrap();
        self.channel
            .regs
            .write_cnt_ctrl(
                id,
                zynq7000::ttc::CounterControl::builder()
                    .with_wave_polarity(zynq7000::ttc::WavePolarity::LowToHighOnMatch1)
                    .with_wave_enable_n(zynq7000::ttc::WaveEnable::Enable)
                    .with_reset(true)
                    .with_match_enable(true)
                    .with_decrementing(false)
                    .with_mode(zynq7000::ttc::Mode::Interval)
                    .with_disable(false)
                    .build(),
            )
            .unwrap();
    }
}

impl embedded_hal::pwm::ErrorType for Pwm {
    type Error = Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for Pwm {
    #[inline]
    fn max_duty_cycle(&self) -> u16 {
        self.max_duty_cycle()
    }

    #[inline]
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty_cycle(duty);
        Ok(())
    }
}
