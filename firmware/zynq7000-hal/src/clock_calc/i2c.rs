//! I2C clock divisor search.
//!
//! The I2C clock is `cpu_1x_clk / (22 * (div_a + 1) * (div_b + 1))`.
use fugit::HertzU32 as Hertz;

/// I2C bus speed.
#[derive(Debug, Clone, Copy)]
pub enum Speed {
    /// 100 kHz.
    Normal100kHz,
    /// 400 kHz.
    HighSpeed400KHz,
}

impl Speed {
    /// Nominal bus frequency.
    pub fn frequency_full_number(&self) -> Hertz {
        Hertz::from_raw(match self {
            Speed::Normal100kHz => 100_000,
            Speed::HighSpeed400KHz => 400_000,
        })
    }

    /// From Xilinx embeddedsw
    /// If frequency 400KHz is selected, 384.6KHz should be set.
    /// If frequency 100KHz is selected, 90KHz should be set.
    /// This is due to a hardware limitation.
    pub fn frequency_for_calculation(&self) -> Hertz {
        Hertz::from_raw(match self {
            Speed::Normal100kHz => 90_000,
            Speed::HighSpeed400KHz => 384_600,
        })
    }
}

/// No divisor combination reaches the requested speed.
#[derive(Debug, thiserror::Error)]
#[error("I2C speed not attainable")]
pub struct SpeedNotAttainable;

/// I2C clock speed resulting from the given divisors.
#[inline]
pub fn calculate_i2c_speed(cpu_1x_clk: Hertz, clk_config: ClockConfig) -> Hertz {
    cpu_1x_clk / (22 * (clk_config.div_a as u32 + 1) * (clk_config.div_b as u32 + 1))
}

/// Search the divisors which come closest to the requested speed.
pub fn calculate_divisors(
    cpu_1x_clk: Hertz,
    speed: Speed,
) -> Result<ClockConfig, SpeedNotAttainable> {
    let target_speed = speed.frequency_for_calculation();
    if cpu_1x_clk > 22 * 64 * 4 * target_speed {
        return Err(SpeedNotAttainable);
    }
    let mut smallest_deviation = u32::MAX;
    let mut best_div_a = 1;
    let mut best_div_b = 1;
    for divisor_a in 1..=4 {
        for divisor_b in 1..=64 {
            let i2c_clock = cpu_1x_clk / (22 * divisor_a * divisor_b);
            let deviation =
                (target_speed.to_raw() as i32 - i2c_clock.to_raw() as i32).unsigned_abs();
            if deviation < smallest_deviation {
                smallest_deviation = deviation;
                best_div_a = divisor_a;
                best_div_b = divisor_b;
            }
        }
    }
    Ok(ClockConfig::new(best_div_a as u8 - 1, best_div_b as u8 - 1))
}

/// I2C clock divisors.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct ClockConfig {
    div_a: u8,
    div_b: u8,
}

impl ClockConfig {
    /// Create a configuration from the raw `div_a` and `div_b` register values.
    pub fn new(div_a: u8, div_b: u8) -> Self {
        Self { div_a, div_b }
    }

    /// Raw `div_a` register value.
    pub fn div_a(&self) -> u8 {
        self.div_a
    }

    /// Raw `div_b` register value.
    pub fn div_b(&self) -> u8 {
        self.div_b
    }
}
