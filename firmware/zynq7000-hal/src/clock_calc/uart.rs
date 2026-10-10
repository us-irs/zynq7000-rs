//! UART baud rate divisor search.
//!
//! The Zynq-7000 UART baud rate generator derives the baud rate from a reference clock
//! using two cascaded divisors: `baud = ref_clk / (cd * (bdiv + 1))`.
use fugit::HertzU32 as Hertz;

/// Maximum acceptable baud rate error, as a fraction (0.5 %).
pub const MAX_BAUDERROR_RATE: f32 = 0.005;

/// Result of a UART baud-rate divisor search.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct BaudDivisors {
    /// Baud rate generator (clock divisor) value.
    pub cd: u16,
    /// Baud rate divider value.
    pub bdiv: u8,
    /// Resulting baud rate error as a fraction, e.g. `0.02` means a 2 % deviation from the
    /// requested target baud rate.
    pub error: f32,
}

pub(crate) fn actual_baud(ref_clk: Hertz, cd: u16, bdiv: u8) -> f32 {
    ref_clk.to_raw() as f32 / (cd as f32 * (bdiv as f32 + 1.0))
}

pub(crate) fn error_fraction(ref_clk: Hertz, cd: u16, bdiv: u8, target_baud: u32) -> f32 {
    (actual_baud(ref_clk, cd, bdiv) - target_baud as f32).abs() / target_baud as f32
}

/// Finds the `(cd, bdiv)` pair which gets closest to `target_baud` from `ref_clk`.
///
/// `ref_clk` must already reflect the selected reference clock, i.e. the caller is
/// responsible for e.g. dividing by 8 for a `UartRefClkDiv8`-equivalent clock selection
/// before calling this function.
pub fn best_baud_divisors(ref_clk: Hertz, target_baud: u32) -> BaudDivisors {
    let mut best = BaudDivisors {
        cd: 1,
        bdiv: 0,
        error: f32::MAX,
    };
    for bdiv in 4..u8::MAX {
        let cd =
            libm::roundf(ref_clk.to_raw() as f32 / ((bdiv as u32 + 1) as f32 * target_baud as f32))
                as u64;
        if cd == 0 || cd > u16::MAX as u64 {
            continue;
        }
        let error = error_fraction(ref_clk, cd as u16, bdiv, target_baud);
        if error < best.error {
            best = BaudDivisors {
                cd: cd as u16,
                bdiv,
                error,
            };
        }
    }
    best
}

/// Finds all `(cd, bdiv)` pairs within [`MAX_BAUDERROR_RATE`] of `target_baud`.
///
/// See [`best_baud_divisors`] for the meaning of `ref_clk`.
#[cfg(feature = "alloc")]
pub fn viable_baud_divisors(ref_clk: Hertz, target_baud: u32) -> alloc::vec::Vec<BaudDivisors> {
    let mut viable = alloc::vec::Vec::new();
    for bdiv in 4..u8::MAX {
        let cd =
            libm::roundf(ref_clk.to_raw() as f32 / ((bdiv as u32 + 1) as f32 * target_baud as f32))
                as u64;
        if cd == 0 || cd > u16::MAX as u64 {
            continue;
        }
        let error = error_fraction(ref_clk, cd as u16, bdiv, target_baud);
        if error < MAX_BAUDERROR_RATE {
            viable.push(BaudDivisors {
                cd: cd as u16,
                bdiv,
                error,
            });
        }
    }
    viable
}
