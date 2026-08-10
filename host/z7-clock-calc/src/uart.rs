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

fn actual_baud(ref_clk: Hertz, cd: u16, bdiv: u8) -> f32 {
    ref_clk.to_raw() as f32 / (cd as f32 * (bdiv as f32 + 1.0))
}

fn error_fraction(ref_clk: Hertz, cd: u16, bdiv: u8, target_baud: u32) -> f32 {
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

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    const REF_UART_CLK: Hertz = Hertz::from_raw(50_000_000);
    const REF_UART_CLK_DIV_8: Hertz = Hertz::from_raw(6_250_000);

    #[test]
    fn test_best_calc_0() {
        // Baud 600.
        let result = best_baud_divisors(REF_UART_CLK, 600);
        assert_eq!(result.cd, 499);
        assert_eq!(result.bdiv, 166);
    }

    #[test]
    fn test_best_calc_matches_divided_clock() {
        // Baud 9600 off an already-divided (by 8) reference clock, e.g. as used for
        // ClockSelect::UartRefClkDiv8. The search must do at least as well as the
        // (cd=81, bdiv=7) reference config used elsewhere for the same target (which
        // achieves a 0.47 % error) - here it actually finds a much closer match.
        let result = best_baud_divisors(REF_UART_CLK_DIV_8, 9600);
        assert_eq!(result.cd, 93);
        assert_eq!(result.bdiv, 6);
        let reference_error = error_fraction(REF_UART_CLK_DIV_8, 81, 7, 9600);
        assert!(result.error <= reference_error);
    }

    #[test]
    fn error_is_the_fraction_actually_achieved() {
        let result = best_baud_divisors(REF_UART_CLK, 9600);
        let achieved = actual_baud(REF_UART_CLK, result.cd, result.bdiv);
        let expected_error = (achieved - 9600.0).abs() / 9600.0;
        assert!((result.error - expected_error).abs() < 1e-6);
        // Sanity check against the doc-promised scale: a well-supported baud rate should
        // be within a fraction of a percent, not multiple "percent" as a bare number.
        assert!(result.error < 0.01);
    }

    #[test]
    fn never_returns_a_zero_divisor() {
        // Target baud far above what any bdiv >= 4 can reach without cd rounding to 0;
        // the search must fall back to its (cd = 1) sentinel rather than ever returning
        // cd = 0, which would be rejected downstream (divisor-zero is invalid hardware
        // config).
        let result = best_baud_divisors(REF_UART_CLK, 50_000_000);
        assert!(result.cd >= 1);
    }

    #[test]
    #[cfg(feature = "alloc")]
    fn test_viable_config_calculation() {
        let cfgs = viable_baud_divisors(REF_UART_CLK, 115200);
        assert!(cfgs.iter().any(|d| d.cd == 62 && d.bdiv == 6));
    }

    #[test]
    #[cfg(feature = "alloc")]
    fn viable_configs_all_within_threshold() {
        let cfgs = viable_baud_divisors(REF_UART_CLK, 115200);
        assert!(!cfgs.is_empty());
        for cfg in &cfgs {
            assert!(cfg.error < MAX_BAUDERROR_RATE);
        }
    }

    #[test]
    #[cfg(feature = "alloc")]
    fn viable_configs_include_the_single_best_match() {
        let best = best_baud_divisors(REF_UART_CLK, 115200);
        let viable = viable_baud_divisors(REF_UART_CLK, 115200);
        assert!(
            viable
                .iter()
                .any(|d| d.cd == best.cd && d.bdiv == best.bdiv)
        );
    }

    #[test]
    #[cfg(feature = "alloc")]
    fn viable_configs_empty_for_unreachable_target() {
        // Same unreachable target as `never_returns_a_zero_divisor`: no candidate should
        // clear the error threshold, so the viable list must be empty rather than
        // containing spurious zero-divisor entries.
        let cfgs = viable_baud_divisors(REF_UART_CLK, 50_000_000);
        assert!(cfgs.is_empty());
    }
}
