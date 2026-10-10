extern crate std;

use crate::uart::*;
use fugit::HertzU32 as Hertz;

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
