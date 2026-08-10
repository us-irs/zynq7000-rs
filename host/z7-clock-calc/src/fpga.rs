//! Two-stage integer clock divisor search.
//!
//! This is the scheme used by the Zynq-7000 FPGA output clocks (and other peripherals
//! with the same `divisor_0 * divisor_1` two-stage divider), where the derived frequency
//! is `ref_in / (divisor_0 * divisor_1)`.
use fugit::HertzU32 as Hertz;

/// A `(divisor_0, divisor_1)` pair found by [`find_two_stage_divisors`].
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DivisorPair {
    /// First-stage divisor.
    pub div0: u32,
    /// Second-stage divisor.
    pub div1: u32,
}

/// Finds the `(divisor_0, divisor_1)` pair in `1..=max_divisor` which derives a frequency
/// closest to `target` from `ref_in`, i.e. minimizes `abs(ref_in / (div0 * div1) - target)`.
///
/// Both divisors are searched over their full range. For a fixed `div0`, the best `div1` is
/// one of the two integers neighboring the exact (real-valued) ratio
/// `ref_in / (target * div0)`, since `1/div1` is monotonic and the true optimum can only be
/// one of those two - so the inner search does not need to sweep the full `div1` range.
/// Ties are broken by the lower `div0`.
pub fn find_two_stage_divisors(ref_in: Hertz, target: Hertz, max_divisor: u32) -> DivisorPair {
    struct Best {
        div0: u32,
        div1: u32,
        diff: u64,
    }

    impl Best {
        const NONE: Self = Self {
            div0: 1,
            div1: 1,
            diff: u64::MAX,
        };
    }

    let ref_raw = ref_in.to_raw() as u64;
    let target = target.to_raw() as u64;
    let mut best = Best::NONE;
    'search: for div0 in 1..=max_divisor {
        let ideal_div1 = (ref_raw / (target * div0 as u64)).clamp(1, max_divisor as u64) as u32;
        for div1 in [ideal_div1, (ideal_div1 + 1).min(max_divisor)] {
            let calc = ref_raw / (div0 as u64 * div1 as u64);
            let diff = calc.abs_diff(target);
            if diff < best.diff {
                best = Best { div0, div1, diff };
                if diff == 0 {
                    break 'search;
                }
            }
        }
    }
    DivisorPair {
        div0: best.div0,
        div1: best.div1,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    const MHZ: u32 = 1_000_000;
    const KHZ: u32 = 1_000;

    fn resulting_freq(ref_in: Hertz, pair: DivisorPair) -> Hertz {
        Hertz::from_raw(ref_in.to_raw() / (pair.div0 * pair.div1))
    }

    #[test]
    fn exact_match_small_divisor() {
        // 1000 MHz / 10 = 100 MHz exactly.
        let ref_in = Hertz::from_raw(1000 * MHZ);
        let pair = find_two_stage_divisors(ref_in, Hertz::from_raw(100 * MHZ), 63);
        assert_eq!(resulting_freq(ref_in, pair), Hertz::from_raw(100 * MHZ));
    }

    #[test]
    fn exact_match_needs_both_divisors() {
        // 1000 MHz / 1000 = 1 MHz exactly, but 1000 does not fit into a single
        // 6-bit (1..=63) divisor, so both stages are required (e.g. 25 * 40).
        let ref_in = Hertz::from_raw(1000 * MHZ);
        let pair = find_two_stage_divisors(ref_in, Hertz::from_raw(MHZ), 63);
        assert!(pair.div0 >= 1 && pair.div1 >= 1);
        assert_eq!(resulting_freq(ref_in, pair), Hertz::from_raw(MHZ));
    }

    #[test]
    fn divisors_stay_in_range() {
        // Largest attainable divisor product is 63 * 63 = 3969, so 1 kHz out of
        // 1000 MHz (needs a divisor of 1_000_000) is unreachable; just make sure
        // the search still returns values within the valid 1..=63 hardware range.
        let ref_in = Hertz::from_raw(1000 * MHZ);
        let pair = find_two_stage_divisors(ref_in, Hertz::from_raw(KHZ), 63);
        assert!((1..=63).contains(&pair.div0));
        assert!((1..=63).contains(&pair.div1));
    }

    #[test]
    fn closest_match_when_not_exact() {
        // 666 MHz cannot be evenly divided down to 100 MHz. The two integer
        // divisors bracketing the ideal ratio (6.66) are 6 -> 111 MHz and
        // 7 -> ~95.14 MHz; 7 is closer to the 100 MHz target.
        let ref_in = Hertz::from_raw(666 * MHZ);
        let target = Hertz::from_raw(100 * MHZ);
        let pair = find_two_stage_divisors(ref_in, target, 63);
        let achieved = resulting_freq(ref_in, pair);
        assert_eq!(achieved, Hertz::from_raw(666_000_000 / 7));

        // No other single-digit divisor gets closer to the target.
        for div in 1..=20u32 {
            let candidate = ref_in.to_raw() / div;
            let candidate_diff = candidate.abs_diff(target.to_raw());
            assert!(candidate_diff >= achieved.to_raw().abs_diff(target.to_raw()));
        }
    }

    #[test]
    fn low_frequency_target_from_high_frequency_pll() {
        // Requires close to the maximum combined divisor (1000 MHz / 3969 ~= 251.9 kHz).
        let ref_in = Hertz::from_raw(1000 * MHZ);
        let target = Hertz::from_raw(252 * KHZ);
        let pair = find_two_stage_divisors(ref_in, target, 63);
        assert!((1..=63).contains(&pair.div0));
        assert!((1..=63).contains(&pair.div1));
        let achieved = resulting_freq(ref_in, pair);
        // Should land close to the target given the achievable resolution at this range.
        assert!(achieved.to_raw().abs_diff(target.to_raw()) < 2_000);
    }
}
