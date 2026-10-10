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
