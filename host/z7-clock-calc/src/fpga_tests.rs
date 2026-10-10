extern crate std;

use crate::fpga::*;
use fugit::HertzU32 as Hertz;

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
