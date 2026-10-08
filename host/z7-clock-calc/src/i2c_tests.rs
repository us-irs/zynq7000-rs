extern crate std;
use crate::i2c::*;
use fugit::RateExtU32;

#[test]
fn example_test() {
    let clk_cfg = calculate_divisors(111.MHz(), Speed::Normal100kHz).unwrap();
    assert_eq!(clk_cfg.div_a(), 0);
    assert_eq!(clk_cfg.div_b(), 55);
    let speed = calculate_i2c_speed(111.MHz(), clk_cfg);
    assert!(speed.to_raw() < 100_000);
    assert!(speed.to_raw() > 85_000);
}

#[test]
fn example_test_2() {
    let clk_cfg = calculate_divisors(111.MHz(), Speed::HighSpeed400KHz).unwrap();
    assert_eq!(clk_cfg.div_a(), 0);
    assert_eq!(clk_cfg.div_b(), 12);
    let speed = calculate_i2c_speed(111.MHz(), clk_cfg);
    assert!(speed.to_raw() < 400_000);
    assert!(speed.to_raw() > 360_000);
}

#[test]
fn example_test_3() {
    let clk_cfg = calculate_divisors(133.MHz(), Speed::Normal100kHz).unwrap();
    assert_eq!(clk_cfg.div_a(), 1);
    assert_eq!(clk_cfg.div_b(), 33);
    let speed = calculate_i2c_speed(133.MHz(), clk_cfg);
    assert!(speed.to_raw() < 100_000);
    assert!(speed.to_raw() > 85_000);
}

#[test]
fn example_test_4() {
    let clk_cfg = calculate_divisors(133.MHz(), Speed::HighSpeed400KHz).unwrap();
    assert_eq!(clk_cfg.div_a(), 0);
    assert_eq!(clk_cfg.div_b(), 15);
    let speed = calculate_i2c_speed(133.MHz(), clk_cfg);
    assert!(speed.to_raw() < 400_000);
    assert!(speed.to_raw() > 360_000);
}
