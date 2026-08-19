//! # Bare-metal SMP hello-world for the Zedboard
//!
//! Boots CPU1 and checks that atomics and `critical_section` work correctly across both
//! physically-running cores.
#![no_std]
#![no_main]

// Import for shared panic handler and exception handlers.
use zedboard_smp as _;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use critical_section::Mutex;
use log::{error, info, warn};
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    gpio::{GpioPins, Output, PinState},
    priv_tim::CpuPrivateTimer,
    uart::{ClockConfig, Config, Uart},
};

use zynq7000_rt as _;

use zedboard_smp::{Handoff, PS_CLOCK_FREQUENCY};

const INIT_STRING: &str = "-- Zynq 7000 Zedboard bare-metal SMP example --\n\r";

// --- CPU1 boot handshake ---

/// Set by CPU1 once it has booted. A plain atomic handshake flag works here without cache
/// maintenance because DDR is mapped shareable and both cores run with ACTLR's SMP bit set, so
/// the SCU keeps L1 caches coherent between them.
static CORE1_BOOTED: AtomicBool = AtomicBool::new(false);
/// How many times CPU0 polls for CPU1 to boot before giving up.
const CORE1_BOOT_ATTEMPTS: u32 = 200;
/// Delay between boot-poll attempts.
const CORE1_BOOT_ATTEMPT_DELAY_MS: u32 = 5;
/// How long CPU0 waits for CPU1 to finish its loops before checking totals.
const CORE1_FINISH_WAIT_MS: u32 = 500;

// --- Shared-state workload, run by both cores via `run_shared_workload` ---

/// Incremented by both cores using an atomic read-modify-write. Safe without cache maintenance
/// for the same reason as `CORE1_BOOTED`: DDR is shareable and SCU-coherent between the cores.
static SHARED_VARIABLE_USING_ATOMIC: AtomicU32 = AtomicU32::new(0);
/// Incremented by both cores from inside a critical section. Same coherency reasoning as above.
static SHARED_VARIABLE_USING_CS_LOCK: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
/// How many atomic fetch-add loops each core runs.
const CAS_LOOPS: u32 = 1000;
/// How many critical-section loops each core runs.
const CS_MUTEX_LOOPS: u32 = 1000;

// --- CPU1's LEDs, handed off from CPU0 ---

/// CPU1's status LEDs (EMIO pins 0-7), constructed on CPU0 and handed off to CPU1 through this
/// static before CPU1 is released via `start_core1`. Needs `Output`/`LowLevelGpio` to be `Send`
/// to move across cores like this.
static EMIO_LEDS: Handoff<[Output; 8]> = Handoff::new();

/// Runs the atomic fetch-add and critical-section increment loops that both cores perform on
/// the shared statics above, to check that both mechanisms work correctly across cores.
fn run_shared_workload() {
    for _ in 0..CAS_LOOPS {
        SHARED_VARIABLE_USING_ATOMIC.fetch_add(1, Ordering::Relaxed);
    }
    for _ in 0..CS_MUTEX_LOOPS {
        critical_section::with(|cs| {
            let mut value_ref = SHARED_VARIABLE_USING_CS_LOCK.borrow_ref_mut(cs);
            *value_ref += 1;
        })
    }
}

/// Logs whether `actual == expected` under `name`.
fn report_result(name: &str, actual: u32, expected: u32) {
    if actual == expected {
        info!("{name} test passed");
    } else {
        warn!("{name} test failed: got {actual}, expected {expected}");
    }
}

/// Entry point on CPU0, called by `zynq7000-rt`'s start-up code.
#[zynq7000_rt::entry]
fn main() -> ! {
    let periphs = zynq7000_hal::init(zynq7000_hal::Config::default()).unwrap();

    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();

    let mut gpio_pins = GpioPins::new(periphs.gpio);

    // CPU0's own status LED.
    let mut mio_led = Output::new_for_mio(gpio_pins.mio.mio7, PinState::Low);

    // Construct CPU1's status LEDs here and hand them off through the shared `EMIO_LEDS`
    // static.
    let emio_leds: [Output; 8] = core::array::from_fn(|i| {
        Output::new_for_emio(gpio_pins.emio.take(i).unwrap(), PinState::Low)
    });
    EMIO_LEDS.put(emio_leds);

    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut log_uart = Uart::new_with_mio_for_uart_1(
        periphs.uart_1,
        Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    log_uart.write_all(INIT_STRING.as_bytes());
    zynq7000_hal::log::uart_blocking::init_with_busy_flag(log_uart, log::LevelFilter::Trace, false);

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mut priv_tim = CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();

    // Everything above touches SLCR (level shifters), and we want to keep that in core0.
    zynq7000_rt::smp::start_core1();

    let mut booted = false;
    for _ in 0..CORE1_BOOT_ATTEMPTS {
        if CORE1_BOOTED.load(Ordering::SeqCst) {
            booted = true;
            break;
        }
        priv_tim.delay_ms(CORE1_BOOT_ATTEMPT_DELAY_MS);
    }
    if !booted {
        error!("CPU1 did not boot in time");
    }

    run_shared_workload();

    // Give CPU1 a chance to finish its own loops.
    priv_tim.delay_ms(CORE1_FINISH_WAIT_MS);

    let total_a = SHARED_VARIABLE_USING_ATOMIC.load(Ordering::Relaxed);
    report_result("atomic fetch-add", total_a, CAS_LOOPS * 2);

    let total_b = critical_section::with(|cs| *SHARED_VARIABLE_USING_CS_LOCK.borrow_ref(cs));
    report_result("critical_section", total_b, CS_MUTEX_LOOPS * 2);

    // Blink our own LED forever, using the private timer already taken above.
    loop {
        mio_led.toggle();
        priv_tim.delay_ms(500);
    }
}

/// Entry point on CPU1, called by `zynq7000-rt`'s start-up code once CPU0 has released it via
/// [`zynq7000_rt::smp::start_core1`].
#[unsafe(no_mangle)]
pub extern "C" fn kmain_secondary() {
    // No interrupts are used on CPU1 here (only blocking, polling-based delays), so the GIC is
    // left alone.
    zynq7000_hal::init_secondary_core(zynq7000_hal::SecondaryCoreConfig {
        init_gic: false,
        ..Default::default()
    })
    .unwrap();

    CORE1_BOOTED.store(true, Ordering::SeqCst);

    run_shared_workload();

    // Clock config is read-only (already set up by CPU0/the boot flow), safe to re-read here.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    let mut priv_tim = CpuPrivateTimer::take(clocks.arm_clocks()).unwrap();

    let mut emio_leds: [Output; 8] = EMIO_LEDS.take();

    let mut led_idx = 0;
    loop {
        emio_leds[led_idx].toggle();
        led_idx = (led_idx + 1) % emio_leds.len();
        priv_tim.delay_ms(200);
    }
}
