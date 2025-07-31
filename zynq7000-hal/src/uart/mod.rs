//! # UART module.
//!
//! Support for the processing system UARTs.
use core::convert::Infallible;

use arbitrary_int::u3;
use libm::round;
use zynq7000::{
    slcr::reset::DualRefAndClkRst,
    uart::{
        BaudRateDiv, Baudgen, ChMode, ClkSel, FifoTrigger, InterruptControl, MmioUart, Mode,
        UART_0_BASE, UART_1_BASE,
    },
};

use crate::{
    enable_amba_periph_clk,
    gpio::{
        IoPeriphPin,
        mio::{
            Mio8, Mio9, Mio10, Mio11, Mio12, Mio13, Mio14, Mio15, Mio28, Mio29, Mio30, Mio31,
            Mio32, Mio33, Mio34, Mio35, Mio36, Mio37, Mio38, Mio39, Mio48, Mio49, Mio52, Mio53,
            MioPinMarker, MuxCfg, Pin,
        },
    },
    slcr::Slcr,
};

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27, Mio40,
    Mio41, Mio42, Mio43, Mio44, Mio45, Mio46, Mio47, Mio50, Mio51,
};

use super::{clocks::IoClocks, time::Hertz};

pub mod tx;
pub use tx::*;

pub mod tx_async;
pub use tx_async::*;

pub mod rx;
pub use rx::*;

pub const FIFO_DEPTH: usize = 64;
pub const DEFAULT_RX_TRIGGER_LEVEL: u8 = 32;
pub const UART_MUX_CONF: MuxCfg = MuxCfg::new_with_l3(u3::new(0b111));

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum UartId {
    Uart0 = 0,
    Uart1 = 1,
}

pub trait PsUart {
    fn reg_block(&self) -> MmioUart<'static>;
    fn uart_id(&self) -> Option<UartId>;
}

impl PsUart for MmioUart<'static> {
    #[inline]
    fn reg_block(&self) -> MmioUart<'static> {
        unsafe { self.clone() }
    }

    fn uart_id(&self) -> Option<UartId> {
        let base_addr = unsafe { self.ptr() } as usize;
        if base_addr == UART_0_BASE {
            return Some(UartId::Uart0);
        } else if base_addr == UART_1_BASE {
            return Some(UartId::Uart1);
        }
        None
    }
}

impl UartId {
    /// Unsafely steal a peripheral MMIO block for the given UART.
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees by the HAL.
    pub const unsafe fn regs(&self) -> MmioUart<'static> {
        match self {
            UartId::Uart0 => unsafe { zynq7000::uart::Uart::new_mmio_fixed_0() },
            UartId::Uart1 => unsafe { zynq7000::uart::Uart::new_mmio_fixed_1() },
        }
    }
}

pub trait RxPin: MioPinMarker {
    const UART_IDX: UartId;
}
pub trait TxPin: MioPinMarker {
    const UART_IDX: UartId;
}

pub trait UartPins {}

#[derive(Debug, thiserror::Error)]
#[error("divisor is zero")]
pub struct DivisorZero;

macro_rules! pin_pairs {
    ($UartPeriph:path, ($( [$(#[$meta:meta], )? $TxMio:ident, $RxMio:ident] ),+ $(,)? )) => {
        $(
            $( #[$meta] )?
            impl TxPin for Pin<$TxMio> {
                const UART_IDX: UartId = $UartPeriph;
            }

            $( #[$meta] )?
            impl RxPin for Pin<$RxMio> {
                const UART_IDX: UartId = $UartPeriph;
            }

            impl UartPins for (Pin<$TxMio>, Pin<$RxMio>) {}
        )+
    };
}

/*
macro_rules! impl_into_uart {
    (($($Mio:ident),+)) => {
        $(
            impl From<Pin<$Mio>> for IoPeriphPin {
                fn from(pin: Pin<$Mio>) -> Self {
                    IoPeriphPin::new(pin, UART_MUX_CONF, None)
                }
            }
        )+
    };
}

impl_into_uart!((
    Mio10, Mio11, Mio15, Mio14, Mio31, Mio30, Mio35, Mio34, Mio39, Mio38, Mio8, Mio9, Mio12, Mio13,
    Mio28, Mio29, Mio32, Mio33, Mio36, Mio37, Mio48, Mio49, Mio52, Mio53
));

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_into_uart!((
    Mio19, Mio18, Mio23, Mio22, Mio43, Mio42, Mio47, Mio46, Mio51, Mio50, Mio16, Mio17, Mio20,
    Mio21, Mio24, Mio25, Mio40, Mio41, Mio44, Mio45
));
*/

pin_pairs!(
    UartId::Uart0,
    (
        [Mio11, Mio10],
        [Mio15, Mio14],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio19, Mio18],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio23, Mio22],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio27, Mio26],
        [Mio31, Mio30],
        [Mio35, Mio34],
        [Mio39, Mio38],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio43, Mio42],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio47, Mio46],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio51, Mio50],
    )
);

pin_pairs!(
    UartId::Uart1,
    (
        [Mio8, Mio9],
        [Mio12, Mio13],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio16, Mio17],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio20, Mio21],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio24, Mio25],
        [Mio28, Mio29],
        [Mio32, Mio33],
        [Mio36, Mio37],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio40, Mio41],
        [#[cfg(not(feature ="7z010-7z007s-clg225"))], Mio44, Mio45],
        [Mio48, Mio49],
        [Mio52, Mio53],
    )
);

/// Based on values provided by the vendor library.
pub const MAX_BAUD_RATE: u32 = 6240000;
/// Based on values provided by the vendor library.
pub const MIN_BAUD_RATE: u32 = 110;

#[derive(Debug, Default, Clone, Copy)]
pub enum Parity {
    Even,
    Odd,
    #[default]
    None,
}

#[derive(Debug, Default, Clone, Copy)]
pub enum Stopbits {
    #[default]
    One,
    OnePointFive,
    Two,
}

#[derive(Debug, Default, Clone, Copy)]
pub enum CharLen {
    SixBits,
    SevenBits,
    #[default]
    EightBits,
}

#[derive(Debug, Clone, Copy)]
pub struct ClkConfigRaw {
    cd: u16,
    bdiv: u8,
}

#[cfg(feature = "alloc")]
pub fn calculate_viable_configs(
    mut uart_clk: Hertz,
    clk_sel: ClkSel,
    target_baud: u32,
) -> alloc::vec::Vec<(ClkConfigRaw, f64)> {
    let mut viable_cfgs = alloc::vec::Vec::new();
    if clk_sel == ClkSel::UartRefClkDiv8 {
        uart_clk /= 8;
    }
    let mut current_clk_config = ClkConfigRaw::new(0, 0);
    for bdiv in 4..u8::MAX {
        let cd =
            round(uart_clk.raw() as f64 / ((bdiv as u32 + 1) as f64 * target_baud as f64)) as u64;
        if cd > u16::MAX as u64 {
            continue;
        }
        current_clk_config.cd = cd as u16;
        current_clk_config.bdiv = bdiv;
        let baud = current_clk_config.actual_baud(uart_clk);
        let error = ((baud - target_baud as f64).abs() / target_baud as f64) * 100.0;
        if error < MAX_BAUDERROR_RATE as f64 {
            viable_cfgs.push((current_clk_config, error));
        }
    }

    viable_cfgs
}

/// Calculate the clock configuration for the smallest error to reach the desired target
/// baud rate.
///
/// You can also use [calculate_viable_configs] to get a list of all viable configurations.
pub fn calculate_raw_baud_cfg_smallest_error(
    mut uart_clk: Hertz,
    clk_sel: ClkSel,
    target_baud: u32,
) -> Result<(ClkConfigRaw, f64), DivisorZero> {
    if target_baud == 0 {
        return Err(DivisorZero);
    }
    if clk_sel == ClkSel::UartRefClkDiv8 {
        uart_clk /= 8;
    }
    let mut current_clk_config = ClkConfigRaw::default();
    let mut best_clk_config = ClkConfigRaw::default();
    let mut smallest_error: f64 = 100.0;
    for bdiv in 4..u8::MAX {
        let cd =
            round(uart_clk.raw() as f64 / ((bdiv as u32 + 1) as f64 * target_baud as f64)) as u64;
        if cd > u16::MAX as u64 {
            continue;
        }
        current_clk_config.cd = cd as u16;
        current_clk_config.bdiv = bdiv;
        let baud = current_clk_config.actual_baud(uart_clk);
        let error = ((baud - target_baud as f64).abs() / target_baud as f64) * 100.0;
        if error < smallest_error {
            best_clk_config = current_clk_config;
            smallest_error = error;
        }
    }

    Ok((best_clk_config, smallest_error))
}

impl ClkConfigRaw {
    #[inline]
    pub const fn new(cd: u16, bdiv: u8) -> Result<Self, DivisorZero> {
        if cd == 0 {
            return Err(DivisorZero);
        }
        Ok(ClkConfigRaw { cd, bdiv })
    }

    /// Auto-calculates the best clock configuration settings for the target baudrate.
    ///
    /// This function assumes [ClkSel::UartRefClk] as the clock source. It returns a tuple
    /// where the first entry is the clock configuration while the second entry is the associated
    /// baud error from 0.0 to 1.0. It is recommended to keep this error below 2-3 %.
    pub fn new_autocalc_with_error(
        io_clks: &IoClocks,
        target_baud: u32,
    ) -> Result<(Self, f64), DivisorZero> {
        Self::new_autocalc_generic(io_clks, ClkSel::UartRefClk, target_baud)
    }

    pub fn new_autocalc_generic(
        io_clks: &IoClocks,
        clk_sel: ClkSel,
        target_baud: u32,
    ) -> Result<(Self, f64), DivisorZero> {
        Self::new_autocalc_with_raw_clk(io_clks.uart_clk(), clk_sel, target_baud)
    }

    pub fn new_autocalc_with_raw_clk(
        uart_clk: Hertz,
        clk_sel: ClkSel,
        target_baud: u32,
    ) -> Result<(Self, f64), DivisorZero> {
        calculate_raw_baud_cfg_smallest_error(uart_clk, clk_sel, target_baud)
    }

    #[inline]
    pub const fn cd(&self) -> u16 {
        self.cd
    }

    #[inline]
    pub const fn bdiv(&self) -> u8 {
        self.bdiv
    }

    #[inline]
    pub fn rounded_baud(&self, sel_clk: Hertz) -> u32 {
        round(self.actual_baud(sel_clk)) as u32
    }

    #[inline]
    pub fn actual_baud(&self, sel_clk: Hertz) -> f64 {
        sel_clk.raw() as f64 / (self.cd as f64 * (self.bdiv + 1) as f64)
    }
}

impl Default for ClkConfigRaw {
    #[inline]
    fn default() -> Self {
        ClkConfigRaw::new(1, 0).unwrap()
    }
}

#[derive(Debug)]
pub struct UartConfig {
    clk_config: ClkConfigRaw,
    chmode: ChMode,
    parity: Parity,
    stopbits: Stopbits,
    chrl: CharLen,
    clk_sel: ClkSel,
}

impl UartConfig {
    pub fn new_with_clk_config(clk_config: ClkConfigRaw) -> Self {
        Self::new(
            clk_config,
            ChMode::default(),
            Parity::default(),
            Stopbits::default(),
            CharLen::default(),
            ClkSel::default(),
        )
    }

    #[inline]
    pub const fn new(
        clk_config: ClkConfigRaw,
        chmode: ChMode,
        parity: Parity,
        stopbits: Stopbits,
        chrl: CharLen,
        clk_sel: ClkSel,
    ) -> Self {
        UartConfig {
            clk_config,
            chmode,
            parity,
            stopbits,
            chrl,
            clk_sel,
        }
    }

    #[inline]
    pub const fn raw_clk_config(&self) -> ClkConfigRaw {
        self.clk_config
    }

    #[inline]
    pub const fn chmode(&self) -> ChMode {
        self.chmode
    }

    #[inline]
    pub const fn parity(&self) -> Parity {
        self.parity
    }

    #[inline]
    pub const fn stopbits(&self) -> Stopbits {
        self.stopbits
    }

    #[inline]
    pub const fn chrl(&self) -> CharLen {
        self.chrl
    }

    #[inline]
    pub const fn clksel(&self) -> ClkSel {
        self.clk_sel
    }
}

// TODO: Impl Debug
pub struct Uart {
    rx: Rx,
    tx: Tx,
    cfg: UartConfig,
}

#[derive(Debug, thiserror::Error)]
#[error("invalid UART ID")]
pub struct InvalidPsUart;

#[derive(Debug, thiserror::Error)]
pub enum UartConstructionError {
    #[error("invalid UART ID")]
    InvalidPsUart(#[from] InvalidPsUart),
    #[error("missmatch between pins index and passed index")]
    IdxMissmatch,
    #[error("invalid pin mux conf for UART")]
    InvalidMuxConf(MuxCfg),
}

impl Uart {
    /// This is the constructor to use the PS UART with EMIO pins to route the UART into the PL
    /// or expose them via the PL package pins.
    ///
    /// A valid PL design which routes the UART pins through into the PL must be used for this to
    /// work.
    pub fn new_with_emio(uart: impl PsUart, cfg: UartConfig) -> Result<Uart, InvalidPsUart> {
        if uart.uart_id().is_none() {
            return Err(InvalidPsUart);
        }
        Ok(Self::new_generic_unchecked(
            uart.reg_block(),
            uart.uart_id().unwrap(),
            cfg,
        ))
    }

    /// This is the constructor to use the PS UART with MIO pins.
    pub fn new_with_mio<TxPinI: TxPin, RxPinI: RxPin>(
        uart: impl PsUart,
        cfg: UartConfig,
        pins: (TxPinI, RxPinI),
    ) -> Result<Self, UartConstructionError>
    where
        (TxPinI, RxPinI): UartPins,
    {
        let id = uart.uart_id();
        if id.is_none() {
            return Err(InvalidPsUart.into());
        }
        if id.unwrap() != TxPinI::UART_IDX || id.unwrap() != RxPinI::UART_IDX {
            return Err(UartConstructionError::IdxMissmatch);
        }
        IoPeriphPin::new(pins.0, UART_MUX_CONF, None);
        IoPeriphPin::new(pins.1, UART_MUX_CONF, None);
        Ok(Self::new_generic_unchecked(
            uart.reg_block(),
            id.unwrap(),
            cfg,
        ))
    }

    /// This is the generic constructor used by all other constructors.
    ///
    /// It does not do any pin checks and resource control. It is recommended to use the other
    /// constructors instead.
    pub fn new_generic_unchecked(
        mut reg_block: MmioUart<'static>,
        uart_id: UartId,
        cfg: UartConfig,
    ) -> Uart {
        let periph_sel = match uart_id {
            UartId::Uart0 => crate::PeriphSelect::Uart0,
            UartId::Uart1 => crate::PeriphSelect::Uart1,
        };
        enable_amba_periph_clk(periph_sel);
        reset(uart_id);
        reg_block.modify_cr(|mut v| {
            v.set_tx_dis(true);
            v.set_rx_dis(true);
            v
        });
        // Disable all interrupts.
        reg_block.write_idr(InterruptControl::new_with_raw_value(0xFFFF_FFFF));
        let mode = Mode::builder()
            .with_chmode(cfg.chmode)
            .with_nbstop(match cfg.stopbits {
                Stopbits::One => zynq7000::uart::Stopbits::One,
                Stopbits::OnePointFive => zynq7000::uart::Stopbits::OnePointFive,
                Stopbits::Two => zynq7000::uart::Stopbits::Two,
            })
            .with_par(match cfg.parity {
                Parity::Even => zynq7000::uart::Parity::Even,
                Parity::Odd => zynq7000::uart::Parity::Odd,
                Parity::None => zynq7000::uart::Parity::NoParity,
            })
            .with_chrl(match cfg.chrl {
                CharLen::SixBits => zynq7000::uart::Chrl::SixBits,
                CharLen::SevenBits => zynq7000::uart::Chrl::SevenBits,
                CharLen::EightBits => zynq7000::uart::Chrl::EightBits,
            })
            .with_clksel(cfg.clk_sel)
            .build();
        reg_block.write_mr(mode);
        reg_block.write_baudgen(
            Baudgen::builder()
                .with_cd(cfg.raw_clk_config().cd())
                .build(),
        );
        reg_block.write_baud_rate_div(
            BaudRateDiv::builder()
                .with_bdiv(cfg.raw_clk_config().bdiv())
                .build(),
        );
        // Soft reset for both TX and RX.
        reg_block.modify_cr(|mut v| {
            v.set_tx_rst(true);
            v.set_rx_rst(true);
            v
        });

        // Write default value.
        reg_block.write_rx_fifo_trigger(FifoTrigger::new_with_raw_value(
            DEFAULT_RX_TRIGGER_LEVEL as u32,
        ));

        // Enable TX and RX.
        reg_block.modify_cr(|mut v| {
            v.set_tx_dis(false);
            v.set_rx_dis(false);
            v.set_tx_en(true);
            v.set_rx_en(true);
            v
        });

        Uart {
            rx: Rx {
                regs: unsafe { reg_block.clone() },
            },
            tx: Tx {
                regs: reg_block,
                idx: uart_id,
            },
            cfg,
        }
    }

    #[inline]
    pub fn set_mode(&mut self, mode: ChMode) {
        self.regs().modify_mr(|mut mr| {
            mr.set_chmode(mode);
            mr
        });
    }

    #[inline]
    pub const fn regs(&mut self) -> &mut MmioUart<'static> {
        &mut self.rx.regs
    }

    #[inline]
    pub const fn cfg(&self) -> &UartConfig {
        &self.cfg
    }

    #[inline]
    pub const fn split(self) -> (Tx, Rx) {
        (self.tx, self.rx)
    }
}

impl embedded_hal_nb::serial::ErrorType for Uart {
    type Error = Infallible;
}

impl embedded_hal_nb::serial::Write for Uart {
    #[inline]
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl embedded_hal_nb::serial::Read for Uart {
    /// Read one byte from the FIFO.
    ///
    /// This operation is infallible because pulling an available byte from the FIFO
    /// always succeeds. If you want to be informed about RX errors, you should look at the
    /// non-blocking API using interrupts, which also tracks the RX error bits.
    #[inline]
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

impl embedded_io::ErrorType for Uart {
    type Error = Infallible;
}

impl embedded_io::Write for Uart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl embedded_io::Read for Uart {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf)
    }
}

/// Reset the UART peripheral using the SLCR reset register for UART.
///
/// Please note that this function will interfere with an already configured
/// UART instance.
#[inline]
pub fn reset(id: UartId) {
    let assert_reset = match id {
        UartId::Uart0 => DualRefAndClkRst::builder()
            .with_periph1_ref_rst(false)
            .with_periph0_ref_rst(true)
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        UartId::Uart1 => DualRefAndClkRst::builder()
            .with_periph1_ref_rst(true)
            .with_periph0_ref_rst(false)
            .with_periph1_cpu1x_rst(true)
            .with_periph0_cpu1x_rst(false)
            .build(),
    };
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl().write_uart(assert_reset);
            // Keep it in reset for one cycle.. not sure if this is necessary.
            cortex_ar::asm::nop();
            regs.reset_ctrl().write_uart(DualRefAndClkRst::DEFAULT);
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::abs_diff_eq;
    use fugit::HertzU32;
    use zynq7000::uart::ClkSel;

    const REF_UART_CLK: HertzU32 = HertzU32::from_raw(50_000_000);
    const REF_UART_CLK_DIV_8: HertzU32 = HertzU32::from_raw(6_250_000);

    #[test]
    fn test_error_calc_0() {
        // Baud 600
        let cfg_0 = ClkConfigRaw::new(10417, 7).unwrap();
        let actual_baud_0 = cfg_0.actual_baud(REF_UART_CLK);
        assert!(abs_diff_eq!(actual_baud_0, 599.980, epsilon = 0.01));
    }

    #[test]
    fn test_error_calc_1() {
        // Baud 9600
        let cfg = ClkConfigRaw::new(81, 7).unwrap();
        let actual_baud = cfg.actual_baud(REF_UART_CLK_DIV_8);
        assert!(abs_diff_eq!(actual_baud, 9645.061, epsilon = 0.01));
    }

    #[test]
    fn test_error_calc_2() {
        // Baud 9600
        let cfg = ClkConfigRaw::new(651, 7).unwrap();
        let actual_baud = cfg.actual_baud(REF_UART_CLK);
        assert!(abs_diff_eq!(actual_baud, 9600.614, epsilon = 0.01));
    }

    #[test]
    fn test_error_calc_3() {
        // Baud 28800
        let cfg = ClkConfigRaw::new(347, 4).unwrap();
        let actual_baud = cfg.actual_baud(REF_UART_CLK);
        assert!(abs_diff_eq!(actual_baud, 28818.44, epsilon = 0.01));
    }

    #[test]
    fn test_error_calc_4() {
        // Baud 921600
        let cfg = ClkConfigRaw::new(9, 5).unwrap();
        let actual_baud = cfg.actual_baud(REF_UART_CLK);
        assert!(abs_diff_eq!(actual_baud, 925925.92, epsilon = 0.01));
    }

    #[test]
    fn test_best_calc_0() {
        let result = ClkConfigRaw::new_autocalc_with_raw_clk(REF_UART_CLK, ClkSel::UartRefClk, 600);
        assert!(result.is_ok());
        let (cfg, _error) = result.unwrap();
        assert_eq!(cfg.cd(), 499);
        assert_eq!(cfg.bdiv(), 166);
    }

    #[test]
    #[cfg(feature = "alloc")]
    fn test_viable_config_calculation() {
        let cfgs = calculate_viable_configs(REF_UART_CLK, ClkSel::UartRefClk, 115200);
        assert!(
            cfgs.iter()
                .find(|(cfg, _error)| { cfg.cd() == 62 && cfg.bdiv() == 6 })
                .is_some()
        );
    }
}
