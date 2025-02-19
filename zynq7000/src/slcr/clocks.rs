//! SLCR clock control registers.
//!
//! Writing any of these registers required unlocking the SLCR first.
use super::{CLOCK_CONTROL_OFFSET, SLCR_BASE_ADDR};
use arbitrary_int::{u4, u6, u7, u10};

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum BypassForce {
    EnabledOrSetByBootMode = 0b0,
    Bypassed = 0b1,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum BypassQual {
    BypassForceBit = 0b0,
    BootModeFourthBit = 0b1,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct PllCtrl {
    /// Feedback divisor for the PLL.
    ///
    /// NOTE: Before changing this value, the PLL must first be bypassed and then put into
    /// reset mode.
    #[bits(12..=18, rw)]
    fdiv: u7,
    /// Select source for the ARM PLL bypass control
    #[bit(4, rw)]
    bypass_force: BypassForce,
    /// Select source for the ARM PLL bypass control
    #[bit(3, rw)]
    bypass_qual: BypassQual,
    // Power-down control
    #[bit(1, rw)]
    pwrdwn: bool,
    /// Reset control
    #[bit(0, rw)]
    reset: bool,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct PllCfg {
    #[bits(12..=21, rw)]
    lock_count: u10,
    /// Charge Pump control
    #[bits(8..=11, rw)]
    pll_cp: u4,
    /// Loop resistor control
    #[bits(4..=7, rw)]
    pll_res: u4,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct PllStatus {
    #[bit(5)]
    io_pll_stable: bool,
    #[bit(4)]
    ddr_pll_stable: bool,
    #[bit(3)]
    arm_pll_stable: bool,
    #[bit(2)]
    io_pll_lock: bool,
    #[bit(1)]
    drr_pll_lock: bool,
    #[bit(0)]
    arm_pll_lock: bool,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct FpgaClkControl {
    // Reset value 0x1
    #[bits(20..=25, rw)]
    divisor_1: u6,
    // Reset value 0x18
    #[bits(8..=13, rw)]
    divisor_0: u6,
    #[bits(4..=5, rw)]
    srcsel: SrcSelIo,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct FpgaClkBlock {
    clk_ctrl: FpgaClkControl,
    thr_ctrl: u32,
    thr_cnt: u32,
    thr_status: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<FpgaClkBlock>(), 0x10);

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug)]
pub enum SrcSelArm {
    ArmPll = 0b00,
    ArmPllAlt = 0b01,
    DdrPll = 0b10,
    IoPll = 0b11,
}

#[bitbybit::bitfield(u32)]
pub struct ArmClkCtrl {
    #[bit(28, rw)]
    cpu_peri_clk_act: bool,
    #[bit(27, rw)]
    cpu_1x_clk_act: bool,
    #[bit(26, rw)]
    cpu_2x_clk_act: bool,
    #[bit(25, rw)]
    cpu_3or2x_clk_act: bool,
    #[bit(24, rw)]
    cpu_6or4x_clk_act: bool,
    /// Reset value: 0x4. There is a requirement for the quality of the high speed clock that
    /// it has to be divided by an even number. This field must be equal to or greater than 2.
    #[bits(8..=13, rw)]
    divisor: u6,
    /// Reset value: 0x0
    #[bits(4..=5, rw)]
    srcsel: SrcSelArm,
}

#[bitbybit::bitfield(u32)]
pub struct DdrClkCtrl {
    /// Divisor for DDR 2x clock. Reset value: 0x6
    #[bits(26..=31, rw)]
    div_2x_clk: u6,
    /// Divisor for DDR 3x clock. Only even divisors are allowed! Reset value: 0x4
    #[bits(20..=25, rw)]
    div_3x_clk: u6,
    /// Reset value: 0x1
    #[bit(1, rw)]
    ddr_2x_clk_act: bool,
    /// Reset value: 0x1
    #[bit(0, rw)]
    ddr_3x_clk_act: bool,
}

#[bitbybit::bitfield(u32)]
pub struct DciClkCtrl {
    /// Second cascade divider. Reset value: 0x1E
    #[bits(20..=25, rw)]
    divisor_1: u6,
    /// Reset value: 0x32
    #[bits(8..=13, rw)]
    divisor_0: u6,
    /// Reset value: 0x1
    #[bit(0, rw)]
    clk_act: bool,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct ClockRatioSelectReg {
    /// Reset value: 0x1 (6:2:1 clock)
    #[bit(0, rw)]
    sel: ClockRatioSelect,
}

#[bitbybit::bitenum(u1, exhaustive = true)]
#[derive(Debug)]
pub enum ClockRatioSelect {
    /// 4:2:1 clock ratio, which is an abbreviation for 4:2:2:1.
    FourToTwoToOne = 0b0,
    /// 6:2:1 clock ratio, which is an abbreviation for 6:3:2:1.
    SixToTwoToOne = 0b1,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug)]
pub enum SrcSelIo {
    IoPll = 0b00,
    IoPllAlt = 0b01,
    ArmPll = 0b10,
    DdrPll = 0b11,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct GigEthClkCtrl {
    #[bits(20..=25, rw)]
    divisor_1: u6,
    #[bits(8..=13, rw)]
    divisor_0: u6,
    #[bit(6, rw)]
    use_emio_tx_clk: bool,
    #[bits(4..=5, rw)]
    srcsel: SrcSelIo,
    #[bit(0, rw)]
    clk_act: bool,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct CanClkCtrl {
    #[bits(20..=25, rw)]
    divisor_1: u6,
    #[bits(8..=13, rw)]
    divisor_0: u6,
    #[bits(4..=5, rw)]
    srcsel: SrcSelIo,
    #[bit(1, rw)]
    clk_1_act: bool,
    #[bit(0, rw)]
    clk_0_act: bool,
}

#[bitbybit::bitfield(u32)]
pub struct SingleCommonPeriphIoClkCtrl {
    #[bits(8..=13, rw)]
    divisor: u6,
    #[bits(4..=5, rw)]
    srcsel: SrcSelIo,
    #[bit(0, rw)]
    clk_act: bool,
}

#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct DualCommonPeriphIoClkCtrl {
    #[bits(8..=13, rw)]
    divisor: u6,
    #[bits(4..=5, rw)]
    srcsel: SrcSelIo,
    #[bit(1, rw)]
    clk_1_act: bool,
    #[bit(0, rw)]
    clk_0_act: bool,
}

#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(Debug)]
pub enum SrcSelTpiu {
    IoPll = 0b000,
    IoPllAlt = 0b001,
    ArmPll = 0b010,
    DdrPll = 0b011,
    EmioTraceClk = 0b100,
    EmioTraceClkAlt0 = 0b101,
    EmioTraceClkAlt1 = 0b110,
    EmioTraceClkAlt2 = 0b111,
}

#[bitbybit::bitfield(u32)]
pub struct TracePortClkCtrl {
    #[bits(8..=13, rw)]
    divisor: u6,
    #[bits(4..=6, rw)]
    srcsel: SrcSelTpiu,
    #[bit(1, rw)]
    clk_1x_clk_act: bool,
    #[bit(0, rw)]
    clk_act: bool,
}

/// AMBA peripheral clock control.
///
/// These clocks must be enabled if you want to read from the peripheral register space.
#[bitbybit::bitfield(u32)]
#[derive(Debug)]
pub struct AperClkCtrl {
    #[bit(24, rw)]
    smc_1x_clk_act: bool,
    #[bit(23, rw)]
    lqspi_1x_clk_act: bool,
    #[bit(22, rw)]
    gpio_1x_clk_act: bool,
    #[bit(21, rw)]
    uart_1_1x_clk_act: bool,
    #[bit(20, rw)]
    uart_0_1x_clk_act: bool,
    #[bit(19, rw)]
    i2c_1_1x_clk_act: bool,
    #[bit(18, rw)]
    i2c_0_1x_clk_act: bool,
    #[bit(17, rw)]
    can_1_1x_clk_act: bool,
    #[bit(16, rw)]
    can_0_1x_clk_act: bool,
    #[bit(15, rw)]
    spi_1_1x_clk_act: bool,
    #[bit(14, rw)]
    spi_0_1x_clk_act: bool,
    #[bit(11, rw)]
    sdio_1_1x_clk_act: bool,
    #[bit(10, rw)]
    sdio_0_1x_clk_act: bool,
    #[bit(7, rw)]
    gem_1_1x_clk_act: bool,
    #[bit(6, rw)]
    gem_0_1x_clk_act: bool,
    #[bit(3, rw)]
    usb_1_cpu_1x_clk_act: bool,
    #[bit(2, rw)]
    usb_0_cpu_1x_clk_act: bool,
    #[bit(0, rw)]
    dma_cpu_2x_clk_act: bool,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct ClockControl {
    arm_pll: PllCtrl,
    ddr_pll: PllCtrl,
    io_pll: PllCtrl,
    pll_status: PllStatus,
    arm_pll_cfg: PllCfg,
    ddr_pll_cfg: PllCfg,
    io_pll_cfg: PllCfg,
    _gap0: u32,
    arm_clk_ctrl: ArmClkCtrl,
    ddr_clk_ctrl: DdrClkCtrl,
    dci_clk_ctrl: DciClkCtrl,
    /// AMBA peripheral clock control
    aper_clk_ctrl: AperClkCtrl,
    usb_0_clk_ctrl: u32,
    usb_1_clk_ctrl: u32,
    gem_0_rclk_ctrl: u32,
    gem_1_rclk_ctrl: u32,
    gem_0_clk_ctrl: GigEthClkCtrl,
    gem_1_clk_ctrl: GigEthClkCtrl,
    smc_clk_ctrl: SingleCommonPeriphIoClkCtrl,
    lqspi_clk_ctrl: SingleCommonPeriphIoClkCtrl,
    sdio_clk_ctrl: DualCommonPeriphIoClkCtrl,
    uart_clk_ctrl: DualCommonPeriphIoClkCtrl,
    spi_clk_ctrl: DualCommonPeriphIoClkCtrl,
    can_clk_ctrl: CanClkCtrl,
    can_mioclk_ctrl: u32,
    /// Debug or Trace Port clock control.
    dbg_clk_ctrl: TracePortClkCtrl,
    pcap_clk_ctrl: SingleCommonPeriphIoClkCtrl,
    topsw_clk_ctrl: u32,
    #[mmio(inner)]
    fpga_0_clk_ctrl: FpgaClkBlock,
    #[mmio(inner)]
    fpga_1_clk_ctrl: FpgaClkBlock,
    #[mmio(inner)]
    fpga_2_clk_ctrl: FpgaClkBlock,
    #[mmio(inner)]
    fpga_3_clk_ctrl: FpgaClkBlock,
    _gap1: [u32; 5],
    clk_621_true: ClockRatioSelectReg,
}

impl ClockControl {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioClockControl<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR + CLOCK_CONTROL_OFFSET) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<ClockControl>(), 0xC8);
