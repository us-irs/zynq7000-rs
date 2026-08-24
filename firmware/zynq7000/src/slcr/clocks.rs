//! SLCR clock control registers.
//!
//! Writing any of these registers required unlocking the SLCR first.
use super::{CLOCK_CONTROL_OFFSET, SLCR_BASE_ADDR};

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u4, u6, u7, u10};

    /// PLL bypass mode selection.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Bypass {
        /// PLL output is used, not bypassed.
        NotBypassed = 0b00,
        /// This is the default reset value.
        PinStrapSettings = 0b01,
        /// Bypassed unless overridden by pin strapping.
        Bypassed = 0b10,
        /// Bypassed regardless of pin strapping.
        BypassedRegardlessOfPinStrapping = 0b11,
    }

    /// PLL control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct PllControl {
        /// Feedback divisor for the PLL.
        ///
        /// NOTE: Before changing this value, the PLL must first be bypassed and then put into
        /// reset mode.
        #[bits(12..=18, rw)]
        fdiv: u7,
        /// Select source for the ARM PLL bypass control
        #[bit(4, rw)]
        bypass_force: bool,
        /// Select source for the ARM PLL bypass control
        #[bit(3, rw)]
        bypass_qual: bool,
        /// Power-down control
        #[bit(1, rw)]
        pwrdwn: bool,
        /// Reset control
        #[bit(0, rw)]
        reset: bool,
    }

    impl PllControl {
        /// Set the bypass mode using the combined bypass force and qualifier bits.
        #[inline]
        pub fn set_bypass(&mut self, bypass: Bypass) {
            match bypass {
                Bypass::NotBypassed => {
                    self.set_bypass_force(false);
                    self.set_bypass_qual(false);
                }
                Bypass::PinStrapSettings => {
                    self.set_bypass_force(false);
                    self.set_bypass_qual(true);
                }
                Bypass::Bypassed => {
                    self.set_bypass_force(true);
                    self.set_bypass_qual(false);
                }
                Bypass::BypassedRegardlessOfPinStrapping => {
                    self.set_bypass_force(true);
                    self.set_bypass_qual(true);
                }
            }
        }

        /// Read the bypass mode from the combined bypass force and qualifier bits.
        #[inline]
        pub fn bypass(&self) -> Bypass {
            match (self.bypass_force(), self.bypass_qual()) {
                (false, false) => Bypass::NotBypassed,
                (false, true) => Bypass::PinStrapSettings,
                (true, false) => Bypass::Bypassed,
                (true, true) => Bypass::BypassedRegardlessOfPinStrapping,
            }
        }
    }

    /// PLL configuration register.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct PllConfig {
        /// PLL lock circuit count control.
        #[bits(12..=21, rw)]
        lock_count: u10,
        /// Charge Pump control
        #[bits(8..=11, rw)]
        charge_pump: u4,
        /// Loop resistor control
        #[bits(4..=7, rw)]
        loop_resistor: u4,
    }

    /// PLL status register.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct PllStatus {
        /// IO PLL is stable.
        #[bit(5, r)]
        io_pll_stable: bool,
        /// DDR PLL is stable.
        #[bit(4, r)]
        ddr_pll_stable: bool,
        /// ARM PLL is stable.
        #[bit(3, r)]
        arm_pll_stable: bool,
        /// IO PLL is locked.
        #[bit(2, r)]
        io_pll_lock: bool,
        /// DDR PLL is locked.
        #[bit(1, r)]
        drr_pll_lock: bool,
        /// ARM PLL is locked.
        #[bit(0, r)]
        arm_pll_lock: bool,
    }

    /// FPGA PL clock control register.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    #[derive(PartialEq, Eq)]
    pub struct FpgaClockControl {
        /// Second cascade divider. Reset value 0x1
        #[bits(20..=25, rw)]
        divisor_1: u6,
        /// First cascade divider. Reset value 0x18
        #[bits(8..=13, rw)]
        divisor_0: u6,
        /// Clock source selection.
        #[bits(4..=5, rw)]
        srcsel: SrcSelIo,
    }

    impl Default for FpgaClockControl {
        fn default() -> Self {
            Self::ZERO
                .with_divisor_1(u6::new(1))
                .with_divisor_0(u6::new(0x18))
                .with_srcsel(SrcSelIo::IoPll)
        }
    }

    /// Clock source selection for the CPU clock.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug)]
    pub enum SrcSelArm {
        /// ARM PLL output.
        ArmPll = 0b00,
        /// ARM PLL output, alternate encoding.
        ArmPllAlt = 0b01,
        /// DDR PLL output.
        DdrPll = 0b10,
        /// IO PLL output.
        IoPll = 0b11,
    }

    /// ARM/CPU clock control register.
    #[bitbybit::bitfield(u32, debug, default = 0x0, forbid_overlaps)]
    pub struct ArmClockControl {
        /// CPU peripheral clock (CPU_1x) enable.
        #[bit(28, rw)]
        cpu_peri_clk_act: bool,
        /// CPU 1x clock enable.
        #[bit(27, rw)]
        cpu_1x_clk_act: bool,
        /// CPU 2x clock enable.
        #[bit(26, rw)]
        cpu_2x_clk_act: bool,
        /// CPU 3x or 2x clock enable, depending on the clock ratio mode.
        #[bit(25, rw)]
        cpu_3or2x_clk_act: bool,
        /// CPU 6x or 4x clock enable, depending on the clock ratio mode.
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

    /// DDR clock control register.
    #[bitbybit::bitfield(u32, debug, forbid_overlaps)]
    pub struct DdrClockControl {
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

    /// DDR DCI (Digitally Controlled Impedance) clock control register.
    #[bitbybit::bitfield(u32, default = 0x0, debug, forbid_overlaps)]
    pub struct DciClockControl {
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

    /// CPU clock ratio select register.
    #[bitbybit::bitfield(u32, debug, default = 0x0, forbid_overlaps)]
    pub struct ClockRatioSelectReg {
        /// Reset value: 0x1 (6:2:1 clock)
        #[bit(0, rw)]
        sel: CpuClockRatio,
    }

    /// CPU clock ratio mode, controlling the relationship between the CPU core and bus clocks.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum CpuClockRatio {
        /// 4:2:1 clock ratio, which is an abbreviation for 4:2:2:1.
        ///
        /// The 4x clock is calculated by dividing the reference clock
        /// by the divisor, the rest by dividing by 2, 2 and 4 respectively.
        FourToTwoToOne = 0b0,
        /// 6:2:1 clock ratio, which is an abbreviation for 6:3:2:1.
        ///
        /// The 6x clock is calculated by dividing the reference clock
        /// by the divisor, the rest by dividing by 2, 3 and 6 respectively.
        SixToTwoToOne = 0b1,
    }

    /// Clock source selection for IO peripheral clocks.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SrcSelIo {
        /// IO PLL output.
        IoPll = 0b00,
        /// IO PLL output, alternate encoding.
        IoPllAlt = 0b01,
        /// ARM PLL output.
        ArmPll = 0b10,
        /// DDR PLL output.
        DdrPll = 0b11,
    }

    impl PartialEq for SrcSelIo {
        fn eq(&self, other: &Self) -> bool {
            match (self, other) {
                // IoPll and IoPllAlt are equal to each other
                (Self::IoPll, Self::IoPll)
                | (Self::IoPll, Self::IoPllAlt)
                | (Self::IoPllAlt, Self::IoPll)
                | (Self::IoPllAlt, Self::IoPllAlt) => true,

                // For other variants, only equal if exactly the same
                (Self::ArmPll, Self::ArmPll) => true,
                (Self::DdrPll, Self::DdrPll) => true,

                // Otherwise, not equal
                _ => false,
            }
        }
    }

    /// Gigabit Ethernet controller TX clock control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct GigEthClockControl {
        /// Second cascade divider.
        #[bits(20..=25, rw)]
        divisor_1: u6,
        /// First cascade divider.
        #[bits(8..=13, rw)]
        divisor_0: u6,
        /// Use the EMIO TX clock instead of the MIO TX clock.
        #[bit(6, rw)]
        use_emio_tx_clk: bool,
        /// Clock source selection.
        #[bits(4..=5, rw)]
        srcsel: SrcSelIo,
        /// Clock enable.
        #[bit(0, rw)]
        clk_act: bool,
    }

    /// Clock source selection for the Gigabit Ethernet RX clock.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SrcSelGigEthRclk {
        /// RX clock is sourced from the MIO pins.
        Mio = 0,
        /// RX clock is sourced from the EMIO pins.
        Emio = 1,
    }

    /// Gigabit Ethernet controller RX clock control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct GigEthRclkControl {
        /// Clock source selection.
        #[bit(4, rw)]
        srcsel: SrcSelGigEthRclk,
        /// Enable the ethernet controller RX clock.
        #[bit(0, rw)]
        clk_enable: bool,
    }

    /// CAN controller reference clock control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct CanClockControl {
        /// Second cascade divider.
        #[bits(20..=25, rw)]
        divisor_1: u6,
        /// First cascade divider.
        #[bits(8..=13, rw)]
        divisor_0: u6,
        /// Clock source selection.
        #[bits(4..=5, rw)]
        srcsel: SrcSelIo,
        /// CAN 1 clock enable.
        #[bit(1, rw)]
        clk_1_act: bool,
        /// CAN 0 clock enable.
        #[bit(0, rw)]
        clk_0_act: bool,
    }

    /// Clock control register for a peripheral with a single clock output.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct SingleCommonPeriphIoClockControl {
        /// Clock divisor.
        #[bits(8..=13, rw)]
        divisor: u6,
        /// Clock source selection.
        #[bits(4..=5, rw)]
        srcsel: SrcSelIo,
        /// Clock enable.
        #[bit(0, rw)]
        clk_act: bool,
    }

    /// Clock control register for a peripheral with two clock outputs.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DualCommonPeriphIoClockControl {
        /// Clock divisor.
        #[bits(8..=13, rw)]
        divisor: u6,
        /// Clock source selection.
        #[bits(4..=5, rw)]
        srcsel: SrcSelIo,
        /// Clock 1 enable.
        #[bit(1, rw)]
        clk_1_act: bool,
        /// Clock 0 enable.
        #[bit(0, rw)]
        clk_0_act: bool,
    }

    /// Clock source selection for the Trace Port Interface Unit (TPIU) clock.
    #[bitbybit::bitenum(u3, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SrcSelTpiu {
        /// IO PLL output.
        IoPll = 0b000,
        /// IO PLL output, alternate encoding.
        IoPllAlt = 0b001,
        /// ARM PLL output.
        ArmPll = 0b010,
        /// DDR PLL output.
        DdrPll = 0b011,
        /// Trace clock from the EMIO pins.
        EmioTraceClk = 0b100,
        /// Trace clock from the EMIO pins, alternate encoding.
        EmioTraceClkAlt0 = 0b101,
        /// Trace clock from the EMIO pins, alternate encoding.
        EmioTraceClkAlt1 = 0b110,
        /// Trace clock from the EMIO pins, alternate encoding.
        EmioTraceClkAlt2 = 0b111,
    }

    /// Trace port clock control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct TracePortClockControl {
        /// Clock divisor.
        #[bits(8..=13, rw)]
        divisor: u6,
        /// Clock source selection.
        #[bits(4..=6, rw)]
        srcsel: SrcSelTpiu,
        /// Trace 1x clock enable.
        #[bit(1, rw)]
        clk_1x_clk_act: bool,
        /// Trace clock enable.
        #[bit(0, rw)]
        clk_act: bool,
    }

    /// AMBA peripheral clock control.
    ///
    /// These clocks must be enabled if you want to read from the peripheral register space.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct AperClockControl {
        /// SMC peripheral clock enable.
        #[bit(24, rw)]
        smc_1x_clk_act: bool,
        /// LQSPI peripheral clock enable.
        #[bit(23, rw)]
        lqspi_1x_clk_act: bool,
        /// GPIO peripheral clock enable.
        #[bit(22, rw)]
        gpio_1x_clk_act: bool,
        /// UART 1 peripheral clock enable.
        #[bit(21, rw)]
        uart_1_1x_clk_act: bool,
        /// UART 0 peripheral clock enable.
        #[bit(20, rw)]
        uart_0_1x_clk_act: bool,
        /// I2C 1 peripheral clock enable.
        #[bit(19, rw)]
        i2c_1_1x_clk_act: bool,
        /// I2C 0 peripheral clock enable.
        #[bit(18, rw)]
        i2c_0_1x_clk_act: bool,
        /// CAN 1 peripheral clock enable.
        #[bit(17, rw)]
        can_1_1x_clk_act: bool,
        /// CAN 0 peripheral clock enable.
        #[bit(16, rw)]
        can_0_1x_clk_act: bool,
        /// SPI 1 peripheral clock enable.
        #[bit(15, rw)]
        spi_1_1x_clk_act: bool,
        /// SPI 0 peripheral clock enable.
        #[bit(14, rw)]
        spi_0_1x_clk_act: bool,
        /// SDIO 1 peripheral clock enable.
        #[bit(11, rw)]
        sdio_1_1x_clk_act: bool,
        /// SDIO 0 peripheral clock enable.
        #[bit(10, rw)]
        sdio_0_1x_clk_act: bool,
        /// Gigabit Ethernet 1 peripheral clock enable.
        #[bit(7, rw)]
        gem_1_1x_clk_act: bool,
        /// Gigabit Ethernet 0 peripheral clock enable.
        #[bit(6, rw)]
        gem_0_1x_clk_act: bool,
        /// USB 1 CPU interface clock enable.
        #[bit(3, rw)]
        usb_1_cpu_1x_clk_act: bool,
        /// USB 0 CPU interface clock enable.
        #[bit(2, rw)]
        usb_0_cpu_1x_clk_act: bool,
        /// DMA controller CPU interface clock enable.
        #[bit(0, rw)]
        dma_cpu_2x_clk_act: bool,
    }
}

/// FPGA PL clock register block, including throttling control.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct FpgaClockControlRegisters {
    /// Clock control register
    ctrl: FpgaClockControl,
    /// Throttle control register
    thr_ctrl: u32,
    /// Throttle count register
    thr_cnt: u32,
    /// Throttle status register
    thr_status: u32,
}

static_assertions::const_assert_eq!(core::mem::size_of::<FpgaClockControlRegisters>(), 0x10);

/// SLCR clock generation register block.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct ClockControlRegisters {
    /// ARM PLL control register
    arm_pll_ctrl: PllControl,
    /// DDR PLL control register
    ddr_pll_ctrl: PllControl,
    /// IO PLL control register
    io_pll_ctrl: PllControl,
    /// PLL status register
    pll_status: PllStatus,
    /// ARM PLL configuration register
    arm_pll_cfg: PllConfig,
    /// DDR PLL configuration register
    ddr_pll_cfg: PllConfig,
    /// IO PLL configuration register
    io_pll_cfg: PllConfig,
    _gap0: u32,
    /// ARM clock control register
    arm_clk_ctrl: ArmClockControl,
    /// DDR clock control register
    ddr_clk_ctrl: DdrClockControl,
    /// DDR DCI clock control register
    dci_clk_ctrl: DciClockControl,
    /// AMBA peripheral clock control
    aper_clk_ctrl: AperClockControl,
    /// USB 0 clock control register
    usb_0_clk_ctrl: u32,
    /// USB 1 clock control register
    usb_1_clk_ctrl: u32,
    /// Gigabit Ethernet 0 RX clock control register
    gem_0_rclk_ctrl: GigEthRclkControl,
    /// Gigabit Ethernet 1 RX clock control register
    gem_1_rclk_ctrl: GigEthRclkControl,
    /// Gigabit Ethernet 0 TX clock control register
    gem_0_clk_ctrl: GigEthClockControl,
    /// Gigabit Ethernet 1 TX clock control register
    gem_1_clk_ctrl: GigEthClockControl,
    /// SMC clock control register
    smc_clk_ctrl: SingleCommonPeriphIoClockControl,
    /// LQSPI clock control register
    lqspi_clk_ctrl: SingleCommonPeriphIoClockControl,
    /// SDIO clock control register
    sdio_clk_ctrl: DualCommonPeriphIoClockControl,
    /// UART reference clock control register
    uart_clk_ctrl: DualCommonPeriphIoClockControl,
    /// SPI clock control register
    spi_clk_ctrl: DualCommonPeriphIoClockControl,
    /// CAN reference clock control register
    can_clk_ctrl: CanClockControl,
    /// CAN MIO clock control register
    can_mioclk_ctrl: u32,
    /// Debug or Trace Port clock control.
    dbg_clk_ctrl: TracePortClockControl,
    /// PCAP clock control register
    pcap_clk_ctrl: SingleCommonPeriphIoClockControl,
    /// Central interconnect clock control register
    topsw_clk_ctrl: u32,
    /// FPGA PL clock 0 register block
    #[mmio(Inner)]
    fpga_0_clk_ctrl: FpgaClockControlRegisters,
    /// FPGA PL clock 1 register block
    #[mmio(Inner)]
    fpga_1_clk_ctrl: FpgaClockControlRegisters,
    /// FPGA PL clock 2 register block
    #[mmio(Inner)]
    fpga_2_clk_ctrl: FpgaClockControlRegisters,
    /// FPGA PL clock 3 register block
    #[mmio(Inner)]
    fpga_3_clk_ctrl: FpgaClockControlRegisters,
    _gap1: [u32; 5],
    /// CPU clock ratio select register
    clk_ratio_select: ClockRatioSelectReg,
}

impl ClockControlRegisters {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioClockControlRegisters<'static> {
        unsafe { Self::new_mmio_at(SLCR_BASE_ADDR + CLOCK_CONTROL_OFFSET) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<ClockControlRegisters>(), 0xC8);
