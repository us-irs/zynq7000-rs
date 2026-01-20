//! # Global Interrupt Controller (GIC) module
//!
//! The primary interface to configure and allow handling the interrupts are the
//! [GicConfigurator] and the [GicInterruptHelper] structures.
#![deny(missing_docs)]
use arbitrary_int::prelude::*;

use aarch32_cpu::interrupt;
use zynq7000::gic::{
    CpuInterfaceRegisters, DistributorControlRegister, DistributorRegisters, InterfaceControl,
    InterruptProcessorTargetRegister, InterruptSignalRegister, MmioCpuInterfaceRegisters,
    MmioDistributorRegisters, PriorityRegister,
};

/// Spurious interrupt ID.
pub const SPURIOUS_INTERRUPT_ID: u32 = 1023;

/// Highest interrupt priority (smallest number).
pub const HIGHEST_PRIORITY: u8 = 0;
/// Lowest interrupt priority (largest number).
pub const LOWEST_PRIORITY: u8 = 31;

/// These fixed values must be programmed according to the Zynq7000 TRM p.236.
/// Configures #32 to #47.
pub const ICFR_2_FIXED_VALUE: u32 = 0b01010101010111010101010001011111;
/// These fixed values must be programmed according to the Zynq7000 TRM p.236.
/// This configures `PL[2:0]` to high-level sensitivity.
/// Configures #48 to #63.
pub const ICFR_3_FIXED_VALUE: u32 = 0b01010101010101011101010101010101;
/// These fixed values must be programmed according to the Zynq7000 TRM p.236.
/// This configures `PL[7:3]` to high-level sensitivity.
/// Configures #64 to #79.
pub const ICFR_4_FIXED_VALUE: u32 = 0b01110101010101010101010101010101;
/// These fixed values must be programmed according to the Zynq7000 TRM p.236.
/// This configures `PL[15:8]` to high-level sensitivity.
/// Configures #80 to #95.
pub const ICFR_5_FIXED_VALUE: u32 = 0b00000011010101010101010101010101;

/// Helper value to target all interrupts which can be targetted to CPU 0
pub const TARGETS_ALL_CPU_0_IPTR_VAL: InterruptProcessorTargetRegister =
    InterruptProcessorTargetRegister::new_with_raw_value(0x01010101);
/// Helper value to target all interrupts which can be targetted to CPU 1
pub const TARGETS_ALL_CPU_1_IPTR_VAL: InterruptProcessorTargetRegister =
    InterruptProcessorTargetRegister::new_with_raw_value(0x02020202);

/// Mask for activating all softare generated interrupts.
pub const ACTIVATE_ALL_SGIS_MASK_ISER: u32 = 0x0000_FFFF;
/// Mask for activating all private peripheral interrupts.
pub const ACTIVATE_ALL_PPIS_MASK_ISER: u32 = 0xF800_0000;

/// Shared peripheral interrupt sensitivity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SpiSensitivity {
    /// Level triggered interrupt.
    Level = 0b01,
    /// Edge triggered interrupt.
    Edge = 0b11,
}

/// CPU enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Cpu {
    /// CPU 0.
    Cpu0,
    /// CPU 1.
    Cpu1,
}

bitflags::bitflags! {
    /// Target CPU bitflags.
    #[derive(Debug, Eq, PartialEq, Clone, Copy)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    pub struct TargetCpus: u8 {
        /// No CPU.
        const NONE = 0b00;
        /// CPU 0.
        const CPU_0 = 0b01;
        /// CPU 1.
        const CPU_1 = 0b10;
        /// Both CPUs.
        const BOTH_CPUS = 0b11;
    }
}

/// Private Peripheral Interrupt (PPI) which are private to the CPU.
#[derive(Debug, Eq, PartialEq, Clone, Copy, num_enum::TryFromPrimitive)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum PpiInterrupt {
    /// Global timer.
    GlobalTimer = 27,
    /// Interrupt signal from the PL. CPU0: `IRQF2P[18]` and CPU1: `IRQF2P[19]`
    NFiq = 28,
    /// CPU private timer.
    CpuPrivateTimer = 29,
    /// AWDT0 and AWDT1 for each CPU.
    Awdt = 30,
    /// Interrupt signal from the PL. CPU0: `IRQF2P[16]` and CPU1: `IRQF2P[17]`
    NIrq = 31,
}

/// Shared Peripheral Interrupt IDs.
#[derive(Debug, Eq, PartialEq, Clone, Copy, num_enum::TryFromPrimitive)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum SpiInterrupt {
    /// CPU 0.
    Cpu0 = 32,
    /// CPU 1.
    Cpu1 = 33,
    /// L2 cache.
    L2Cache = 34,
    /// On-chip memory.
    Ocm = 35,
    /// Reserved.
    _Reserved0 = 36,
    /// Performance monitor unit 0.
    Pmu0 = 37,
    /// Performance monitor unit 1.
    Pmu1 = 38,
    /// XADC.
    Xadc = 39,
    /// Device configuration.
    DevC = 40,
    /// System watchdog timer.
    Swdt = 41,
    /// Triple timer counter 00.
    Ttc00 = 42,
    /// Triple timer counter 01.
    Ttc01 = 43,
    /// Triple timer counter 02.
    Ttc02 = 44,
    /// DMAC abort.
    DmacAbort = 45,
    /// DMAC 0.
    Dmac0 = 46,
    /// DMAC 1.
    Dmac1 = 47,
    /// DMAC 2.
    Dmac2 = 48,
    /// DMAC 3.
    Dmac3 = 49,
    /// Shared memory controller.
    Smc = 50,
    /// Quad SPI.
    Qspi = 51,
    /// GPIO.
    Gpio = 52,
    /// USB 0.
    Usb0 = 53,
    /// Ethernet 0.
    Eth0 = 54,
    /// Ethernet 0 wakeup.
    Eth0Wakeup = 55,
    /// SDIO 0.
    Sdio0 = 56,
    /// I2C 0.
    I2c0 = 57,
    /// SPI 0.
    Spi0 = 58,
    /// UART 0.
    Uart0 = 59,
    /// CAN 0.
    Can0 = 60,
    /// Programmable Logic 0.
    Pl0 = 61,
    /// Programmable Logic 1.
    Pl1 = 62,
    /// Programmable Logic 2.
    Pl2 = 63,
    /// Programmable Logic 3.
    Pl3 = 64,
    /// Programmable Logic 4.
    Pl4 = 65,
    /// Programmable Logic 5.
    Pl5 = 66,
    /// Programmable Logic 6.
    Pl6 = 67,
    /// Programmable Logic 7.
    Pl7 = 68,
    /// Triple timer counter 10.
    Ttc10 = 69,
    /// Triple timer counter 11.
    Ttc11 = 70,
    /// Triple timer counter 12.
    Ttc12 = 71,
    /// DMAC 4.
    Dmac4 = 72,
    /// DMAC 5.
    Dmac5 = 73,
    /// DMAC 6.
    Dmac6 = 74,
    /// DMAC 7.
    Dmac7 = 75,
    /// USB 1.
    Usb1 = 76,
    /// Ethernet 1.
    Eth1 = 77,
    /// Ethernet 1 wakeup.
    Eth1Wakeup = 78,
    /// SDIO 1.
    Sdio1 = 79,
    /// I2C 1.
    I2c1 = 80,
    /// SPI 1.
    Spi1 = 81,
    /// UART 1.
    Uart1 = 82,
    /// CAN 1.
    Can1 = 83,
    /// Programmable Logic 8.
    Pl8 = 84,
    /// Programmable Logic 9.
    Pl9 = 85,
    /// Programmable Logic 10.
    Pl10 = 86,
    /// Programmable Logic 11.
    Pl11 = 87,
    /// Programmable Logic 12.
    Pl12 = 88,
    /// Programmable Logic 13.
    Pl13 = 89,
    /// Programmable Logic 14.
    Pl14 = 90,
    /// Programmable Logic 15.
    Pl15 = 91,
    /// Snoop control unit parity.
    ScuParity = 92,
}

/// Interrupt ID wrapper.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Interrupt {
    /// Software-generated interrupt (SGI).
    Sgi(usize),
    /// Private peripheral interrupt (PPI).
    Ppi(PpiInterrupt),
    /// Shared peripheral interrupt (SPI).
    Spi(SpiInterrupt),
    /// Detects an invalid interrupt ID.
    Invalid(usize),
    /// Spurious interrupt (ID# 1023, [SPURIOUS_INTERRUPT_ID]).
    Spurious,
}

/// Interrupt information structure.
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InterruptInfo {
    raw_reg: InterruptSignalRegister,
    interrupt: Interrupt,
    cpu_id: u8,
}

impl InterruptInfo {
    /// Raw interrupt signal register value.
    #[inline]
    pub const fn raw_reg(&self) -> InterruptSignalRegister {
        self.raw_reg
    }

    /// CPU ID.
    #[inline]
    pub const fn cpu_id(&self) -> u8 {
        self.cpu_id
    }

    /// Interrupt ID.
    #[inline]
    pub const fn interrupt(&self) -> Interrupt {
        self.interrupt
    }
}

/// Invalid priority value error.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[error("Invalid priority value {0}, range is [0, 31]")]
pub struct InvalidPriorityValue(pub u8);

/// Invalid programmable logic interrupt ID.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[error("Invalid PL interrupt ID {0}")]
pub struct InvalidPlInterruptId(pub usize);

/// Invalid Shared Peripheral Interrupt (SPI) ID.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[error("Invalid SPI interrupt ID {0}")]
pub struct InvalidSpiInterruptId(pub usize);

/// Invalid Software Generated Interrupt (SGI) ID.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[error("Invalid SGI interrupt ID {0}")]
pub struct InvalidSgiInterruptId(pub usize);

/// Higher-level GIC controller for the Zynq70000 SoC.
///
/// The flow of using this controller is as follows:
///
/// 1. Create the controller using [Self::new_with_init]. You can use the [zynq7000::Peripherals]
///    structure or the [zynq7000::gic::CpuInterfaceRegisters::new_mmio] and [zynq7000::gic::DistributorRegisters::new_mmio]
///    functions to retrieve the MMIO instances. The constructor configures all PL interrupts
///    sensivities to high-level sensitivity and configures all sensitivities which are expected
///    to have a certain value. It also sets the priority mask to 0xff by calling
///    [Self::set_priority_mask] to prevent masking of the interrupts.
/// 2. Perform the configuration of the interrupt targets and the interrupt sensitivities.
///    The CPU targets are encoded with [TargetCpus] while the sensitivities are encoded by
///    the [SpiSensitivity] enum. You can use the following (helper) API to configure the
///    interrupts:
///
///     - [Self::set_spi_interrupt_target_for_cpu]
///     - [Self::set_spi_interrupt_cpu_target_flags]
///     - [Self::set_all_spi_interrupt_targets_cpu0]
///     - [Self::set_pl_interrupt_sensitivity]
///
/// 3. Enable all required interrupts. The following API can be used for this:
///
///     - [Self::enable_sgi_interrupt]
///     - [Self::enable_ppi_interrupt]
///     - [Self::enable_spi_interrupt]
///     - [Self::enable_all_spi_interrupts]
///     - [Self::enable_all_ppi_interrupts]
///     - [Self::enable_all_sgi_interrupts]
///     - [Self::enable_all_interrupts]
///
///    You might also chose to enable these interrupts at run-time after the GIC was started.
/// 4. Start the GIC by calling [Self::update_ctrl_regs] with the required settings or
///    with [Self::enable] which assumes a certain configuration.
/// 5. Enable interrupts for the Cortex-A core by calling [Self::enable_interrupts].
///
/// For the handling of the interrupts, you can use the [GicInterruptHelper] which assumes a
/// properly configured GIC.
pub struct GicConfigurator {
    /// GIC CPU interface registers.
    pub gicc: MmioCpuInterfaceRegisters<'static>,
    /// GIC Distributor interface registers.
    pub gicd: MmioDistributorRegisters<'static>,
}

impl GicConfigurator {
    /// Create a new GIC controller instance and calls [Self::initialize] to perform
    /// strongly recommended initialization routines for the GIC.
    #[inline]
    pub fn new_with_init(
        gicc: MmioCpuInterfaceRegisters<'static>,
        gicd: MmioDistributorRegisters<'static>,
    ) -> Self {
        let mut gic = GicConfigurator { gicc, gicd };
        gic.initialize();
        gic
    }

    /// Create a new GIC controller instance without performing any initialization routines.
    ///
    /// # Safety
    ///
    /// This creates the GIC without performing any of the initialization routines necessary
    /// for proper operation. It also circumvents ownership checks. It is mainly intended to be
    /// used inside the interrupt handler.
    #[inline]
    pub unsafe fn steal() -> Self {
        GicConfigurator {
            gicc: unsafe { CpuInterfaceRegisters::new_mmio_fixed() },
            gicd: unsafe { DistributorRegisters::new_mmio_fixed() },
        }
    }

    /// Sets up the GIC by configuring the required sensitivites for the shared peripheral
    /// interrupts.
    ///
    /// With a few exeception, the GIC expects software to set up the sensitivities
    /// to fixed values. The only exceptions are the interupts coming from the programmable
    /// logic. These are configured to high level sensitivity by this function.
    /// If you need a different sensitivity, you need to update the bits using the
    /// [Self::set_pl_interrupt_sensitivity] function.
    #[inline]
    pub fn initialize(&mut self) {
        self.gicd.write_icfr_2_spi(ICFR_2_FIXED_VALUE);
        self.gicd.write_icfr_3_spi(ICFR_3_FIXED_VALUE);
        self.gicd.write_icfr_4_spi(ICFR_4_FIXED_VALUE);
        self.gicd.write_icfr_5_spi(ICFR_5_FIXED_VALUE);
        self.set_priority_mask(0xff);
    }

    /// Set the priority mask for the CPU.
    ///
    /// Only interrupts with a higher priority than the mask will be accepted.
    /// A lower numerical number means a higher priority. This means that the reset value 0x0
    /// will mask all interrupts to the CPU while 0xff will unmask all interrupts.
    ///
    /// Please note that the highest priority mask will NOT be necessarily the number of priority
    /// levels: The IPRn register always sets the priority level number to the upper bits of the
    /// 8-bit bitfield. See p.83 of the ARM GICv1 architecture specification.
    pub fn set_priority_mask(&mut self, mask: u8) {
        self.gicc
            .write_pmr(PriorityRegister::new_with_raw_value(mask as u32));
    }

    /// Set the sensitivity of a the Programmable Logic SPI interrupts.
    ///
    /// These are the only interrupt IDs which are configurable for SPI. They are set
    /// to high-level sensitivity by default by the [Self::initialize] function. You can
    /// use this method to override certain sensitivies.
    #[inline]
    pub fn set_pl_interrupt_sensitivity(
        &mut self,
        pl_int_id: usize,
        sensitivity: SpiSensitivity,
    ) -> Result<(), InvalidPlInterruptId> {
        if pl_int_id >= 16 {
            return Err(InvalidPlInterruptId(pl_int_id));
        }
        match pl_int_id {
            0..=2 => {
                let pos = 26 + (pl_int_id * 2);
                let mask = 0b11 << pos;
                self.gicd
                    .modify_icfr_3_spi(|v| (v & !mask) | ((sensitivity as u32) << pos));
            }
            3..=7 => {
                let pos = pl_int_id * 2;
                let mask = 0b11 << pos;
                self.gicd
                    .modify_icfr_4_spi(|v| (v & !mask) | ((sensitivity as u32) << pos));
            }
            8..=15 => {
                let pos = 8 + (pl_int_id * 2);
                let mask = 0b11 << pos;
                self.gicd
                    .modify_icfr_5_spi(|v| (v & !mask) | ((sensitivity as u32) << pos));
            }
            _ => unreachable!(),
        }
        Ok(())
    }

    /// Set the CPU target(s) for a SPI interrupt.
    ///
    /// See [Self::set_all_spi_interrupt_targets_cpu0] for a utility method to handle all
    /// interrupts with one core.
    #[inline]
    pub fn set_spi_interrupt_cpu_target_flags(
        &mut self,
        spi_int: SpiInterrupt,
        target: TargetCpus,
    ) {
        let spi_int_raw = spi_int as u32;
        let spi_offset_to_0 = spi_int_raw as usize - 32;
        // Unwrap okay, calculated index is always valid.
        self.gicd
            .modify_iptr_spi(spi_offset_to_0 / 4, |mut v| {
                // Every register contains 4 target flags, the modulo extracts the index because
                // counting starts at 32 (32 -> index 0, 33 -> index 1 etc.).
                v.set_targets(spi_offset_to_0 % 4, u2::new(target.bits()));
                v
            })
            .unwrap();
    }

    /// Enable SPI interrupt target for a specific CPU without clearing the other CPU bit if it is
    /// set.
    #[inline]
    pub fn set_spi_interrupt_target_for_cpu(&mut self, spi_int: SpiInterrupt, cpu: Cpu) {
        let bitflag = match cpu {
            Cpu::Cpu0 => TargetCpus::CPU_0,
            Cpu::Cpu1 => TargetCpus::CPU_1,
        };
        let spi_int_raw = spi_int as u32;
        let spi_offset_to_0 = spi_int_raw as usize - 32;
        // Unwrap okay, calculated index is always valid.
        self.gicd
            .modify_iptr_spi(spi_offset_to_0 / 4, |mut v| {
                v.set_targets(
                    spi_offset_to_0 % 4,
                    // Extract the target bits, bitwise OR them with [TargetCpus::CPU_0], and set
                    // them back.
                    u2::new(
                        (TargetCpus::from_bits(v.targets(spi_offset_to_0 % 4).as_u8()).unwrap()
                            | bitflag)
                            .bits(),
                    ),
                );
                v
            })
            .unwrap();
    }

    /// Utility function to set all SGI interrupt targets to CPU0.
    ///
    /// This does not clear interrupt target bits for CPU1, it only activates the interrupts for
    /// CPU 0 as well.
    /// This is useful if only CPU0 is active in a system, or if CPU0 handles most interrupts in
    /// the system.
    #[inline]
    pub fn set_all_spi_interrupt_targets_cpu0(&mut self) {
        for i in 0..0x10 {
            self.gicd
                .modify_iptr_spi(i, |v| {
                    InterruptProcessorTargetRegister::new_with_raw_value(
                        v.raw_value() | TARGETS_ALL_CPU_0_IPTR_VAL.raw_value(),
                    )
                })
                .unwrap();
        }
    }

    /// Enable a specific SGI interrupt.
    #[inline]
    pub fn enable_sgi_interrupt(&mut self, int_id: usize) -> Result<(), InvalidSpiInterruptId> {
        if int_id >= 16 {
            return Err(InvalidSpiInterruptId(int_id));
        }
        unsafe {
            self.gicd
                .modify_iser_unchecked(0, |val| val | (1 << int_id))
        };
        Ok(())
    }

    /// Enable all SGI interrupts.
    #[inline]
    pub fn enable_all_sgi_interrupts(&mut self) {
        // Unwrap okay, index is valid.
        self.gicd
            .modify_iser(0, |mut v| {
                v |= ACTIVATE_ALL_SGIS_MASK_ISER;
                v
            })
            .unwrap();
    }

    /// Enable specific PPI interrupt.
    #[inline]
    pub fn enable_ppi_interrupt(&mut self, ppi_int: PpiInterrupt) {
        // Unwrap okay, index is valid.
        self.gicd
            .modify_iser(0, |v| v | 1 << (ppi_int as u32))
            .unwrap();
    }

    /// Enable all PPI interrupts.
    #[inline]
    pub fn enable_all_ppi_interrupts(&mut self) {
        unsafe {
            self.gicd.modify_iser_unchecked(0, |mut v| {
                v |= ACTIVATE_ALL_PPIS_MASK_ISER;
                v
            })
        };
    }

    /// Enable specific SPI interrupt.
    #[inline]
    pub fn enable_spi_interrupt(&mut self, spi_int: SpiInterrupt) {
        let spi_int_raw = spi_int as u32;
        match spi_int_raw {
            32..=63 => {
                let bit_pos = spi_int_raw - 32;
                // Unwrap okay, valid index.
                self.gicd.modify_iser(1, |v| v | (1 << bit_pos)).unwrap();
            }
            64..=92 => {
                let bit_pos = spi_int_raw - 64;
                // Unwrap okay, valid index.
                self.gicd.modify_iser(2, |v| v | (1 << bit_pos)).unwrap();
            }
            _ => unreachable!(),
        }
    }

    /// Enable all SPI interrupts.
    #[inline]
    pub fn enable_all_spi_interrupts(&mut self) {
        self.gicd.write_iser(1, 0xFFFF_FFFF).unwrap();
        self.gicd.write_iser(2, 0xFFFF_FFFF).unwrap();
    }

    /// Enables all interrupts by calling [Self::enable_all_sgi_interrupts],
    /// [Self::enable_all_ppi_interrupts] and [Self::enable_all_spi_interrupts].
    pub fn enable_all_interrupts(&mut self) {
        self.enable_all_sgi_interrupts();
        self.enable_all_ppi_interrupts();
        self.enable_all_spi_interrupts();
    }

    /// Enable the GIC assuming a possibly non-secure configuration.
    ///
    /// This function will NOT configure and enable the various interrupt sources. You need to
    /// set the interrupt sensitivities and targets before calling this function.
    /// This function configured the control registers with the following settings:
    ///
    /// - CPU interface: Secure and non-secure interrupts are enabled. SBPR, FIQen and AckCtrl
    ///   fields set to default value 0.
    /// - Distributor interface: Both non-secure and secure interrupt distribution enabled.
    ///
    /// It calls [Self::update_ctrl_regs] to update the control registers.
    /// If you need custom settings, you can call [Self::update_ctrl_regs] with your required
    /// settings.
    ///
    /// This will not enable the interrupt exception for the Cortex-AR core. You might also have
    /// to call [Self::enable_interrupts] for interrupts to work.
    pub fn enable(&mut self) {
        self.update_ctrl_regs(
            InterfaceControl::builder()
                .with_sbpr(false)
                .with_fiq_en(false)
                .with_ack_ctrl(false)
                .with_enable_non_secure(true)
                .with_enable_secure(true)
                .build(),
            DistributorControlRegister::builder()
                .with_enable_non_secure(true)
                .with_enable_secure(true)
                .build(),
        );
    }

    /// Enable the regular interrupt exceprion for the Cortex-A core by calling the
    /// [interrupt::enable] function. You also need to [Self::enable] the GIC for interrupts to
    /// work.
    ///
    /// # Safety
    ///
    /// Do not call this in a critical section.
    pub unsafe fn enable_interrupts(&self) {
        unsafe {
            interrupt::enable();
        }
    }

    /// Enable the interrupts for the Cortex-A core by calling the [interrupt::enable] module.
    pub fn disable_interrupts(&self) {
        interrupt::disable();
    }

    /// Update the control registers which control the safety configuration and which also enable
    /// the GIC.
    pub fn update_ctrl_regs(&mut self, icr: InterfaceControl, dcr: DistributorControlRegister) {
        self.gicc.write_icr(icr);
        self.gicd.write_dcr(dcr);
    }
}

/// Helper structure which should only be used inside the interrupt handler once the GIC has
/// been configured with the [GicConfigurator].
pub struct GicInterruptHelper(MmioCpuInterfaceRegisters<'static>);

impl GicInterruptHelper {
    /// Create the interrupt helper with the fixed GICC MMIO instance.
    pub const fn new() -> Self {
        GicInterruptHelper(unsafe { CpuInterfaceRegisters::new_mmio_fixed() })
    }

    /// Acknowledges an interrupt by reading the IAR register and returning the interrupt context
    /// information structure.
    ///
    /// This should be called at the start of an interrupt handler.
    pub fn acknowledge_interrupt(&mut self) -> InterruptInfo {
        let iar = self.0.read_iar();
        let int_id = iar.ack_int_id().as_u32();
        let interrupt = match int_id {
            0..=15 => Interrupt::Sgi(int_id as usize),
            27..=31 => Interrupt::Ppi(PpiInterrupt::try_from(int_id as u8).unwrap()),
            32..=92 => Interrupt::Spi(SpiInterrupt::try_from(int_id as u8).unwrap()),
            SPURIOUS_INTERRUPT_ID => Interrupt::Spurious,
            _ => Interrupt::Invalid(int_id as usize),
        };
        InterruptInfo {
            interrupt,
            cpu_id: iar.cpu_id().as_u8(),
            raw_reg: iar,
        }
    }

    /// Acknowledges the end of an interrupt by writing the EOIR register of the GICC.
    ///
    /// This should be called at the end of an interrupt handler.
    pub fn end_of_interrupt(&mut self, irq_info: InterruptInfo) {
        self.0.write_eoir(irq_info.raw_reg())
    }
}

impl Default for GicInterruptHelper {
    fn default() -> Self {
        Self::new()
    }
}
