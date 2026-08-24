pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u2, u3};

    /// Selects the internal reference voltage level for the memory standard in use.
    #[bitbybit::bitenum(u4, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum VRefSel {
        /// VREF = 0.6 V
        Lpddr2 = 0b0001,
        /// VREF = 0.675 V
        Ddr3l = 0b0010,
        /// VREF = 0.75 V
        Ddr3 = 0b0100,
        /// VREF = 0.9 V
        Ddr2 = 0b1000,
    }

    /// DDR IO reference voltage control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DdrControl {
        /// Enables VRP/VRN.
        #[bit(9, rw)]
        refio_enable: bool,
        /// Enables the external reference voltage for the upper data bits.
        #[bit(6, rw)]
        vref_ext_en_upper_bits: bool,
        /// Enables the external reference voltage for the lower data bits.
        #[bit(5, rw)]
        vref_ext_en_lower_bits: bool,
        /// Selects the internal reference voltage for the memory standard in use.
        #[bits(1..=4, rw)]
        vref_sel: Option<VRefSel>,
        /// Enables the internal reference voltage generator.
        #[bit(0, rw)]
        vref_int_en: bool,
    }

    /// DCI (Digitally Controlled Impedance) calibration control register.
    #[bitbybit::bitfield(
        u32,
        default = 0x00,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DciControl {
        /// Controls periodic update of the DCI calibration.
        #[bit(20, rw)]
        update_control: bool,
        /// PDRV impedance calibration reference select, option 2.
        #[bits(17..=19, rw)]
        pref_opt2: u3,
        /// PDRV impedance calibration reference select, option 1.
        #[bits(14..=15, rw)]
        pref_opt1: u2,
        /// NDRV impedance calibration reference select, option 4.
        #[bits(11..=13, rw)]
        nref_opt4: u3,
        /// NDRV impedance calibration reference select, option 2.
        #[bits(8..=10, rw)]
        nref_opt2: u3,
        /// NDRV impedance calibration reference select, option 1.
        #[bits(6..=7, rw)]
        nref_opt1: u2,
        /// Enables the DCI calibration logic.
        #[bit(1, rw)]
        enable: bool,
        /// Reset value 0. Should be toggled once to initialize flops in DCI system.
        #[bit(0, rw)]
        reset: bool,
    }

    /// DCI calibration status register.
    #[bitbybit::bitfield(
        u32,
        default = 0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DciStatus {
        /// DCI calibration sequence has completed.
        #[bit(13, rw)]
        done: bool,
        /// DCI calibration loop has locked.
        #[bit(0, rw)]
        lock: bool,
    }

    /// Selects which buffer direction is active for a DDR IO pin.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum OutputEnable {
        /// Input buffer is active.
        IBuf = 0b00,
        /// Reserved.
        __Reserved0 = 0b01,
        /// Reserved.
        __Reserved1 = 0b10,
        /// Output buffer is active.
        OBuf = 0b11,
    }

    /// Selects the input receiver type for a DDR IO pin.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum InputType {
        /// Input receiver is disabled.
        Off = 0b00,
        /// VREF based differential receiver for SSTL or HSTL signaling.
        VRefBasedDifferentialReceiverForSstlHstl = 0b01,
        /// True differential input receiver, used e.g. for LVDS.
        DifferentialInputReceiver = 0b10,
        /// Single ended LVCMOS receiver.
        LvcmosReceiver = 0b11,
    }

    /// Selects the DCI (impedance controlled) mode for a DDR IO pin.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DciType {
        /// DCI is disabled.
        Disabled = 0b00,
        /// DCI controls the output drive impedance.
        DciDrive = 0b01,
        /// Reserved.
        __Reserved = 0b10,
        /// DCI controls the on-die termination impedance.
        DciTermination = 0b11,
    }

    /// Per-pin-group IO buffer configuration for a DDR interface signal group.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct DdriobConfig {
        /// Enables the internal pull-up resistor.
        #[bit(11, rw)]
        pullup_enable: bool,
        /// Selects the active buffer direction.
        #[bits(9..=10, rw)]
        output_enable: OutputEnable,
        /// Selects the termination disable mode.
        #[bit(8, rw)]
        term_disable_mode: bool,
        /// Selects the input buffer disable mode.
        #[bit(7, rw)]
        ibuf_disable_mode: bool,
        /// Selects the DCI mode for this pin group.
        #[bits(5..=6, rw)]
        dci_type: DciType,
        /// Enables on-die termination.
        #[bit(4, rw)]
        termination_enable: bool,
        /// Enables DCI calibration updates for this pin group.
        #[bit(3, rw)]
        dci_update_enable: bool,
        /// Selects the input receiver type.
        #[bits(1..=2, rw)]
        inp_type: InputType,
    }
}

/// SLCR DDR IO buffer configuration registers.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct DdrIobRegisters {
    /// IO buffer configuration for DDR address/control pins, group 0.
    ddriob_addr0: DdriobConfig,
    /// IO buffer configuration for DDR address/control pins, group 1.
    ddriob_addr1: DdriobConfig,
    /// IO buffer configuration for DDR data pins, group 0.
    ddriob_data0: DdriobConfig,
    /// IO buffer configuration for DDR data pins, group 1.
    ddriob_data1: DdriobConfig,
    /// IO buffer configuration for DDR differential (DQS) pins, group 0.
    ddriob_diff0: DdriobConfig,
    /// IO buffer configuration for DDR differential (DQS) pins, group 1.
    ddriob_diff1: DdriobConfig,
    /// IO buffer configuration for the DDR clock pins.
    ddriob_clock: DdriobConfig,
    /// Drive strength and slew rate control for DDR address/control pins.
    ddriob_drive_slew_addr: u32,
    /// Drive strength and slew rate control for DDR data pins.
    ddriob_drive_slew_data: u32,
    /// Drive strength and slew rate control for DDR differential pins.
    ddriob_drive_slew_diff: u32,
    /// Drive strength and slew rate control for the DDR clock pins.
    ddriob_drive_slew_clock: u32,
    /// DDR IO reference voltage control register.
    ddr_control: DdrControl,
    /// DCI calibration control register.
    dci_control: DciControl,
    /// DCI calibration status register.
    dci_status: DciStatus,
}

impl DdrIobRegisters {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioDdrIobRegisters<'static> {
        unsafe { Self::new_mmio_at(super::SLCR_BASE_ADDR + super::DDRIOB_OFFSET) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<DdrIobRegisters>(), 0x38);
