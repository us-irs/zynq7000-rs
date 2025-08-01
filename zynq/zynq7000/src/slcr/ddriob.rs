use arbitrary_int::{u2, u3};

#[bitbybit::bitenum(u4, exhaustive = false)]
#[derive(Debug)]
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

#[bitbybit::bitfield(u32, debug)]
pub struct DdrControl {
    /// Enables VRP/VRN.
    #[bit(9, rw)]
    refio_enable: bool,
    #[bit(6, rw)]
    vref_ext_en_upper_bits: bool,
    #[bit(5, rw)]
    vref_ext_en_lower_bits: bool,
    #[bits(1..=4, rw)]
    vref_sel: Option<VRefSel>,
    #[bit(0, rw)]
    vref_int_en: bool,
}

#[bitbybit::bitfield(u32, default = 0x00, debug)]
pub struct DciControl {
    #[bit(20, rw)]
    update_control: bool,
    #[bits(17..=19, rw)]
    pref_opt2: u3,
    #[bits(14..=15, rw)]
    pref_opt1: u2,
    #[bits(11..=13, rw)]
    nref_opt4: u3,
    #[bits(8..=10, rw)]
    nref_opt2: u3,
    #[bits(6..=7, rw)]
    nref_opt1: u2,
    #[bit(1, rw)]
    enable: bool,
    /// Reset value 0. Should be toggled once to initialize flops in DCI system.
    #[bit(0, rw)]
    reset: bool,
}

#[bitbybit::bitfield(u32, debug)]
pub struct DciStatus {
    #[bit(13, rw)]
    done: bool,
    #[bit(0, rw)]
    lock: bool,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug)]
pub enum OutputEnable {
    IBuf = 0b00,
    __Reserved0 = 0b01,
    __Reserved1 = 0b10,
    OBuf = 0b11,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug)]
pub enum InputType {
    Off = 0b00,
    VRefBasedDifferentialReceiverForSstlHstl = 0b01,
    DifferentialInputReceiver = 0b10,
    LvcmosReceiver = 0b11,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug)]
pub enum DciType {
    Disabled = 0b00,
    DciDrive = 0b01,
    __Reserved = 0b10,
    DciTermination = 0b11,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct DdriobConfig {
    #[bit(11, rw)]
    pullup_enable: bool,
    #[bits(9..=10, rw)]
    output_enable: OutputEnable,
    #[bit(8, rw)]
    term_disable_mode: bool,
    #[bit(7, rw)]
    ibuf_disable_mode: bool,
    #[bits(5..=6, rw)]
    dci_type: DciType,
    #[bit(4, rw)]
    termination_enable: bool,
    #[bit(3, rw)]
    dci_update_enable: bool,
    #[bits(1..=2, rw)]
    inp_type: InputType,
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct DdrIoB {
    ddriob_addr0: DdriobConfig,
    ddriob_addr1: DdriobConfig,
    ddriob_data0: DdriobConfig,
    ddriob_data1: DdriobConfig,
    ddriob_diff0: DdriobConfig,
    ddriob_diff1: DdriobConfig,
    ddriob_clock: DdriobConfig,
    ddriob_drive_slew_addr: u32,
    ddriob_drive_slew_data: u32,
    ddriob_drive_slew_diff: u32,
    ddriob_drive_slew_clock: u32,
    ddr_ctrl: DdrControl,
    dci_ctrl: DciControl,
    dci_status: DciStatus,
}

impl DdrIoB {
    /// Create a new handle to this peripheral.
    ///
    /// Writing to this register requires unlocking the SLCR registers first.
    ///
    /// # Safety
    ///
    /// If you create multiple instances of this handle at the same time, you are responsible for
    /// ensuring that there are no read-modify-write races on any of the registers.
    pub unsafe fn new_mmio_fixed() -> MmioDdrIoB<'static> {
        unsafe { Self::new_mmio_at(super::SLCR_BASE_ADDR + super::DDRIOB_OFFSET) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<DdrIoB>(), 0x38);
