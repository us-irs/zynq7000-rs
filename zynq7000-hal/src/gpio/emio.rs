//! EMIO (Extended Multiplexed I/O) resource management module.
use zynq7000::gpio::MmioGpio;

pub use crate::gpio::PinState;

pub struct EmioPin {
    offset: usize,
}

impl EmioPin {
    /// This offset ranges from 0 to 64.
    pub fn offset(&self) -> usize {
        self.offset
    }
}

pub struct Pins {
    emios: [Option<EmioPin>; 64],
}

impl Pins {
    /// Create a new EMIO pin structure.
    ///
    /// This structure is supposed to be used as a singleton. It will configure all
    /// EMIO pins as inputs. If you want to retrieve individual pins without this structure,
    /// use [EmioPin::steal] instead.
    pub fn new(mut mmio: MmioGpio) -> Self {
        let mut emios = [const { None }; 64];
        // Configure all EMIO pins as inputs.
        mmio.bank_2().write_dirm(0);
        mmio.bank_3().write_dirm(0);

        (0..64).for_each(|i| {
            emios[i] = Some(EmioPin { offset: i });
        });
        Self { emios }
    }

    pub fn take(&mut self, offset: usize) -> Option<EmioPin> {
        self.emios[offset].take()
    }

    pub fn give(&mut self, emio: EmioPin) {
        self.emios[emio.offset].replace(emio);
    }
}
