//! # I2C module
use arbitrary_int::{u2, u3, u6};
use embedded_hal::i2c::NoAcknowledgeSource;
use zynq7000::{
    i2c::{
        Control, I2C_0_BASE_ADDR, I2C_1_BASE_ADDR, InterruptStatus, MmioRegisters, TransferSize,
    },
    slcr::reset::DualClockReset,
};

#[cfg(not(feature = "7z010-7z007s-clg225"))]
use crate::gpio::mio::{
    Mio16, Mio17, Mio18, Mio19, Mio20, Mio21, Mio22, Mio23, Mio24, Mio25, Mio26, Mio27, Mio40,
    Mio41, Mio42, Mio43, Mio44, Mio45, Mio46, Mio47, Mio50, Mio51,
};
use crate::{
    enable_amba_peripheral_clock,
    gpio::{
        IoPeriphPin,
        mio::{
            Mio10, Mio11, Mio12, Mio13, Mio14, Mio15, Mio28, Mio29, Mio30, Mio31, Mio32, Mio33,
            Mio34, Mio35, Mio36, Mio37, Mio38, Mio39, Mio48, Mio49, Mio52, Mio53, MioPin,
            MuxConfig, Pin,
        },
    },
    slcr::Slcr,
    time::Hertz,
};

pub const I2C_MUX_CONF: MuxConfig = MuxConfig::new_with_l3(u3::new(0b010));
pub const FIFO_DEPTH: usize = 16;
/// Maximum read size in one read operation.
pub const MAX_READ_SIZE: usize = 255;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cId {
    I2c0 = 0,
    I2c1 = 1,
}

pub trait PsI2c {
    fn reg_block(&self) -> MmioRegisters<'static>;
    fn id(&self) -> Option<I2cId>;
}

impl PsI2c for MmioRegisters<'static> {
    #[inline]
    fn reg_block(&self) -> MmioRegisters<'static> {
        unsafe { self.clone() }
    }

    #[inline]
    fn id(&self) -> Option<I2cId> {
        let base_addr = unsafe { self.ptr() } as usize;
        if base_addr == I2C_0_BASE_ADDR {
            return Some(I2cId::I2c0);
        } else if base_addr == I2C_1_BASE_ADDR {
            return Some(I2cId::I2c1);
        }
        None
    }
}

pub trait SdaPin: MioPin {
    const ID: I2cId;
}

pub trait SckPin: MioPin {
    const ID: I2cId;
}

pub trait I2cPins {}

macro_rules! i2c_pin_impls {
    ($Id: path, $SckMio:ident, $SdaMio:ident) => {
        impl SckPin for Pin<$SckMio> {
            const ID: I2cId = $Id;
        }

        impl SdaPin for Pin<$SdaMio> {
            const ID: I2cId = $Id;
        }

        impl I2cPins for (Pin<$SckMio>, Pin<$SdaMio>) {}
    };
}

/*
macro_rules! into_i2c {
    ($($Mio:ident),+) => {
        $(
            impl <M: PinMode> MioPin<$Mio, M> {
                /// Convert the pin into I2C pins by configuring the pin routing via the
                /// MIO multiplexer bits. Also enables pull-ups for the pins.
                pub fn into_i2c(self) -> MioPin<$Mio, IoPeriph> {
                    // Enable pull-ups for the I2C pins.
                    self.into_io_periph(I2C_MUX_CONF, Some(true))
                }
            }
        )+
    };
}

into_i2c!(
    Mio10, Mio11, Mio14, Mio15, Mio30, Mio31, Mio34, Mio35, Mio38, Mio39, Mio12, Mio13, Mio28,
    Mio29, Mio32, Mio33, Mio36, Mio37, Mio48, Mio49, Mio52, Mio53
);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
into_i2c!(
    Mio18, Mio19, Mio22, Mio23, Mio26, Mio27, Mio42, Mio43, Mio46, Mio47, Mio50, Mio51, Mio16,
    Mio17, Mio20, Mio21, Mio24, Mio25, Mio40, Mio41, Mio44, Mio45
);
*/

i2c_pin_impls!(I2cId::I2c0, Mio10, Mio11);
i2c_pin_impls!(I2cId::I2c0, Mio14, Mio15);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c0, Mio18, Mio19);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c0, Mio22, Mio23);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c0, Mio26, Mio27);
i2c_pin_impls!(I2cId::I2c0, Mio30, Mio31);
i2c_pin_impls!(I2cId::I2c0, Mio34, Mio35);
i2c_pin_impls!(I2cId::I2c0, Mio38, Mio39);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c0, Mio42, Mio43);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c0, Mio46, Mio47);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c0, Mio50, Mio51);

i2c_pin_impls!(I2cId::I2c1, Mio12, Mio13);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c1, Mio16, Mio17);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c1, Mio20, Mio21);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c1, Mio24, Mio25);
i2c_pin_impls!(I2cId::I2c1, Mio28, Mio29);
i2c_pin_impls!(I2cId::I2c1, Mio32, Mio33);
i2c_pin_impls!(I2cId::I2c1, Mio36, Mio37);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c1, Mio40, Mio41);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
i2c_pin_impls!(I2cId::I2c1, Mio44, Mio45);
i2c_pin_impls!(I2cId::I2c1, Mio48, Mio49);
i2c_pin_impls!(I2cId::I2c1, Mio52, Mio53);

#[derive(Debug, Clone, Copy)]
pub enum I2cSpeed {
    Normal100kHz,
    HighSpeed400KHz,
}

impl I2cSpeed {
    pub fn frequency_full_number(&self) -> Hertz {
        Hertz::from_raw(match self {
            I2cSpeed::Normal100kHz => 100_000,
            I2cSpeed::HighSpeed400KHz => 400_000,
        })
    }

    /// From Xilinx embeddedsw
    /// If frequency 400KHz is selected, 384.6KHz should be set.
    /// If frequency 100KHz is selected, 90KHz should be set.
    /// This is due to a hardware limitation.
    pub fn frequency_for_calculation(&self) -> Hertz {
        Hertz::from_raw(match self {
            I2cSpeed::Normal100kHz => 90_000,
            I2cSpeed::HighSpeed400KHz => 384_600,
        })
    }
}

#[derive(Debug, thiserror::Error)]
#[error("I2C speed not attainable")]
pub struct I2cSpeedNotAttainable;

#[derive(Debug, thiserror::Error)]
pub enum I2cTxError {
    #[error("arbitration lost")]
    ArbitrationLoss,
    #[error("transfer not acknowledged: {0}")]
    Nack(NoAcknowledgeSource),
    #[error("TX overflow")]
    TxOverflow,
    #[error("timeout of transfer")]
    Timeout,
}

#[derive(Debug, thiserror::Error)]
pub enum I2cRxError {
    #[error("arbitration lost")]
    ArbitrationLoss,
    #[error("transfer not acknowledged")]
    Nack(NoAcknowledgeSource),
    #[error("RX underflow")]
    RxUnderflow,
    #[error("RX overflow")]
    RxOverflow,
    #[error("timeout of transfer")]
    Timeout,
    #[error("read data exceeds maximum allowed 255 bytes per transfer")]
    ReadDataLenTooLarge,
}

#[derive(Debug, thiserror::Error)]
pub enum I2cError {
    #[error("arbitration lost")]
    ArbitrationLoss,
    #[error("transfer not acknowledged: {0}")]
    Nack(NoAcknowledgeSource),
    #[error("TX overflow")]
    TxOverflow,
    #[error("RX underflow")]
    RxUnderflow,
    #[error("RX overflow")]
    RxOverflow,
    #[error("timeout of transfer")]
    Timeout,
    #[error("read data exceeds maximum allowed 255 bytes per transfer")]
    ReadDataLenTooLarge,
}

impl From<I2cRxError> for I2cError {
    fn from(err: I2cRxError) -> Self {
        match err {
            I2cRxError::ArbitrationLoss => I2cError::ArbitrationLoss,
            I2cRxError::Nack(nack) => I2cError::Nack(nack),
            I2cRxError::RxUnderflow => I2cError::RxUnderflow,
            I2cRxError::RxOverflow => I2cError::RxOverflow,
            I2cRxError::Timeout => I2cError::Timeout,
            I2cRxError::ReadDataLenTooLarge => I2cError::ReadDataLenTooLarge,
        }
    }
}

impl From<I2cTxError> for I2cError {
    fn from(err: I2cTxError) -> Self {
        match err {
            I2cTxError::ArbitrationLoss => I2cError::ArbitrationLoss,
            I2cTxError::Nack(nack) => I2cError::Nack(nack),
            I2cTxError::TxOverflow => I2cError::TxOverflow,
            I2cTxError::Timeout => I2cError::Timeout,
        }
    }
}

#[inline]
pub fn calculate_i2c_speed(cpu_1x_clk: Hertz, clk_config: ClockConfig) -> Hertz {
    cpu_1x_clk / (22 * (clk_config.div_a as u32 + 1) * (clk_config.div_b as u32 + 1))
}

pub fn calculate_divisors(
    cpu_1x_clk: Hertz,
    speed: I2cSpeed,
) -> Result<ClockConfig, I2cSpeedNotAttainable> {
    let target_speed = speed.frequency_for_calculation();
    if cpu_1x_clk > 22 * 64 * 4 * target_speed {
        return Err(I2cSpeedNotAttainable);
    }
    let mut smallest_deviation = u32::MAX;
    let mut best_div_a = 1;
    let mut best_div_b = 1;
    for divisor_a in 1..=4 {
        for divisor_b in 1..=64 {
            let i2c_clock = cpu_1x_clk / (22 * divisor_a * divisor_b);
            let deviation = (target_speed.raw() as i32 - i2c_clock.raw() as i32).unsigned_abs();
            if deviation < smallest_deviation {
                smallest_deviation = deviation;
                best_div_a = divisor_a;
                best_div_b = divisor_b;
            }
        }
    }
    Ok(ClockConfig::new(best_div_a as u8 - 1, best_div_b as u8 - 1))
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct ClockConfig {
    div_a: u8,
    div_b: u8,
}

impl ClockConfig {
    pub fn new(div_a: u8, div_b: u8) -> Self {
        Self { div_a, div_b }
    }

    pub fn div_a(&self) -> u8 {
        self.div_a
    }

    pub fn div_b(&self) -> u8 {
        self.div_b
    }
}

#[derive(Debug, thiserror::Error)]
#[error("invalid I2C ID")]
pub struct InvalidPsI2cError;

#[derive(Debug, thiserror::Error)]
pub enum I2cConstructionError {
    #[error("invalid I2C ID {0}")]
    InvalidPsI2c(#[from] InvalidPsI2cError),
    #[error("pin invalid for I2C ID")]
    PinInvalidForI2cId,
    #[error("invalid pin configuration for I2C")]
    InvalidPinConf,
}
pub struct I2c {
    regs: MmioRegisters<'static>,
}

impl I2c {
    pub fn new_with_mio<Sck: SckPin, Sda: SdaPin>(
        i2c: impl PsI2c,
        clk_cfg: ClockConfig,
        i2c_pins: (Sck, Sda),
    ) -> Result<Self, I2cConstructionError> {
        if i2c.id().is_none() {
            return Err(InvalidPsI2cError.into());
        }
        if Sck::ID != Sda::ID {
            return Err(I2cConstructionError::PinInvalidForI2cId);
        }
        IoPeriphPin::new(i2c_pins.0, I2C_MUX_CONF, Some(true));
        IoPeriphPin::new(i2c_pins.1, I2C_MUX_CONF, Some(true));
        Ok(Self::new_generic(
            i2c.id().unwrap(),
            i2c.reg_block(),
            clk_cfg,
        ))
    }

    pub fn new_with_emio(i2c: impl PsI2c, clk_cfg: ClockConfig) -> Result<Self, InvalidPsI2cError> {
        if i2c.id().is_none() {
            return Err(InvalidPsI2cError);
        }
        Ok(Self::new_generic(
            i2c.id().unwrap(),
            i2c.reg_block(),
            clk_cfg,
        ))
    }

    pub fn new_generic(id: I2cId, mut regs: MmioRegisters<'static>, clk_cfg: ClockConfig) -> Self {
        let periph_sel = match id {
            I2cId::I2c0 => crate::PeriphSelect::I2c0,
            I2cId::I2c1 => crate::PeriphSelect::I2c1,
        };
        enable_amba_peripheral_clock(periph_sel);
        //reset(id);
        regs.write_cr(
            Control::builder()
                .with_div_a(u2::new(clk_cfg.div_a()))
                .with_div_b(u6::new(clk_cfg.div_b()))
                .with_clear_fifo(true)
                .with_slv_mon(false)
                .with_hold_bus(false)
                .with_acken(false)
                .with_addressing(true)
                .with_mode(zynq7000::i2c::Mode::Master)
                .with_dir(zynq7000::i2c::Direction::Transmitter)
                .build(),
        );
        Self { regs }
    }

    /// Start the transfer by writing the I2C address.
    #[inline]
    fn start_transfer(&mut self, address: u8) {
        self.regs
            .write_addr(zynq7000::i2c::Address::new_with_raw_value(address as u32));
    }

    #[inline]
    pub fn set_hold_bit(&mut self) {
        self.regs.modify_cr(|mut cr| {
            cr.set_hold_bus(true);
            cr
        });
    }

    #[inline]
    pub fn clear_hold_bit(&mut self) {
        self.regs.modify_cr(|mut cr| {
            cr.set_hold_bus(false);
            cr
        });
    }

    pub fn write_transfer_blocking(
        &mut self,
        addr: u8,
        data: &[u8],
        generate_stop: bool,
    ) -> Result<(), I2cTxError> {
        self.regs.modify_cr(|mut cr| {
            cr.set_acken(true);
            cr.set_mode(zynq7000::i2c::Mode::Master);
            cr.set_clear_fifo(true);
            cr.set_dir(zynq7000::i2c::Direction::Transmitter);
            if !generate_stop {
                cr.set_hold_bus(true);
            }
            cr
        });
        let mut first_write_cycle = true;
        let mut addr_set = false;
        let mut written = 0;
        // Clear the interrupt status register before using it to monitor the transfer.
        self.regs.modify_isr(|isr| isr);
        loop {
            let bytes_to_write = core::cmp::min(
                FIFO_DEPTH - self.regs.read_transfer_size().size() as usize,
                data.len() - written,
            );
            (0..bytes_to_write).for_each(|_| {
                self.regs
                    .write_data(zynq7000::i2c::Fifo::new_with_raw_value(
                        data[written] as u32,
                    ));
                written += 1;
            });
            if !addr_set {
                self.start_transfer(addr);
                addr_set = true;
            }
            let mut status = self.regs.read_sr();
            // While the hardware is busy sending out data, we poll for errors.
            while status.tx_busy() {
                let isr = self.regs.read_isr();
                self.check_and_handle_tx_errors(isr, first_write_cycle, bytes_to_write)?;
                // Re-read for next check.
                status = self.regs.read_sr();
            }
            first_write_cycle = false;
            // Just need to poll to completion now.
            if written == data.len() {
                break;
            }
        }
        // Poll to completion.
        while !self.regs.read_isr().complete() {
            let isr = self.regs.read_isr();
            self.check_and_handle_tx_errors(isr, first_write_cycle, data.len())?;
        }
        if generate_stop {
            self.clear_hold_bit();
        }
        Ok(())
    }

    fn check_and_handle_tx_errors(
        &mut self,
        isr: InterruptStatus,
        first_write_cycle: bool,
        first_chunk_len: usize,
    ) -> Result<(), I2cTxError> {
        if isr.tx_overflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cTxError::TxOverflow);
        }
        if isr.arbitration_lost() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cTxError::ArbitrationLoss);
        }
        if isr.nack() {
            self.clean_up_after_transfer_or_on_error();
            // I have no tested this yet, but if no data was sent yet, this is probably
            // an address NACK.
            if first_write_cycle
                && self.regs.read_transfer_size().size() as usize + 1 == first_chunk_len
            {
                return Err(I2cTxError::Nack(NoAcknowledgeSource::Address));
            } else {
                return Err(I2cTxError::Nack(NoAcknowledgeSource::Data));
            }
        }
        if isr.timeout() {
            // Timeout / Stall condition.
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cTxError::Timeout);
        }
        Ok(())
    }

    pub fn clean_up_after_transfer_or_on_error(&mut self) {
        self.regs.modify_cr(|mut cr| {
            cr.set_acken(false);
            cr.set_clear_fifo(true);
            cr
        });
    }

    pub fn read_transfer_blocking(&mut self, addr: u8, data: &mut [u8]) -> Result<(), I2cRxError> {
        self.regs.modify_cr(|mut cr| {
            cr.set_acken(true);
            cr.set_mode(zynq7000::i2c::Mode::Master);
            cr.set_clear_fifo(true);
            cr.set_dir(zynq7000::i2c::Direction::Receiver);
            if data.len() > FIFO_DEPTH {
                cr.set_hold_bus(true);
            }
            cr
        });
        let mut read = 0;
        if data.len() > MAX_READ_SIZE {
            return Err(I2cRxError::ReadDataLenTooLarge);
        }
        // Clear the interrupt status register before using it to monitor the transfer.
        self.regs.modify_isr(|isr| isr);
        self.regs
            .write_transfer_size(TransferSize::new_with_raw_value(data.len() as u32));
        self.start_transfer(addr);
        loop {
            let mut status = self.regs.read_sr();
            loop {
                let isr = self.regs.read_isr();
                self.check_and_handle_rx_errors(read, isr)?;
                if status.rx_valid() {
                    break;
                }
                // Re-read for next check.
                status = self.regs.read_sr();
            }
            // Data to be read.
            while self.regs.read_sr().rx_valid() {
                data[read] = self.regs.read_data().data();
                read += 1;
            }
            // The outstading read size is smaller than the FIFO. Clear the HOLD register as
            // specified in TRM p.649 polled read step 6.
            if self.regs.read_transfer_size().size() as usize <= FIFO_DEPTH {
                self.clear_hold_bit();
            }
            // Read everything, just need to poll to completion now.
            if read == data.len() {
                break;
            }
        }

        // Poll to completion.
        while !self.regs.read_isr().complete() {
            let isr = self.regs.read_isr();
            self.check_and_handle_rx_errors(read, isr)?
        }
        self.clear_hold_bit();
        self.clean_up_after_transfer_or_on_error();
        Ok(())
    }

    fn check_and_handle_rx_errors(
        &mut self,
        read_count: usize,
        isr: InterruptStatus,
    ) -> Result<(), I2cRxError> {
        if isr.rx_overflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cRxError::RxOverflow);
        }
        if isr.rx_underflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cRxError::RxUnderflow);
        }
        if isr.nack() {
            self.clean_up_after_transfer_or_on_error();
            // I have no tested this yet, but if no data was sent yet, this is probably
            // an address NACK.
            if read_count == 0 {
                return Err(I2cRxError::Nack(NoAcknowledgeSource::Address));
            } else {
                return Err(I2cRxError::Nack(NoAcknowledgeSource::Data));
            }
        }
        if isr.timeout() {
            // Timeout / Stall condition.
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cRxError::Timeout);
        }
        Ok(())
    }
}

impl embedded_hal::i2c::ErrorType for I2c {
    type Error = I2cError;
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            I2cError::ArbitrationLoss => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            I2cError::Nack(nack_kind) => embedded_hal::i2c::ErrorKind::NoAcknowledge(*nack_kind),
            I2cError::RxOverflow => embedded_hal::i2c::ErrorKind::Overrun,
            I2cError::TxOverflow => embedded_hal::i2c::ErrorKind::Other,
            I2cError::RxUnderflow => embedded_hal::i2c::ErrorKind::Other,
            I2cError::Timeout | I2cError::ReadDataLenTooLarge => {
                embedded_hal::i2c::ErrorKind::Other
            }
        }
    }
}

impl embedded_hal::i2c::I2c for I2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                embedded_hal::i2c::Operation::Read(items) => {
                    self.read_transfer_blocking(address, items)?
                }
                embedded_hal::i2c::Operation::Write(items) => {
                    self.write_transfer_blocking(address, items, true)?
                }
            }
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        // I have never tested this, so I am not sure whether the master still generates a stop
        // condition somehow.. which might break the trait contract.
        self.write_transfer_blocking(address, write, false)?;
        Ok(self.read_transfer_blocking(address, read)?)
    }
}

/// Reset the SPI peripheral using the SLCR reset register for SPI.
///
/// Please note that this function will interfere with an already configured
/// SPI instance.
#[inline]
pub fn reset(id: I2cId) {
    let assert_reset = match id {
        I2cId::I2c0 => DualClockReset::builder()
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        I2cId::I2c1 => DualClockReset::builder()
            .with_periph1_cpu1x_rst(true)
            .with_periph0_cpu1x_rst(false)
            .build(),
    };
    unsafe {
        Slcr::with(|regs| {
            regs.reset_ctrl().write_i2c(assert_reset);
            // Keep it in reset for some cycles.. The TMR just mentions some small delay,
            // no idea what is meant with that.
            for _ in 0..3 {
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_i2c(DualClockReset::DEFAULT);
        });
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use fugit::RateExtU32;
    use std::println;

    #[test]
    fn example_test() {
        let clk_cfg = calculate_divisors(111.MHz(), I2cSpeed::Normal100kHz).unwrap();
        assert_eq!(clk_cfg.div_a(), 0);
        assert_eq!(clk_cfg.div_b(), 55);
        let speed = calculate_i2c_speed(111.MHz(), clk_cfg);
        assert!(speed.raw() < 100_000);
        assert!(speed.raw() > 85_000);
    }

    #[test]
    fn example_test_2() {
        let clk_cfg = calculate_divisors(111.MHz(), I2cSpeed::HighSpeed400KHz).unwrap();
        assert_eq!(clk_cfg.div_a(), 0);
        assert_eq!(clk_cfg.div_b(), 12);
        let speed = calculate_i2c_speed(111.MHz(), clk_cfg);
        assert!(speed.raw() < 400_000);
        assert!(speed.raw() > 360_000);
    }

    #[test]
    fn example_test_3() {
        let clk_cfg = calculate_divisors(133.MHz(), I2cSpeed::Normal100kHz).unwrap();
        assert_eq!(clk_cfg.div_a(), 1);
        assert_eq!(clk_cfg.div_b(), 33);
        let speed = calculate_i2c_speed(133.MHz(), clk_cfg);
        assert!(speed.raw() < 100_000);
        assert!(speed.raw() > 85_000);
    }

    #[test]
    fn example_test_4() {
        let clk_cfg = calculate_divisors(133.MHz(), I2cSpeed::HighSpeed400KHz).unwrap();
        assert_eq!(clk_cfg.div_a(), 0);
        assert_eq!(clk_cfg.div_b(), 15);
        let speed = calculate_i2c_speed(133.MHz(), clk_cfg);
        assert!(speed.raw() < 400_000);
        assert!(speed.raw() > 360_000);
    }
}
