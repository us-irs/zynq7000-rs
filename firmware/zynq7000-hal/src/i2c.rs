//! # I2C module
use arbitrary_int::{u2, u3, u6};
use embedded_hal::i2c::{NoAcknowledgeSource, Operation};
use zynq7000::{
    i2c::{
        Address, Control, Direction, Fifo, I2C_0_BASE_ADDR, I2C_1_BASE_ADDR, InterruptStatus,
        MmioRegisters, Mode, TransferSize,
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

pub use crate::clock_calc::i2c::{
    ClockConfig, Speed, SpeedNotAttainable, calculate_divisors, calculate_i2c_speed,
};

#[derive(Debug, thiserror::Error)]
pub enum Error {
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
    /// The Zynq-7000 I2C controller (Cadence r1p10) can not do a repeated start after a read.
    /// With HOLD set, it does not signal completion of a master receive. The Linux driver
    /// rejects these transfers too:
    /// <https://github.com/torvalds/linux/blob/v6.12/drivers/i2c/busses/i2c-cadence.c#L869-L883>
    #[error("write after read in one transaction is not supported")]
    WriteAfterRead,
}

#[derive(Debug, thiserror::Error)]
#[error("invalid I2C ID")]
pub struct InvalidPsI2cError;

#[derive(Debug, thiserror::Error)]
pub enum ConstructionError {
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
    ) -> Result<Self, ConstructionError> {
        if i2c.id().is_none() {
            return Err(InvalidPsI2cError.into());
        }
        if Sck::ID != Sda::ID {
            return Err(ConstructionError::PinInvalidForI2cId);
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
        regs.write_control(
            Control::builder()
                .with_div_a(u2::new(clk_cfg.div_a()))
                .with_div_b(u6::new(clk_cfg.div_b()))
                .with_clear_fifo(true)
                .with_slv_mon(false)
                .with_hold_bus(false)
                .with_acken(false)
                .with_addressing(true)
                .with_mode(Mode::Master)
                .with_dir(zynq7000::i2c::Direction::Transmitter)
                .build(),
        );
        Self { regs }
    }

    /// Execute the provided operations on the I2C bus.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an acknowledge for the last byte.
    ///
    /// Unlike the `embedded-hal` contract, a write after a read is rejected with
    /// [Error::WriteAfterRead]. The controller can not continue a transaction with a write after
    /// a read. Adjacent reads are limited to [MAX_READ_SIZE] bytes in total.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0 to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    pub fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Error> {
        if operations.is_empty() {
            return Ok(());
        }
        // Checked up front so an invalid transaction does not leave the bus held by an
        // earlier group.
        check_operations(operations)?;

        self.regs.modify_control(|cr| {
            cr.with_acken(true)
                .with_clear_fifo(true)
                .with_slv_mon(false)
                .with_mode(Mode::Master)
        });
        let ops_len = operations.len();

        // Empty reads are skipped entirely. The hardware can not do a zero-length read, and
        // skipping them in all lookups below ensures they never split or end a merged transfer.
        let mut prev_dir = None;
        for i in 0..ops_len {
            if is_empty_read(&operations[i]) {
                continue;
            }
            // Consecutive operations with the same direction are merged into one group, which
            // is a single hardware transfer. The address is only written for the first operation
            // of a group:
            //  - ST = Start condition
            //  - SR = Repeated start
            //  - SP = Stop condition
            //
            // [W] -> ST + SP on W
            // [W, R] -> ST on W, SR + SP on R
            // [W, W] -> ST on first W, SP on last W
            //
            // HOLD keeps the bus between groups, so the next group starts with an SR. Clearing
            // HOLD in the last group makes the controller send the SP.
            let dir = op_direction(&operations[i]);
            let next_dir = operations[i + 1..]
                .iter()
                .find(|op| !is_empty_read(op))
                .map(op_direction);

            // Either the first operation or a direction change.
            let first_op_in_group = prev_dir != Some(dir);
            // Only used for the first operation of a group. The look ahead below adds the lengths
            // of the other operations in the group.
            let mut group_len = op_len(&operations[i]);
            let more_ops_in_group = next_dir == Some(dir);

            if first_op_in_group {
                // Clears flags left over from the previous group or transaction, most importantly
                // COMP. Clearing per group is safe because errors abort the whole transaction.
                self.regs.modify_interrupt_status(|isr| isr);
                // Add the lengths of the following operations in the same group.
                for next_op in operations[i + 1..].iter().filter(|op| !is_empty_read(op)) {
                    if op_direction(next_op) != dir {
                        break;
                    }

                    group_len += op_len(next_op);
                }
                // A read group which fits into the FIFO can finish before `read_internal` clears
                // HOLD. The controller then never signals completion, so HOLD is only set for
                // larger read groups. Write groups always set it, `write_internal` clears it for
                // the last group.
                self.regs.modify_control(|cr| {
                    cr.with_dir(dir)
                        .with_hold_bus(dir == Direction::Transmitter || group_len > FIFO_DEPTH)
                        .with_clear_fifo(true)
                });
                // Only read groups need a transfer size. The TRM sets it after the direction.
                if dir == Direction::Receiver {
                    self.regs
                        .write_transfer_size(TransferSize::new_with_raw_value(group_len as u32));
                }
            }
            let last_group = next_dir.is_none();
            match &mut operations[i] {
                Operation::Read(items) => {
                    self.read_internal(address, items, first_op_in_group, more_ops_in_group)?
                }
                Operation::Write(items) => self.write_internal(
                    address,
                    items,
                    first_op_in_group,
                    more_ops_in_group,
                    last_group,
                )?,
            }
            prev_dir = Some(dir);
        }
        Ok(())
    }

    /// Reads enough bytes from slave with `address` to fill `read`.
    ///
    /// # I2C Events (contract)
    ///
    /// ``` text
    /// Master: ST SAD+R        MAK    MAK ...    NMAK SP
    /// Slave:           SAK B0     B1     ... BN
    /// ```
    ///
    /// Where
    ///
    /// - `ST` = start condition
    /// - `SAD+R` = slave address followed by bit 1 to indicate reading
    /// - `SAK` = slave acknowledge
    /// - `Bi` = ith byte of data
    /// - `MAK` = master acknowledge
    /// - `NMAK` = master no acknowledge
    /// - `SP` = stop condition
    pub fn read(&mut self, addr: u8, data: &mut [u8]) -> Result<(), Error> {
        <Self as embedded_hal::i2c::I2c>::read(self, addr, data)
    }

    /// Writes bytes to slave with address `address`.
    ///
    /// # I2C Events (contract)
    ///
    /// ``` text
    /// Master: ST SAD+W     B0     B1     ... BN     SP
    /// Slave:           SAK    SAK    SAK ...    SAK
    /// ```
    ///
    /// Where
    ///
    /// - `ST` = start condition
    /// - `SAD+W` = slave address followed by bit 0 to indicate writing
    /// - `SAK` = slave acknowledge
    /// - `Bi` = ith byte of data
    /// - `SP` = stop condition
    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error> {
        <Self as embedded_hal::i2c::I2c>::write(self, addr, data)
    }

    /// Writes bytes to slave with address `address` and then reads enough bytes to fill `read` *in a
    /// single transaction*.
    ///
    /// # I2C Events (contract)
    ///
    /// ``` text
    /// Master: ST SAD+W     O0     O1     ... OM     SR SAD+R        MAK    MAK ...    NMAK SP
    /// Slave:           SAK    SAK    SAK ...    SAK          SAK I0     I1     ... IN
    /// ```
    ///
    /// Where
    ///
    /// - `ST` = start condition
    /// - `SAD+W` = slave address followed by bit 0 to indicate writing
    /// - `SAK` = slave acknowledge
    /// - `Oi` = ith outgoing byte of data
    /// - `SR` = repeated start condition
    /// - `SAD+R` = slave address followed by bit 1 to indicate reading
    /// - `Ii` = ith incoming byte of data
    /// - `MAK` = master acknowledge
    /// - `NMAK` = master no acknowledge
    /// - `SP` = stop condition
    pub fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        <Self as embedded_hal::i2c::I2c>::write_read(self, address, write, read)
    }

    fn write_internal(
        &mut self,
        addr: u8,
        data: &[u8],
        first_op_in_group: bool,
        more_ops_in_group: bool,
        last_group: bool,
    ) -> Result<(), Error> {
        let mut addr_set = false;
        let mut written = 0;
        loop {
            let busy = self.regs.read_status().tx_busy();
            let used = if busy {
                core::cmp::min(
                    self.regs.read_transfer_size().size() as usize + 1,
                    FIFO_DEPTH,
                )
            } else {
                0
            };
            let free = FIFO_DEPTH - used;
            let pushed = core::cmp::min(free, data.len() - written);
            (0..pushed).for_each(|_| {
                self.regs
                    .write_data(Fifo::new_with_raw_value(data[written] as u32));
                written += 1;
            });
            if !addr_set && first_op_in_group {
                self.start_transfer(addr);
                addr_set = true;
            }
            let isr = self.regs.read_interrupt_status();
            self.check_and_handle_tx_errors(isr, first_op_in_group, written)?;
            // We have written all bytes of the current buffer.
            if written == data.len() {
                // More operations in the same group, so return immediately so we can write from
                // the next buffer.
                if more_ops_in_group {
                    return Ok(());
                }
                // Clear HOLD only after the last byte was pushed. If it were cleared earlier and
                // the FIFO ran empty, the controller would send the STOP too early.
                if last_group {
                    self.clear_hold_bit();
                }
                break;
            }
        }
        // Poll to completion.
        while !self.regs.read_interrupt_status().complete() {
            let isr = self.regs.read_interrupt_status();
            // For writes which fit into the FIFO, an address NACK usually shows up here.
            self.check_and_handle_tx_errors(isr, first_op_in_group, written)?;
        }
        // COMP is also set when the FIFO ran empty while HOLD was still set, so it does not
        // mean the STOP is on the bus yet.
        if last_group {
            while self.regs.read_status().bus_active() {
                let isr = self.regs.read_interrupt_status();
                self.check_and_handle_tx_errors(isr, first_op_in_group, written)?;
            }
        }
        Ok(())
    }

    fn check_and_handle_tx_errors(
        &mut self,
        isr: InterruptStatus,
        address_nack_possible: bool,
        written: usize,
    ) -> Result<(), Error> {
        if isr.tx_overflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(Error::TxOverflow);
        }
        if isr.arbitration_lost() {
            self.clean_up_after_transfer_or_on_error();
            return Err(Error::ArbitrationLoss);
        }
        if isr.nack() {
            // The hardware has no address NACK flag. After a NACK, the transfer size register
            // holds the number of unsent bytes minus one. If all bytes written so far are still
            // unsent, the target NACKed its address. Write groups are always the first group,
            // so this only works for the first operation of the transaction.
            //
            // Limitations:
            // - Later operations in the group always report a data NACK.
            // - A NACK on the first data byte might look like an address NACK. The TRM does not
            //   say whether the NACKed byte counts as sent.
            //
            // The register must be read before the cleanup, because clearing the FIFO resets it.
            let unsent = self.regs.read_transfer_size().size() as usize + 1;
            self.clean_up_after_transfer_or_on_error();
            let err = if address_nack_possible && (written == 0 || unsent == written) {
                Error::Nack(NoAcknowledgeSource::Address)
            } else {
                Error::Nack(NoAcknowledgeSource::Data)
            };

            return Err(err);
        }
        if isr.timeout() {
            // Timeout / Stall condition.
            self.clean_up_after_transfer_or_on_error();
            return Err(Error::Timeout);
        }
        Ok(())
    }

    pub fn clean_up_after_transfer_or_on_error(&mut self) {
        self.regs.modify_control(|cr| {
            cr.with_acken(false)
                .with_clear_fifo(true)
                .with_hold_bus(false)
        });
    }

    // Size has to be set by caller for each read group.
    fn read_internal(
        &mut self,
        addr: u8,
        data: &mut [u8],
        first_op_in_group: bool,
        more_ops_in_group: bool,
    ) -> Result<(), Error> {
        // Internal API, this should never happen, empty reads do not make sense.
        assert!(!data.is_empty());
        let mut read = 0;
        if first_op_in_group {
            self.start_transfer(addr);
        }
        loop {
            let mut status = self.regs.read_status();
            loop {
                let isr = self.regs.read_interrupt_status();
                self.check_and_handle_rx_errors(isr)?;
                if status.rx_valid() {
                    break;
                }
                // Re-read for next check.
                status = self.regs.read_status();
            }
            // Data to be read.
            while self.regs.read_status().rx_valid() && read < data.len() {
                data[read] = self.regs.read_data().data();
                read += 1;
            }
            // The outstanding read size fits into the FIFO. Clear the HOLD bit as specified in
            // TRM p.649 polled read step 6.
            if self.regs.read_transfer_size().size() as usize <= FIFO_DEPTH {
                self.clear_hold_bit();
            }
            // We have read everything into the current buffer.
            if read == data.len() {
                // More operations in the same group, so return immediately so we can read into
                // the next buffer.
                if more_ops_in_group {
                    return Ok(());
                }
                // Otherwise, we can poll to completion.
                break;
            }
        }

        // Poll to completion.
        while !self.regs.read_interrupt_status().complete() {
            let isr = self.regs.read_interrupt_status();
            self.check_and_handle_rx_errors(isr)?
        }
        // Read groups are always last. Wait until the STOP is on the bus.
        while self.regs.read_status().bus_active() {
            let isr = self.regs.read_interrupt_status();
            self.check_and_handle_rx_errors(isr)?;
        }
        self.clean_up_after_transfer_or_on_error();
        Ok(())
    }

    fn check_and_handle_rx_errors(&mut self, isr: InterruptStatus) -> Result<(), Error> {
        if isr.rx_overflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(Error::RxOverflow);
        }
        if isr.rx_underflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(Error::RxUnderflow);
        }
        if isr.nack() {
            self.clean_up_after_transfer_or_on_error();
            // For reception, the slave only ACKs/NAKs the address.
            return Err(Error::Nack(NoAcknowledgeSource::Address));
        }
        if isr.timeout() {
            // Timeout / Stall condition.
            self.clean_up_after_transfer_or_on_error();
            return Err(Error::Timeout);
        }
        Ok(())
    }

    /// Start the transfer by writing the I2C address.
    #[inline]
    fn start_transfer(&mut self, address: u8) {
        self.regs
            .write_addr(Address::new_with_raw_value(address as u32));
    }

    #[inline]
    pub fn set_hold_bit(&mut self) {
        self.regs.modify_control(|mut cr| {
            cr.set_hold_bus(true);
            cr
        });
    }

    #[inline]
    pub fn clear_hold_bit(&mut self) {
        self.regs.modify_control(|mut cr| {
            cr.set_hold_bus(false);
            cr
        });
    }
}

fn check_operations(operations: &[Operation<'_>]) -> Result<(), Error> {
    let mut prev_dir = None;
    let mut group_len = 0;
    for op in operations.iter().filter(|op| !is_empty_read(op)) {
        let dir = op_direction(op);
        if prev_dir == Some(Direction::Receiver) && dir == Direction::Transmitter {
            return Err(Error::WriteAfterRead);
        }
        if prev_dir != Some(dir) {
            group_len = 0;
        }
        group_len += op_len(op);
        if group_len > MAX_READ_SIZE && dir == Direction::Receiver {
            return Err(Error::ReadDataLenTooLarge);
        }
        prev_dir = Some(dir);
    }
    Ok(())
}

fn op_direction(op: &Operation<'_>) -> Direction {
    match op {
        Operation::Read(_) => Direction::Receiver,
        Operation::Write(_) => Direction::Transmitter,
    }
}

fn op_len(op: &Operation<'_>) -> usize {
    match op {
        Operation::Read(items) => items.len(),
        Operation::Write(items) => items.len(),
    }
}

fn is_empty_read(op: &Operation<'_>) -> bool {
    matches!(op, Operation::Read(buf) if buf.is_empty())
}

impl embedded_hal::i2c::ErrorType for I2c {
    type Error = Error;
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::ArbitrationLoss => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Error::Nack(nack_kind) => embedded_hal::i2c::ErrorKind::NoAcknowledge(*nack_kind),
            Error::RxOverflow => embedded_hal::i2c::ErrorKind::Overrun,
            Error::TxOverflow => embedded_hal::i2c::ErrorKind::Other,
            Error::RxUnderflow => embedded_hal::i2c::ErrorKind::Other,
            Error::Timeout | Error::ReadDataLenTooLarge | Error::WriteAfterRead => {
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
        self.transaction(address, operations)
    }
}

/// Reset the I2C peripheral using the SLCR reset register for I2C.
///
/// Please note that this function will interfere with an already configured
/// I2C instance.
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
