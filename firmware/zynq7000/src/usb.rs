//! # USB register module

/// Base address of instance 0.
pub const BASE_ADDR_0: usize = 0xE000_2000;
/// Base address of instance 1.
pub const BASE_ADDR_1: usize = 0xE000_3000;

pub use types::*;

/// Register helper types.
pub mod types {
    use arbitrary_int::{u2, u3, u4, u5, u6, u7, u9, u12, u14, u21, u24};

    /// IP version and revision, hardwired by the IP supplier.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct Id {
        /// Reserved, reads 0b111.
        #[bits(29..=31, r)]
        civersion: u3,
        /// IP version. 0x2 refers to IP version 2.20a together with revision and tag.
        #[bits(25..=28, r)]
        version: u4,
        /// IP revision, refer to [Self::version].
        #[bits(21..=24, r)]
        revision: u4,
        /// IP tag, refer to [Self::version].
        #[bits(16..=20, r)]
        tag: u5,
        /// Controller ID, ones complement of [Self::id].
        #[bits(8..=13, r)]
        nid: u6,
        /// Controller ID. 0x5 means the controller supports HS, On-The-Go and FS/LS.
        #[bits(0..=5, r)]
        id: u6,
    }

    /// Misc IP config constants, hardwired by the IP supplier. Bits \[31:12\] are reserved.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct HwGeneral {
        /// PHY serial interface constant. 0 selects a parallel I/O port interface.
        #[bits(10..=11, r)]
        sm: u2,
        /// PHY type constant. 0b0010 selects an 8-bit ULPI single data rate interface.
        #[bits(6..=9, r)]
        phym: u4,
        /// PHY data bus width constant. 0 selects an 8-bit data bus.
        #[bits(4..=5, r)]
        phyw: u2,
        /// Clock configuration constant. 1 means CPU_1x must run at a higher frequency
        /// than the 60 MHz UTMI clock.
        #[bits(1..=2, r)]
        clkc: u2,
        /// Reset type constant. 1 means an asynchronous reset.
        #[bit(0, r)]
        rt: bool,
    }

    /// Host mode IP config constants, hardwired by the IP supplier.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct HwHost {
        /// Number of periodic contexts in the embedded TT.
        #[bits(24..=31, r)]
        ttper: u8,
        /// Number of asynchronous contexts in the embedded TT.
        #[bits(16..=23, r)]
        ttasy: u8,
        /// Number of downstream ports supported. 0 means one port.
        #[bits(1..=3, r)]
        nport: u3,
        /// Host mode is supported.
        #[bit(0, r)]
        hc: bool,
    }

    /// Device mode IP config constants, hardwired by the IP supplier.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct HwDevice {
        /// Number of endpoints supported in device mode, including EP0.
        #[bits(1..=5, r)]
        devep: u5,
        /// Device mode is supported.
        #[bit(0, r)]
        dc: bool,
    }

    /// TX buffer IP config constants, hardwired by the IP supplier. Bit 31 is reserved.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct HwTxBuf {
        /// Number of address bits used for each 64-byte endpoint TX buffer.
        #[bits(16..=23, r)]
        txchanadd: u8,
        /// TX buffer address width. 0xA means a 10-bit address and a 768-byte buffer.
        #[bits(8..=15, r)]
        txadd: u8,
        /// Burst length used on the AHB bus by the DMA engine, in bytes.
        #[bits(0..=7, r)]
        txburst: u8,
    }

    /// RX buffer IP config constants, hardwired by the IP supplier.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct HwRxBuf {
        /// RX buffer address width. 0xA means a 10-bit address and a 1024-byte buffer.
        #[bits(8..=15, r)]
        rxadd: u8,
        /// Burst length used on the AHB bus by the DMA engine, in bytes.
        #[bits(0..=7, r)]
        rxburst: u8,
    }

    /// General purpose timer load value, shared layout for both GP timers.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct GpTimerLoad {
        /// Timer duration in microseconds minus 1, loaded into the counter on reset.
        /// Maximum value is 0xFF_FFFF (16.777215 seconds).
        #[bits(0..=23, rw)]
        value: u24,
    }

    /// Countdown behavior of a general purpose timer.
    #[bitbybit::bitenum(u1, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum GpTimerMode {
        /// The timer counts down to zero, generates an interrupt and stops.
        OneShot = 0,
        /// The timer counts down to zero, generates an interrupt and reloads
        /// automatically.
        Repeat = 1,
    }

    /// General purpose timer control, shared layout for both GP timers.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct GpTimerControl {
        /// Enables the timer. Has no effect on the current counter value.
        #[bit(31, rw)]
        run: bool,
        /// Reloads the counter from the load value. Write 1 to reload.
        #[bit(30, w)]
        reset: bool,
        /// Countdown mode.
        #[bit(24, rw)]
        mode: GpTimerMode,
        /// Running countdown value.
        #[bits(0..=23, rw)]
        count: u24,
    }

    /// DMA master AHB burst mode configuration.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct SbusConfig {
        /// AHB burst size, using the standard AHB burst type encoding. The reset value
        /// 0x3 selects INCR16, with non-multiple transfers of INCR16 decomposed into
        /// INCR8, INCR4 and single transfers.
        #[bits(0..=2, rw)]
        ahbbrst: u3,
    }

    /// EHCI capability register address space size and HCI version, read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct CapLengthHciVersion {
        /// Host controller interface version, BCD encoded.
        #[bits(16..=31, r)]
        hciversion: u16,
        /// Address space taken by the capability registers. Add this offset to the
        /// address of the first capability register to get the address of the first
        /// operational register.
        #[bits(0..=15, r)]
        caplength: u16,
    }

    /// EHCI host controller structural parameters, read-only. This implementation is
    /// confined to a single host port.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct HcsParams {
        /// Number of Transaction Translators. 0 means none, referring to the embedded TT.
        #[bits(24..=27, r)]
        n_tt: u4,
        /// Number of ports per Transaction Translator. 0 means a single host port.
        #[bits(20..=23, r)]
        n_ptt: u4,
        /// Port indicator control is available via EMIO, see [super::PortStatusControl].
        #[bit(16, r)]
        pi: bool,
        /// Companion controller hardware. 0 means none, refer to the embedded TT instead.
        #[bits(12..=15, r)]
        n_cc: u4,
        /// Ports supported by each companion controller. 0 means none, refer to the
        /// embedded TT instead.
        #[bits(8..=11, r)]
        n_pcc: u4,
        /// VBUS power control is available via EMIO, see [super::PortStatusControl].
        #[bit(4, r)]
        ppc: bool,
        /// Number of downstream ports. 1 means a single downstream port.
        #[bits(0..=3, r)]
        n_ports: u4,
    }

    /// EHCI host controller capability parameters, read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct HccParams {
        /// EHCI extended capabilities pointer.
        #[bits(8..=15, r)]
        eecp: u8,
        /// Isochronous scheduling threshold, relative to the current position of the
        /// executing host controller, at which software may reliably update the periodic
        /// schedule.
        #[bits(4..=7, r)]
        ist: u4,
        /// Asynchronous schedule park mode is supported.
        #[bit(2, r)]
        asp: bool,
        /// Programmable frame list sizes are supported in host mode, configured using
        /// [super::UsbCommand].
        #[bit(1, r)]
        pfl: bool,
        /// 0 means 32-bit system memory addressing.
        #[bit(0, r)]
        adc: bool,
    }

    /// Device controller interface version, BCD encoded, read-only. The upper byte is
    /// the major revision and the lower byte the minor revision.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct DciVersion {
        /// BCD-encoded version.
        #[bits(0..=15, r)]
        dciversion: u16,
    }

    /// Host and device mode capability, read-only.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct DccParams {
        /// The controller supports EHCI compatible host mode.
        #[bit(8, r)]
        hc: bool,
        /// The controller supports device mode.
        #[bit(7, r)]
        dc: bool,
        /// Number of endpoints supported in device mode: control EP0 plus EP {11:1}.
        #[bits(0..=4, r)]
        den: u5,
    }

    /// Maximum rate at which the host controller issues an interrupt.
    #[bitbybit::bitenum(u8, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum InterruptThreshold {
        /// Interrupt immediately.
        Immediate = 0x00,
        /// One micro-frame.
        One = 0x01,
        /// Two micro-frames.
        Two = 0x02,
        /// Four micro-frames.
        Four = 0x04,
        /// Eight micro-frames, equivalent to 1 ms.
        Eight = 0x08,
        /// Sixteen micro-frames.
        Sixteen = 0x10,
        /// Thirty-two micro-frames.
        ThirtyTwo = 0x20,
        /// Sixty-four micro-frames.
        SixtyFour = 0x40,
    }

    /// USB commands executed by the host/device controller.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct UsbCommand {
        /// Maximum interrupt rate, see [InterruptThreshold].
        #[bits(16..=23, rw)]
        itc: Option<InterruptThreshold>,
        /// MSB of the frame list size, refer to [Self::fs01].
        #[bit(15, rw)]
        fs2: bool,
        /// Add dTD tripwire, used as a semaphore to ensure the proper addition of a new
        /// dTD to an active endpoint's linked list. Host extended.
        #[bit(14, rw)]
        atdtw: bool,
        /// Setup tripwire, used between the device controller driver and the hardware to
        /// extract setup data from a queue head without corruption. Device mode.
        #[bit(13, rw)]
        sutw: bool,
        /// Asynchronous schedule park mode enable. Only has an effect if
        /// [HccParams::asp] is set, otherwise park mode is disabled.
        #[bit(11, rw)]
        aspe: bool,
        /// Asynchronous schedule park mode count, between 1 and 3. 3 gives the maximum
        /// throughput in endpoint transactions compared to 1 or 2.
        #[bits(8..=9, rw)]
        asp: u2,
        /// Light host/device controller reset. Not supported, always reads 0.
        #[bit(7, r)]
        lr: bool,
        /// Interrupt on async schedule advance doorbell. Write 1 to ring the doorbell
        /// when the controller advances the asynchronous schedule.
        #[bit(6, rw)]
        iaa: bool,
        /// Asynchronous schedule enable. Host mode.
        #[bit(5, rw)]
        ase: bool,
        /// Periodic schedule enable. Host mode.
        #[bit(4, rw)]
        pse: bool,
        /// LSBs of the frame list size, refer to [Self::fs2]. 0b00 selects 1024 elements,
        /// 0b111 selects 8 elements.
        #[bits(2..=3, rw)]
        fs01: u2,
        /// Controller reset.
        #[bit(1, rw)]
        rst: bool,
        /// Run/Stop. Device mode: 0 halts activity after the current packet transfer
        /// completes, 1 resumes execution of the periodic and async schedules.
        #[bit(0, rw)]
        rs: bool,
    }

    /// USB bus and port interrupt status, controller state, and general purpose timer
    /// status. Applies to both host and device mode unless noted otherwise.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct UsbStatus {
        /// GP timer 1 raw interrupt. Write 1 to clear.
        #[bit(25, rw)]
        ti1: bool,
        /// GP timer 0 raw interrupt. Write 1 to clear.
        #[bit(24, rw)]
        ti0: bool,
        /// Host periodic raw interrupt, set when a periodic TD completes with IOC set.
        /// Host mode. Write 1 to clear.
        #[bit(19, rw)]
        up: bool,
        /// Host async raw interrupt, set when an async TD completes with IOC set. Host
        /// mode. Write 1 to clear.
        #[bit(18, rw)]
        ua: bool,
        /// Set when the endpoint sends a NAK handshake and the NAK bit is set. Device
        /// mode, read-only.
        #[bit(16, r)]
        nak: bool,
        /// Async schedule is enabled, mirrors [UsbCommand::ase] once hardware has enabled
        /// processing. Host mode, read-only.
        #[bit(15, r)]
        async_status: bool,
        /// Periodic schedule is enabled, mirrors [UsbCommand::pse] once hardware has
        /// enabled processing. Host mode, read-only.
        #[bit(14, r)]
        ps: bool,
        /// Reclamation status. 0 means unprocessed async transactions, 1 means an empty
        /// async schedule. Host mode, read-only.
        #[bit(13, r)]
        rcl: bool,
        /// The controller halted after the Run/Stop bit was cleared, either by software
        /// or by the controller hardware. Host mode, read-only.
        #[bit(12, r)]
        hch: bool,
        /// ULPI event completed. Write 1 to clear.
        #[bit(10, rw)]
        ulpi: bool,
        /// DCSuspend, set when the controller enters a suspend state from an active
        /// state. Device mode. Write 1 to clear.
        #[bit(8, rw)]
        sle: bool,
        /// Start-of-frame detected. Write 1 to clear.
        #[bit(7, rw)]
        sr: bool,
        /// USB reset detected on the ULPI bus. Device mode. Write 1 to clear.
        #[bit(6, rw)]
        ur: bool,
        /// Async schedule advance, primed by the async advance doorbell bit in
        /// [UsbCommand::iaa]. Host mode. Write 1 to clear.
        #[bit(5, rw)]
        aa: bool,
        /// AHB interconnect system error. Write 1 to clear.
        #[bit(4, rw)]
        sei: bool,
        /// Frame list rolled over to element 0. Write 1 to clear.
        #[bit(3, rw)]
        fre: bool,
        /// Port change detected, refer to [super::PortStatusControl] for the individual
        /// change bits. Write 1 to clear.
        #[bit(2, rw)]
        pc: bool,
        /// A USB transaction completed with an error. Write 1 to clear.
        #[bit(1, rw)]
        ue: bool,
        /// A transaction descriptor finished with the interrupt-on-complete bit set, or a
        /// short packet was detected. Write 1 to clear.
        #[bit(0, rw)]
        ui: bool,
    }

    /// Interrupt enables. An interrupt is generated when a bit here is set and the
    /// corresponding condition in [UsbStatus] is active. [UsbStatus] still reflects
    /// interrupt sources even if they are disabled here, allowing polling.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_bitfields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct UsbInterruptEnable {
        /// Enables [UsbStatus::ti1].
        #[bit(25, rw)]
        ti1: bool,
        /// Enables [UsbStatus::ti0].
        #[bit(24, rw)]
        ti0: bool,
        /// Enables [UsbStatus::up]. Host mode.
        #[bit(19, rw)]
        up: bool,
        /// Enables [UsbStatus::ua]. Host mode.
        #[bit(18, rw)]
        ua: bool,
        /// Enables [UsbStatus::nak]. Device mode.
        #[bit(16, rw)]
        nak: bool,
        /// Enables [UsbStatus::ulpi].
        #[bit(10, rw)]
        ulpi: bool,
        /// Enables [UsbStatus::sle]. Device mode.
        #[bit(8, rw)]
        sle: bool,
        /// Enables [UsbStatus::sr].
        #[bit(7, rw)]
        sr: bool,
        /// Enables [UsbStatus::ur]. Device mode.
        #[bit(6, rw)]
        ur: bool,
        /// Enables [UsbStatus::aa]. Host mode.
        #[bit(5, rw)]
        aa: bool,
        /// Enables [UsbStatus::sei].
        #[bit(4, rw)]
        see: bool,
        /// Enables [UsbStatus::fre].
        #[bit(3, rw)]
        fre: bool,
        /// Enables [UsbStatus::pc].
        #[bit(2, rw)]
        pc: bool,
        /// Enables [UsbStatus::ue].
        #[bit(1, rw)]
        ue: bool,
        /// Enables [UsbStatus::ui].
        #[bit(0, rw)]
        ui: bool,
    }

    /// Periodic frame list index. Updates every 125 us (once each micro-frame). Host
    /// mode: read/write. Device mode: read-only, reflects the frame index from the most
    /// recently received packet.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct FrameIndex {
        /// Frame index value.
        #[bits(0..=13, rw)]
        frindex: u14,
    }

    /// Host mode: periodic list base address (PERIODICLISTBASE). Device mode: device
    /// address advance and address (USBADR).
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct ListBase {
        /// Host mode: periodic list base address bits \[31:25\]. Device mode: 7-bit
        /// device address, combined with the advance semaphore in `perbase_usbadr`. When
        /// that bit is 0, writes here take effect instantaneously. When it is 1 at or
        /// before this field is written, the write is staged and applied only once an IN
        /// on endpoint 0 is ACKed.
        #[bits(25..=31, rw)]
        perbase_usbadra: u7,
        /// Host mode: periodic list base address bit \[24\]. Device mode: address
        /// advance semaphore, refer to `perbase_usbadra`. Hardware clears it
        /// automatically once the staged IN is ACKed, on an OUT/SETUP to endpoint 0, or
        /// on a device reset.
        #[bit(24, rw)]
        perbase_usbadr: bool,
        /// Host mode: periodic list base address bits \[23:12\]. Device mode: reserved.
        #[bits(12..=23, rw)]
        perbase_reserved: u12,
    }

    /// Host mode: asynchronous list base address (ASYNCLISTADDR). Device mode: endpoint
    /// list base address (ENDPOINTLISTADDR).
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct AsyncListAddr {
        /// Host mode: async list base address bits \[31:11\], pointing to the queue
        /// heads (QH). Device mode: endpoint list base address bits \[31:11\], pointing
        /// to the queue heads. The list uses a 16-endpoint stride even though only 12
        /// endpoints are implemented, leaving unused memory locations.
        #[bits(11..=31, rw)]
        asybase_epbase: u21,
        /// Host mode: async list base address bits \[10:5\]. Device mode: reserved.
        #[bits(5..=10, rw)]
        asybase: u6,
    }

    /// Parameters needed for the internal Transaction Translator (TT) operation.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct TtControl {
        /// Hub address matched against the Hub Address field in a QH or siTD to decide
        /// whether a packet is routed to the internal TT for directly attached FS/LS
        /// devices. On a mismatch, the packet is broadcast on the high-speed ports
        /// destined for a downstream high-speed hub at that address.
        #[bits(24..=30, rw)]
        hubaddr: u7,
        /// Clears all pending transactions in the embedded TT asynchronous buffers. The
        /// clear takes as much time as necessary to avoid interfering with a transaction
        /// in progress and self-clears once it has actually occurred.
        #[bit(1, rw)]
        ttas: bool,
        /// Set while one or more transactions are held in the embedded TT asynchronous
        /// buffers. Read-only.
        #[bit(0, r)]
        ttac: bool,
    }

    /// Burst size used during data movement on the initiator/master interface.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct BurstSize {
        /// Maximum TX burst length in 32-bit words, moving data from system memory to
        /// the USB bus. Supported values are 4 to 128. If [SbusConfig::ahbbrst] is
        /// non-zero, the effective burst length instead follows the AHB INCRx length.
        #[bits(8..=16, rw)]
        tx: u9,
        /// Maximum RX burst length in 32-bit words, moving data from the USB bus to
        /// system memory. Supported values are 4 to 128. If [SbusConfig::ahbbrst] is
        /// non-zero, the effective burst length instead follows the AHB INCRx length.
        #[bits(0..=7, rw)]
        rx: u8,
    }

    /// Performance tuning for how the controller posts data to the TX latency FIFO
    /// before moving it onto the USB bus.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct TxFillTuning {
        /// Number of data bursts posted to the TX latency FIFO in host mode before the
        /// packet begins on the bus. Minimum is 2, keep this as low as possible. A higher
        /// value can help in systems with unpredictable latency or bandwidth where the
        /// FIFO might otherwise underrun. Ignored if the stream disable bit in
        /// [super::UsbMode] is set.
        #[bits(16..=21, rw)]
        burst: u6,
        /// Scheduler health counter, incremented whenever the controller fails to fill
        /// the TX latency FIFO to the `burst` threshold before running out of time
        /// to send the packet before the next start-of-frame. Saturates at 31. Writing to
        /// this field clears it.
        #[bits(8..=12, rw)]
        health: u5,
        /// Additional fixed offset added to the schedule time estimator, in units of
        /// 1.267 us (high speed) or 6.333 us (low/full speed). Choose a value that keeps
        /// `health` below roughly 10 events per second on a highly utilized bus, a
        /// too-high value needlessly reduces USB utilization.
        #[bits(0..=6, rw)]
        overhead: u7,
    }

    /// Equivalent of [TxFillTuning] for the embedded TT TX latency FIFO. There is no
    /// burst threshold field because the TT TX latency FIFO is always loaded in a single
    /// burst.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct TtTxFillTuning {
        /// Refer to [TxFillTuning::health].
        #[bits(8..=12, rw)]
        health: u5,
        /// Refer to [TxFillTuning::overhead]. The time unit here is always 6.333 us.
        #[bits(0..=4, rw)]
        overhead: u5,
    }

    /// Voltage supplied to an Inter-Chip USB peripheral through a port. Host mode only,
    /// always 0 in device mode.
    #[bitbybit::bitenum(u3, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum InterChipVoltage {
        /// No voltage.
        None = 0b000,
        /// 1.0 V.
        V1_0 = 0b001,
        /// 1.2 V.
        V1_2 = 0b010,
        /// 1.5 V.
        V1_5 = 0b011,
        /// 1.8 V.
        V1_8 = 0b100,
        /// 3.0 V.
        V3_0 = 0b101,
    }

    /// Enables and controls the Inter-Chip USB FS/LS transceiver. This implementation is
    /// not a multi-port host (MPH), so only port 1 is meaningful, ports 2 to 8 are
    /// hardwired to 0 and read-only.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct IcUsb {
        /// Enables the Inter-Chip transceiver for port 1. To enable the interface, the
        /// PTS bits in [super::PortStatusControl] must also be set to 0b011.
        #[bit(3, rw)]
        ic1: bool,
        /// Voltage supplied to the peripheral through port 1. Host mode only, always 0 in
        /// device mode.
        #[bits(0..=2, rw)]
        ic_vdd1: Option<InterChipVoltage>,
    }

    /// Indirect access to the ULPI PHY register set. The core normally accesses the ULPI
    /// PHY registers on its own, this port exists for cases where software needs direct
    /// access.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct UlpiViewport {
        /// ULPI wake up operation. Write 1 to execute (not undoable), reads 1 while the
        /// operation is in progress. Do not issue a wake up and a viewport transaction in
        /// the same register write.
        #[bit(31, rw)]
        wu: bool,
        /// ULPI viewport transaction. Write 1 to execute (not undoable), reads 1 while
        /// the transaction is in progress. Do not issue a wake up and a viewport
        /// transaction in the same register write.
        #[bit(30, rw)]
        run: bool,
        /// Selects a read (0) or write (1) viewport transaction.
        #[bit(29, rw)]
        rw_select: bool,
        /// ULPI interface is in the normal synchronous state, as opposed to carkit,
        /// serial or low power state. Read-only.
        #[bit(27, r)]
        ss: bool,
        /// Reserved, always write 0.
        #[bits(24..=26, rw)]
        ulpiport: u3,
        /// ULPI register address for the operation.
        #[bits(16..=23, rw)]
        addr: u8,
        /// Result of the most recently completed read operation. Read-only.
        #[bits(8..=15, r)]
        datrd: u8,
        /// Data written by a write operation.
        #[bits(0..=7, rw)]
        datwr: u8,
    }

    /// Generic per-endpoint bitmask, shared by the endpoint NAK, prime, flush, ready and
    /// complete registers. Bit N of each half corresponds to endpoint N. This
    /// implementation supports 12 endpoints (0 to 11), higher bits are reserved.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct EndpointMask {
        /// TX (IN) endpoint bits.
        #[bits(16..=31, rw)]
        tx: u16,
        /// RX (OUT) endpoint bits.
        #[bits(0..=15, rw)]
        rx: u16,
    }

    /// Operating speed negotiated for the port.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum PortSpeed {
        /// Full speed.
        Full = 0b00,
        /// Low speed.
        Low = 0b01,
        /// High speed.
        High = 0b10,
        /// Not connected, the default.
        NotConnected = 0b11,
    }

    /// Raw state of the D+/D- lines, read-only.
    #[bitbybit::bitenum(u2, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum LineState {
        /// SE0.
        Se0 = 0b00,
        /// J-state.
        JState = 0b01,
        /// K-state.
        KState = 0b10,
    }

    /// Port test mode selection.
    #[bitbybit::bitenum(u4, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum PortTestControl {
        /// Normal operation.
        Normal = 0b0000,
        /// Test J_STATE.
        JState = 0b0001,
        /// Test K_STATE.
        KState = 0b0010,
        /// Test SE0 (host) or NAK (device).
        Se0OrNak = 0b0011,
        /// Test packet.
        Packet = 0b0100,
        /// Force high speed enable.
        ForceEnableHs = 0b0101,
        /// Force full speed enable.
        ForceEnableFs = 0b0110,
        /// Force low speed enable.
        ForceEnableLs = 0b0111,
    }

    /// Port indicator LED control outputs, driven on the
    /// EMIOUSBxPORTINDCTL0/EMIOUSBxPORTINDCTL1 signals. Host mode. Refer to the USB
    /// Specification Revision 2.0 for how these bits are used.
    #[bitbybit::bitenum(u2, exhaustive = false)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum PortIndicatorControl {
        /// Port indicators are off.
        Off = 0b00,
        /// Amber.
        Amber = 0b01,
        /// Green.
        Green = 0b10,
    }

    /// Port status and control. This implementation contains only 1 host port.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct PortStatusControl {
        /// PHY type status constant, together with [Self::pts2] as the MSB. 0b010
        /// selects a ULPI interface.
        #[bits(30..=31, rw)]
        pts: u2,
        /// Serial transceiver select constant, always 0 since a serial interface engine
        /// is not implemented. Read-only.
        #[bit(29, r)]
        sts: bool,
        /// Parallel transceiver width constant, always 0 for an 8-bit (60 MHz) UTMI+
        /// interface to ULPI. Read-only.
        #[bit(28, r)]
        ptw: bool,
        /// Port speed, see [PortSpeed].
        #[bits(26..=27, rw)]
        pspd: PortSpeed,
        /// MSB of [Self::pts].
        #[bit(25, rw)]
        pts2: bool,
        /// Forces the port to only connect at full speed by disabling the chirp sequence
        /// that identifies the port as high speed. Useful for testing full speed
        /// configurations with a high speed host, hub or device. Debug only.
        #[bit(24, rw)]
        pfsc: bool,
        /// Disables the PHY clock. Cannot be disabled if it is being used as the system
        /// clock. Device mode: can be set when the device is not running
        /// ([UsbCommand::rs] is 0) or the host has signaled suspend ([Self::susp] is 1),
        /// and is cleared automatically once the host signals resume. Host mode: can be
        /// set when the downstream device is suspended or no downstream device is
        /// connected, entirely under software control.
        #[bit(23, rw)]
        phcd: bool,
        /// Enables the port to be sensitive to over-current conditions as a wakeup
        /// event. Zero if [Self::pp] is 0 or in device mode. Output on the
        /// pwrctl_wake_ovrcurr_en signal for an external power control circuit. Host
        /// mode only.
        #[bit(22, rw)]
        wkoc: bool,
        /// Enables waking on a disconnect event. Host mode, always 0 in device mode.
        #[bit(21, rw)]
        wkds: bool,
        /// Enables waking on a connect event. Host mode, always 0 in device mode.
        #[bit(20, rw)]
        wkcn: bool,
        /// Port test mode, see [PortTestControl].
        #[bits(16..=19, rw)]
        ptc: Option<PortTestControl>,
        /// Port indicator control, see [PortIndicatorControl]. Host mode.
        #[bits(14..=15, rw)]
        pic: Option<PortIndicatorControl>,
        /// Port owner hand-off is not implemented, always 0. Read-only.
        #[bit(13, r)]
        po: bool,
        /// Port power enable, driven on the EMIOUSBxVBUSPWRSELECT signal. Host mode. If
        /// power is unavailable on the port (this bit is 0), the port is non-functional
        /// and will not report attaches, detaches, etc. When an over-current condition
        /// is detected on a powered port and PPC in [HcsParams] is 1, the controller may
        /// clear this bit itself.
        #[bit(12, rw)]
        pp: bool,
        /// Raw line state, see [LineState]. Read-only.
        #[bits(10..=11, rw)]
        ls: Option<LineState>,
        /// High speed port status, redundant with [Self::pspd]. Read-only.
        #[bit(9, r)]
        hsp: bool,
        /// Port reset. Zero if [Self::pp] is 0. Host mode: 1 means the port is in reset.
        /// Device mode: read-only status bit, a device reset from the USB bus is also
        /// indicated in [UsbStatus::ur].
        #[bit(8, rw)]
        pr: bool,
        /// Suspend. Host mode: 1 means the port is suspended. Device mode: read-only
        /// status bit.
        #[bit(7, rw)]
        susp: bool,
        /// Force port resume. Write 1 to drive resume signaling (K-state) on the port.
        #[bit(6, rw)]
        fpr: bool,
        /// Set to 1 on a change to [Self::oca]. Write 1 to clear.
        #[bit(5, rw)]
        occ: bool,
        /// The port currently has an over-current condition. Automatically transitions
        /// back to 0 once the condition is removed. Always 0 in device mode. Read-only.
        #[bit(4, r)]
        oca: bool,
        /// Set to 1 on a port enabled/disabled status change. Host mode only, always 0
        /// in device mode. Write 1 to clear.
        #[bit(3, rw)]
        pec: bool,
        /// Port enabled. Host mode: ports are enabled by the controller itself as part
        /// of reset and enable, software cannot enable a port by writing 1, only disable
        /// it. Device mode: the device port is always enabled and this bit always reads
        /// 1.
        #[bit(2, rw)]
        pe: bool,
        /// Set to 1 on a change to [Self::ccs]. Write 1 to clear. Host mode only,
        /// undefined in device mode.
        #[bit(1, rw)]
        csc: bool,
        /// Current connect status. Host mode: 1 means a device is present. Device mode:
        /// 1 means attached. Read-only.
        #[bit(0, r)]
        ccs: bool,
    }

    /// On-The-Go (OTG) status and control. This implementation provides one OTG status
    /// and control register, grouped into interrupt enables, interrupt status (write 1
    /// to clear), status inputs (read-only) and controls.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct OtgStatusControl {
        /// Enables [Self::dpis].
        #[bit(30, rw)]
        dpie: bool,
        /// Enables [Self::ms_timer_status].
        #[bit(29, rw)]
        ms_timer_ie: bool,
        /// Enables [Self::bseis].
        #[bit(28, rw)]
        bsee: bool,
        /// Enables [Self::bsvis].
        #[bit(27, rw)]
        bsvie: bool,
        /// Enables [Self::asvis].
        #[bit(26, rw)]
        asvie: bool,
        /// Enables [Self::avvis].
        #[bit(25, rw)]
        avvie: bool,
        /// Enables [Self::idis].
        #[bit(24, rw)]
        idie: bool,
        /// Data pulses were detected on DP or DM. Only detected when
        /// [super::UsbMode::cm] selects host mode and [PortStatusControl::pp] is 0.
        /// Non-latched status can be read using [Self::dps]. Write 1 to clear.
        #[bit(22, rw)]
        dpis: bool,
        /// Set by hardware every 1 ms, based on a timer using the 60 MHz ULPI clock.
        /// Write 1 to clear.
        #[bit(21, rw)]
        ms_timer_status: bool,
        /// Set when VBus falls below the B session end threshold. Write 1 to clear.
        #[bit(20, rw)]
        bseis: bool,
        /// Set when VBus rises above or falls below the B session valid threshold
        /// (0.8 VDC). Write 1 to clear.
        #[bit(19, rw)]
        bsvis: bool,
        /// Set when VBus rises above or falls below the B session valid threshold
        /// (0.8 VDC), on an A device. Write 1 to clear.
        #[bit(18, rw)]
        asvis: bool,
        /// Set when VBus rises above or falls below the A VBus valid threshold
        /// (4.4 VDC). Write 1 to clear.
        #[bit(17, rw)]
        avvis: bool,
        /// Set on a change of the ID input. Write 1 to clear.
        #[bit(16, rw)]
        idis: bool,
        /// Data bus pulsing status, refer to [Self::dpis]. Read-only.
        #[bit(14, r)]
        dps: bool,
        /// Toggles high-low every millisecond to signal [Self::ms_timer_status].
        /// Read-only.
        #[bit(13, r)]
        ms_timer_toggle: bool,
        /// VBus is below the B session end threshold. Read-only.
        #[bit(12, r)]
        bse: bool,
        /// VBus is above the B session valid threshold. Read-only.
        #[bit(11, r)]
        bsv: bool,
        /// VBus is above the A session valid threshold. Read-only.
        #[bit(10, r)]
        asv: bool,
        /// VBus is above the A VBus valid threshold. Read-only.
        #[bit(9, r)]
        avv: bool,
        /// USB ID pin state. 0 means an A device, 1 means a B device. Read-only.
        #[bit(8, r)]
        id: bool,
        /// Enables the automatic B-Disconnect to A-Connect sequence.
        #[bit(7, rw)]
        haba: bool,
        /// Starts the hardware assisted data pulsing sequence.
        #[bit(6, rw)]
        hadp: bool,
        /// Controls the ID pullup resistor. Must be enabled for the ID input to be
        /// sampled.
        #[bit(5, rw)]
        idpu: bool,
        /// Asserts a pullup on DP for data pulsing during SRP.
        #[bit(4, rw)]
        dp: bool,
        /// Controls the pulldown on DM. Must be set when the controller is in device
        /// mode.
        #[bit(3, rw)]
        ot: bool,
        /// Enables automatic reset after connect on the host port.
        #[bit(2, rw)]
        haar: bool,
        /// Charges VBus, used for VBus pulsing during SRP.
        #[bit(1, rw)]
        vc: bool,
        /// Discharges VBus through a resistor.
        #[bit(0, rw)]
        vd: bool,
    }

    /// Fixed operating mode of the controller.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ControllerMode {
        /// Idle, the default for a combination host/device controller until initialized.
        Idle = 0b00,
        /// Reserved.
        __Reserved = 0b01,
        /// Device mode, the default for a device-only controller.
        Device = 0b10,
        /// Host mode, the default for a host-only controller.
        Host = 0b11,
    }

    /// USB controller mode selection.
    #[bitbybit::bitfield(
        u32,
        default = 0x0,
        debug,
        defmt_fields(feature = "defmt"),
        forbid_overlaps
    )]
    pub struct UsbMode {
        /// Shortens the reset time. Reserved, should be set to 0.
        #[bit(15, rw)]
        srt: bool,
        /// VBus power select. Connected to the vbus_pwr_select output, usable by generic
        /// external logic to choose between an on-chip VBus power source (charge pump)
        /// and an off-chip source when both are available. Host mode only.
        #[bit(5, rw)]
        vbps: bool,
        /// Stream disable mode. Device mode: disables double priming on both RX and TX
        /// for low bandwidth systems. Host mode: ensures overruns/underruns of the
        /// latency FIFO are eliminated for low bandwidth systems where the RX and TX
        /// buffers are sufficient to contain the entire packet.
        #[bit(4, rw)]
        sdis: bool,
        /// Setup lockout mode. 0 enables setup lockouts, 1 disables them, requiring use
        /// of the setup data buffer tripwire in [super::UsbCommand::sutw]. Device mode
        /// only.
        #[bit(3, rw)]
        slom: bool,
        /// Endian select, reserved, always reads 0.
        #[bit(2, r)]
        es: bool,
        /// Controller mode, see [ControllerMode]. For a combination host/device
        /// controller, this can only be written once after reset, to switch modes
        /// software must first reset the controller via [super::UsbCommand::rst].
        #[bits(0..=1, rw)]
        cm: ControllerMode,
    }

    /// Setup endpoint status. Set when a setup transaction is received on the
    /// corresponding endpoint. Software reads the setup data from the queue head and
    /// then writes 1 to clear the status bit. Device mode. This implementation supports
    /// 12 endpoints (0 to 11), higher bits are reserved.
    #[bitbybit::bitfield(u32, debug, defmt_bitfields(feature = "defmt"), forbid_overlaps)]
    pub struct SetupStatus {
        /// Per-endpoint setup status bits.
        #[bits(0..=15, rw)]
        status: u16,
    }

    /// USB transfer type for an endpoint direction.
    #[bitbybit::bitenum(u2, exhaustive = true)]
    #[derive(Debug, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum EndpointType {
        /// Control transfers. Endpoint 0 is always a control endpoint.
        Control = 0b00,
        /// Isochronous transfers.
        Isochronous = 0b01,
        /// Bulk transfers.
        Bulk = 0b10,
        /// Interrupt transfers.
        Interrupt = 0b11,
    }

    /// Per-endpoint TX/RX configuration, shared by endpoint 0's control register and the
    /// ENDPTCTRL1 to ENDPTCTRL11 array for endpoints 1 to 11. For endpoint 0,
    /// [Self::txe] and [Self::rxe] always read 1 and [Self::txt]/[Self::rxt] are fixed
    /// to [EndpointType::Control], reflecting that endpoint 0 is a permanently enabled
    /// control endpoint.
    #[bitbybit::bitfield(u32, debug, defmt_fields(feature = "defmt"), forbid_overlaps)]
    pub struct EndpointControl {
        /// TX endpoint enable. Enable the endpoint only after it has been configured.
        #[bit(23, rw)]
        txe: bool,
        /// TX data toggle reset. Write 1 to reset the PID sequence. Software must write
        /// 1 here whenever a configuration event is received for this endpoint, to
        /// synchronize the data PIDs between host and device.
        #[bit(22, rw)]
        txr: bool,
        /// TX data toggle inhibit, for testing. 1 disables PID sequencing. Read-only.
        #[bit(21, r)]
        txi: bool,
        /// TX endpoint transfer type, see [EndpointType].
        #[bits(18..=19, rw)]
        txt: EndpointType,
        /// TX endpoint datapath. Always write 0, selecting a dual-port memory buffer
        /// with a DMA engine.
        #[bit(17, rw)]
        txd: bool,
        /// TX endpoint stall. 1 forces a STALL handshake until cleared by software or
        /// automatically on receipt of a new SETUP request.
        #[bit(16, rw)]
        txs: bool,
        /// RX endpoint enable. Enable the endpoint only after it has been configured.
        #[bit(7, rw)]
        rxe: bool,
        /// RX data toggle reset. Write 1 to reset the PID sequence. Software must write
        /// 1 here whenever a configuration event is received for this endpoint, to
        /// synchronize the data PIDs between host and device.
        #[bit(6, rw)]
        rxr: bool,
        /// RX data toggle inhibit, for testing. 1 disables PID sequencing.
        #[bit(5, rw)]
        rxi: bool,
        /// RX endpoint transfer type, see [EndpointType].
        #[bits(2..=3, rw)]
        rxt: EndpointType,
        /// RX endpoint datapath. Always write 0, selecting a dual-port memory buffer
        /// with a DMA engine.
        #[bit(1, rw)]
        rxd: bool,
        /// RX endpoint stall. 1 forces a STALL handshake until cleared by software or
        /// automatically on receipt of a new SETUP request.
        #[bit(0, rw)]
        rxs: bool,
    }
}

/// USB register block.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    #[mmio(PureRead)]
    id: Id,
    #[mmio(PureRead)]
    hw_general: HwGeneral,
    #[mmio(PureRead)]
    hw_host: HwHost,
    #[mmio(PureRead)]
    hw_device: HwDevice,
    #[mmio(PureRead)]
    hw_txbuf: HwTxBuf,
    #[mmio(PureRead)]
    hw_rxbuf: HwRxBuf,

    _gap0: [u32; 0x1A],

    gp_timer0_load_value: GpTimerLoad,
    gp_timer0_control: GpTimerControl,

    gp_timer1_load_value: GpTimerLoad,
    gp_timer1_control: GpTimerControl,

    sbus_config: SbusConfig,

    _gap1: [u32; 0x1B],

    #[mmio(PureRead)]
    caplength_hci_version: CapLengthHciVersion,
    #[mmio(PureRead)]
    hcs_params: HcsParams,
    #[mmio(PureRead)]
    hcc_params: HccParams,

    _gap2: [u32; 0x5],

    #[mmio(PureRead)]
    dci_version: DciVersion,
    #[mmio(PureRead)]
    dcc_params: DccParams,

    _gap3: [u32; 0x6],

    command: UsbCommand,
    interrupt_status: UsbStatus,
    interrupt_enable: UsbInterruptEnable,
    frame: FrameIndex,
    _gap4: u32,
    listbase: ListBase,
    async_list_addr: AsyncListAddr,
    tt_control: TtControl,
    burstsize: BurstSize,
    tx_fill: TxFillTuning,
    tx_tt_fill_tuning: TtTxFillTuning,
    ic_usb: IcUsb,
    ulpi_view: UlpiViewport,
    _gap5: u32,
    endpoint_nak_isr: EndpointMask,
    endpoing_nak_ier: EndpointMask,
    #[mmio(PureRead)]
    config_flag: u32,
    portscr1: PortStatusControl,

    _gap6: [u32; 0x7],

    otg_csr: OtgStatusControl,
    mode: UsbMode,
    endpoint_status: SetupStatus,
    endpoint_primer: EndpointMask,
    endpoint_flush: EndpointMask,
    #[mmio(PureRead)]
    endpoint_ready: EndpointMask,
    endoing_tx_complete: EndpointMask,
    endpoint_control: [EndpointControl; 12],
}

static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, gp_timer0_load_value), 0x80);
static_assertions::const_assert_eq!(
    core::mem::offset_of!(Registers, caplength_hci_version),
    0x100
);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, dci_version), 0x120);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, command), 0x140);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, ulpi_view), 0x170);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, otg_csr), 0x1A4);
static_assertions::const_assert_eq!(core::mem::offset_of!(Registers, endpoint_control), 0x1C0);

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x1F0);

impl Registers {
    /// Create a new USB MMIO instance for for USB block at address [BASE_ADDR_0].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub fn new_mmio_fixed0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(BASE_ADDR_1) }
    }

    /// Create a new USB MMIO instance for for USB block at address [BASE_ADDR_1].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub fn new_mmio_fixed1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(BASE_ADDR_1) }
    }
}
