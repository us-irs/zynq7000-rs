//! Zedboard ethernet example code.
//!
//! This code uses embassy-net, a smoltcp based networking stack, as the IP stack.
//! It uses DHCP by default to assign the IP address. The assigned address will be displayed on
//! the console.
//!
//! Alternatively, you can also set a static IPv4 configuration via the `STATIC_IPV4_CONFIG`
//! constant and by setting `USE_DHCP` to `false`.
//!
//! It also exposes simple UDP and TCP echo servers. You can use the following sample commands
//! to send UDP or TCP data to the Zedboard using the Unix `netcat` application:
//!
//! ## UDP
//!
//! ```sh
//! echo "Hello Zedboard" | nc -uN <ip-address> 8000
//! ```
//!
//! ## TCP
//!
//! ```sh
//! echo "Hello Zedboard" | nc -N <ip-address> 8000
//! ```
#![no_std]
#![no_main]

use core::{net::Ipv4Addr, panic::PanicInfo};
use cortex_ar::asm::nop;
use embassy_executor::Spawner;
use embassy_net::{Ipv4Cidr, StaticConfigV4, tcp::TcpSocket, udp::UdpSocket};
use embassy_time::{Duration, Timer};
use embedded_io::Write;
use embedded_io_async::Write as _;
use log::{LevelFilter, debug, error, info, warn};
use rand::{RngCore, SeedableRng};
use zedboard::PS_CLOCK_FREQUENCY;
use zedboard_bsp::phy_marvell;
use zynq7000_hal::{
    BootMode,
    clocks::Clocks,
    configure_level_shifter,
    eth::{
        AlignedBuffer, ClockDivSet, EthernetConfig, EthernetLowLevel, embassy_net::InterruptResult,
    },
    gic::{GicConfigurator, GicInterruptHelper, Interrupt},
    gpio::{GpioPins, Output, PinState},
    gtc::GlobalTimerCounter,
    l2_cache,
    uart::{ClockConfig, Config, Uart},
};

use zynq7000::{Peripherals, slcr::LevelShifterConfig};
use zynq7000_rt::{self as _, mmu::section_attrs::SHAREABLE_DEVICE, mmu_l1_table_mut};

const USE_DHCP: bool = true;
const UDP_AND_TCP_PORT: u16 = 8000;
const PRINT_PACKET_STATS: bool = false;
const LOG_LEVEL: LevelFilter = LevelFilter::Info;
const NUM_RX_SLOTS: usize = 16;
const NUM_TX_SLOTS: usize = 16;

const STATIC_IPV4_CONFIG: StaticConfigV4 = StaticConfigV4 {
    address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 179, 25), 24),
    gateway: None,
    dns_servers: heapless::Vec::new(),
};

const INIT_STRING: &str = "-- Zynq 7000 Zedboard Ethernet Example --\n\r";

// Unicast address with OUI of the Marvell 88E1518 PHY.
const MAC_ADDRESS: [u8; 6] = [
    0x00,
    ((phy_marvell::MARVELL_88E1518_OUI >> 8) & 0xff) as u8,
    (phy_marvell::MARVELL_88E1518_OUI & 0xff) as u8,
    0x00,
    0x00,
    0x01,
];

/// See memory.x file. 1 MB starting at this address will be configured as uncached memory using the
/// MMU.
const UNCACHED_ADDR: u32 = 0x4000000;

// These descriptors must be placed in uncached memory. The MMU will be used to configure the
// .uncached memory segment as device memory.
#[unsafe(link_section = ".uncached")]
static RX_DESCRIPTORS: zynq7000_hal::eth::rx_descr::DescriptorList<NUM_RX_SLOTS> =
    zynq7000_hal::eth::rx_descr::DescriptorList::new();
#[unsafe(link_section = ".uncached")]
static TX_DESCRIPTORS: zynq7000_hal::eth::tx_descr::DescriptorList<NUM_TX_SLOTS> =
    zynq7000_hal::eth::tx_descr::DescriptorList::new();

static ETH_ERR_QUEUE: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    InterruptResult,
    8,
> = embassy_sync::channel::Channel::new();

#[derive(Debug, PartialEq, Eq)]
pub enum IpMode {
    LinkDown,
    AutoNegotiating,
    AwaitingIpConfig,
    StackReady,
}

/// Entry point (not called like a normal main function)
#[unsafe(no_mangle)]
pub extern "C" fn boot_core(cpu_id: u32) -> ! {
    if cpu_id != 0 {
        panic!("unexpected CPU ID {}", cpu_id);
    }
    main();
}

#[embassy_executor::task]
async fn embassy_net_task(
    mut runner: embassy_net::Runner<'static, zynq7000_hal::eth::embassy_net::Driver>,
) -> ! {
    runner.run().await
}

/// Simple UDP echo task.
#[embassy_executor::task]
async fn udp_task(mut udp: UdpSocket<'static>) -> ! {
    let mut rx_buf = [0; zynq7000_hal::eth::MTU];
    udp.bind(UDP_AND_TCP_PORT)
        .expect("failed to bind UDP socket to port 8000");
    loop {
        match udp.recv_from(&mut rx_buf).await {
            Ok((data, meta)) => {
                log::info!("udp rx {data} bytes from {meta:?}");
                match udp.send_to(&rx_buf[0..data], meta).await {
                    Ok(_) => (),
                    Err(e) => {
                        log::warn!("udp send error: {e:?}");
                        Timer::after_millis(100).await;
                    }
                }
            }
            Err(e) => {
                log::warn!("udp receive error: {e:?}");
                Timer::after_millis(100).await;
            }
        }
    }
}

/// Simple TCP echo task.
#[embassy_executor::task]
async fn tcp_task(mut tcp: TcpSocket<'static>) -> ! {
    let mut rx_buf = [0; zynq7000_hal::eth::MTU];
    tcp.set_timeout(Some(Duration::from_secs(2)));
    loop {
        match tcp.accept(UDP_AND_TCP_PORT).await {
            Ok(_) => {
                log::info!("tcp connection to {:?} accepted", tcp.remote_endpoint());
                loop {
                    if tcp.may_recv() {
                        match tcp.read(&mut rx_buf).await {
                            Ok(0) => {
                                log::info!("tcp EOF received");
                                tcp.close();
                            }
                            Ok(read_bytes) => {
                                log::info!("tcp rx {read_bytes} bytes");
                                if tcp.may_send() {
                                    match tcp.write_all(&rx_buf[0..read_bytes]).await {
                                        Ok(_) => continue,
                                        Err(e) => {
                                            log::warn!("tcp error when writing: {e:?}");
                                            Timer::after_millis(100).await;
                                        }
                                    }
                                } else {
                                    log::warn!("tcp remote endpoint not writeable");
                                    continue;
                                }
                            }
                            Err(_) => {
                                log::warn!("tcp connection reset by remote endpoint.");
                                tcp.close();
                            }
                        }
                    }
                    if !tcp.may_send() && !tcp.may_recv() {
                        log::info!("tcp send and receive side closed");
                        tcp.close();
                    }
                    if tcp.state() == embassy_net::tcp::State::Closed {
                        log::info!("tcp socket closed, exiting loop");
                        break;
                    }
                    Timer::after_millis(100).await;
                }
            }
            Err(e) => {
                log::warn!("tcp error accepting connection: {e:?}");
                Timer::after_millis(100).await;
                continue;
            }
        }
    }
}

#[embassy_executor::main]
#[unsafe(export_name = "main")]
async fn main(spawner: Spawner) -> ! {
    let mut dp = Peripherals::take().unwrap();
    l2_cache::init_with_defaults(&mut dp.l2c);

    // Enable PS-PL level shifters.
    configure_level_shifter(LevelShifterConfig::EnableAll);

    // Configure the uncached memory region using the MMU.
    mmu_l1_table_mut()
        .update(UNCACHED_ADDR, SHAREABLE_DEVICE)
        .expect("configuring uncached memory section failed");

    // Clock was already initialized by PS7 Init TCL script or FSBL, we just read it.
    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    // Set up the global interrupt controller.
    let mut gic = GicConfigurator::new_with_init(dp.gicc, dp.gicd);
    gic.enable_all_interrupts();
    gic.set_all_spi_interrupt_targets_cpu0();
    gic.enable();
    unsafe {
        gic.enable_interrupts();
    }
    let gpio_pins = GpioPins::new(dp.gpio);

    // Set up global timer counter and embassy time driver.
    let gtc = GlobalTimerCounter::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::init(clocks.arm_clocks(), gtc);

    // Set up the UART, we are logging with it.
    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = Uart::new_with_mio(
        dp.uart_1,
        Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    uart.write_all(INIT_STRING.as_bytes()).unwrap();
    // Safety: We are not multi-threaded yet.
    unsafe { zynq7000_hal::log::uart_blocking::init_unsafe_single_core(uart, LOG_LEVEL, false) };

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    static ETH_RX_BUFS: static_cell::ConstStaticCell<[AlignedBuffer; NUM_RX_SLOTS]> =
        static_cell::ConstStaticCell::new(
            [AlignedBuffer([0; zynq7000_hal::eth::MTU]); NUM_RX_SLOTS],
        );
    static ETH_TX_BUFS: static_cell::ConstStaticCell<[AlignedBuffer; NUM_TX_SLOTS]> =
        static_cell::ConstStaticCell::new(
            [AlignedBuffer([0; zynq7000_hal::eth::MTU]); NUM_TX_SLOTS],
        );
    let rx_bufs = ETH_RX_BUFS.take();
    let tx_bufs = ETH_TX_BUFS.take();

    let rx_descr = RX_DESCRIPTORS.take().unwrap();
    let tx_descr = TX_DESCRIPTORS.take().unwrap();
    // Unwraps okay, list length is not 0
    let mut rx_descr_ref =
        zynq7000_hal::eth::rx_descr::DescriptorListWrapper::new(rx_descr.as_mut_slice());
    let mut tx_descr_ref =
        zynq7000_hal::eth::tx_descr::DescriptorListWrapper::new(tx_descr.as_mut_slice());
    rx_descr_ref.init_with_aligned_bufs(rx_bufs.as_slice());
    tx_descr_ref.init_or_reset();

    // Unwrap okay, this is a valid peripheral.
    let eth_ll = EthernetLowLevel::new(dp.eth_0).unwrap();
    let mod_id = eth_ll.regs.read_module_id();
    info!("Ethernet Module ID: {mod_id:?}");
    assert_eq!(mod_id, 0x20118);

    let (clk_divs, clk_errors) = ClockDivSet::calculate_for_rgmii_and_io_clock(clocks.io_clocks());
    debug!(
        "Calculated RGMII clock configuration: {:?}, errors (missmatch from ideal rate in hertz): {:?}",
        clk_divs, clk_errors
    );
    // Unwrap okay, we use a standard clock config, and the clock config should never fail.
    let eth_cfg = EthernetConfig::new(
        zynq7000_hal::eth::ClockConfig::new(clk_divs.cfg_1000_mbps),
        zynq7000_hal::eth::calculate_mdc_clk_div(clocks.arm_clocks()).unwrap(),
        MAC_ADDRESS,
    );
    // Configures all the physical pins for ethernet operation and sets up the
    // ethernet peripheral.
    let mut eth = zynq7000_hal::eth::Ethernet::new_with_mio(
        eth_ll,
        eth_cfg,
        gpio_pins.mio.mio16,
        gpio_pins.mio.mio21,
        (
            gpio_pins.mio.mio17,
            gpio_pins.mio.mio18,
            gpio_pins.mio.mio19,
            gpio_pins.mio.mio20,
        ),
        gpio_pins.mio.mio22,
        gpio_pins.mio.mio27,
        (
            gpio_pins.mio.mio23,
            gpio_pins.mio.mio24,
            gpio_pins.mio.mio25,
            gpio_pins.mio.mio26,
        ),
        Some((gpio_pins.mio.mio52, gpio_pins.mio.mio53)),
    );

    eth.set_rx_buf_descriptor_base_address(rx_descr_ref.base_addr());
    eth.set_tx_buf_descriptor_base_address(tx_descr_ref.base_addr());
    eth.start();
    let (mut phy, phy_rev) = phy_marvell::Marvell88E1518Phy::new_autoprobe_addr(eth.mdio_mut())
        .expect("could not auto-detect phy");
    info!(
        "Detected Marvell 88E1518 PHY with revision number: {:?}",
        phy_rev
    );
    phy.reset();
    phy.restart_auto_negotiation();

    let driver = zynq7000_hal::eth::embassy_net::Driver::new(
        &eth,
        MAC_ADDRESS,
        zynq7000_hal::eth::embassy_net::DescriptorsAndBuffers::new(
            rx_descr_ref,
            rx_bufs,
            tx_descr_ref,
            tx_bufs,
        )
        .unwrap(),
    );
    let config = if USE_DHCP {
        embassy_net::Config::dhcpv4(Default::default())
    } else {
        embassy_net::Config::ipv4_static(STATIC_IPV4_CONFIG)
    };
    static RESOURCES: static_cell::StaticCell<embassy_net::StackResources<3>> =
        static_cell::StaticCell::new();
    let mut rng = rand::rngs::SmallRng::seed_from_u64(1);
    let (stack, runner) = embassy_net::new(
        driver,
        config,
        RESOURCES.init(embassy_net::StackResources::new()),
        rng.next_u64(),
    );

    // Ensure those are in the data section by making them static.
    static RX_UDP_META: static_cell::ConstStaticCell<[embassy_net::udp::PacketMetadata; 8]> =
        static_cell::ConstStaticCell::new([embassy_net::udp::PacketMetadata::EMPTY; 8]);
    static TX_UDP_META: static_cell::ConstStaticCell<[embassy_net::udp::PacketMetadata; 8]> =
        static_cell::ConstStaticCell::new([embassy_net::udp::PacketMetadata::EMPTY; 8]);
    static TX_UDP_BUFS: static_cell::ConstStaticCell<[u8; zynq7000_hal::eth::MTU]> =
        static_cell::ConstStaticCell::new([0; zynq7000_hal::eth::MTU]);
    static RX_UDP_BUFS: static_cell::ConstStaticCell<[u8; zynq7000_hal::eth::MTU]> =
        static_cell::ConstStaticCell::new([0; zynq7000_hal::eth::MTU]);

    let udp_socket = UdpSocket::new(
        stack,
        RX_UDP_META.take(),
        RX_UDP_BUFS.take(),
        TX_UDP_META.take(),
        TX_UDP_BUFS.take(),
    );

    // Ensure those are in the data section by making them static.
    static TX_TCP_BUFS: static_cell::ConstStaticCell<[u8; zynq7000_hal::eth::MTU]> =
        static_cell::ConstStaticCell::new([0; zynq7000_hal::eth::MTU]);
    static RX_TCP_BUFS: static_cell::ConstStaticCell<[u8; zynq7000_hal::eth::MTU]> =
        static_cell::ConstStaticCell::new([0; zynq7000_hal::eth::MTU]);

    let tcp_socket = TcpSocket::new(stack, RX_TCP_BUFS.take(), TX_TCP_BUFS.take());

    // Spawn all embassy tasks.
    spawner.spawn(embassy_net_task(runner)).unwrap();
    spawner.spawn(udp_task(udp_socket)).unwrap();
    spawner.spawn(tcp_task(tcp_socket)).unwrap();

    let mut mio_led = Output::new_for_mio(gpio_pins.mio.mio7, PinState::Low);

    let mut ip_mode = IpMode::LinkDown;
    let mut transmitted_frames = 0;
    let mut received_frames = 0;
    let receiver = ETH_ERR_QUEUE.receiver();
    loop {
        // Handle error messages from ethernet interrupt.
        while let Ok(msg) = receiver.try_receive() {
            info!("Received interrupt result: {msg:?}");
        }
        if PRINT_PACKET_STATS {
            let sent_frames_since_last = eth.ll().regs.statistics().read_tx_count();
            if sent_frames_since_last > 0 {
                transmitted_frames += sent_frames_since_last;
                info!("Frame sent count: {transmitted_frames}");
            }
            let received_frames_since_last = eth.ll().regs.statistics().read_rx_count();
            if received_frames_since_last > 0 {
                received_frames += received_frames_since_last;
                info!("Frame received count: {received_frames}");
            }
        }

        // This is basically a linker checker task. It also takes care of notifying the
        // embassy stack of link state changes.
        match ip_mode {
            // Assuming that auto-negotiation is performed automatically.
            IpMode::LinkDown => {
                mio_led.set_low();
                zynq7000_hal::eth::embassy_net::update_link_state(
                    embassy_net::driver::LinkState::Down,
                );
                ip_mode = IpMode::AutoNegotiating;
            }
            IpMode::AutoNegotiating => {
                let status = phy.read_copper_status();
                if status.auto_negotiation_complete() {
                    let extended_status = phy.read_copper_specific_status_register_1();
                    info!(
                        "link is up and auto-negotiation complete. Setting speed {:?} and duplex {:?}",
                        extended_status.speed().as_zynq7000_eth_speed().unwrap(),
                        extended_status.duplex().as_zynq7000_eth_duplex()
                    );
                    eth.configure_clock_and_speed_duplex(
                        // If this has the reserved bits, what do we even do? For this example app,
                        // I am going to assume this never happens..
                        extended_status.speed().as_zynq7000_eth_speed().unwrap(),
                        extended_status.duplex().as_zynq7000_eth_duplex(),
                        &clk_divs,
                    );
                    zynq7000_hal::eth::embassy_net::update_link_state(
                        embassy_net::driver::LinkState::Up,
                    );
                    ip_mode = IpMode::AwaitingIpConfig;
                } else {
                    Timer::after_millis(100).await;
                }
            }
            IpMode::AwaitingIpConfig => {
                if stack.is_config_up() {
                    let network_config = stack.config_v4();
                    info!("Network configuration is up. config: {network_config:?}!",);
                    ip_mode = IpMode::StackReady;
                    mio_led.set_high();
                } else {
                    Timer::after_millis(100).await;
                }
            }
            IpMode::StackReady => {
                let status = phy.read_copper_status();
                // Periodically check for link changes.
                if status.copper_link_status() == phy_marvell::LatchingLinkStatus::DownSinceLastRead
                {
                    warn!("ethernet link is down.");
                    ip_mode = IpMode::LinkDown;
                    continue;
                }
                Timer::after_millis(100).await;
            }
        }
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {
    let mut gic_helper = GicInterruptHelper::new();
    let irq_info = gic_helper.acknowledge_interrupt();
    match irq_info.interrupt() {
        Interrupt::Sgi(_) => (),
        Interrupt::Ppi(ppi_interrupt) => {
            if ppi_interrupt == zynq7000_hal::gic::PpiInterrupt::GlobalTimer {
                unsafe {
                    zynq7000_embassy::on_interrupt();
                }
            }
        }
        Interrupt::Spi(spi_interrupt) => {
            if spi_interrupt == zynq7000_hal::gic::SpiInterrupt::Eth0 {
                // This generic library provided interrupt handler takes care of waking
                // the driver on received or sent frames while also reporting anomalies
                // and errors.
                let result = zynq7000_hal::eth::embassy_net::on_interrupt(
                    zynq7000_hal::eth::EthernetId::Eth0,
                );
                if result.has_errors() {
                    ETH_ERR_QUEUE.try_send(result).ok();
                }
            }
        }
        Interrupt::Invalid(_) => (),
        Interrupt::Spurious => (),
    }
    gic_helper.end_of_interrupt(irq_info);
}

#[zynq7000_rt::exception(DataAbort)]
fn data_abort_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(Undefined)]
fn undefined_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(PrefetchAbort)]
fn prefetch_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

/// Panic handler
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {info:?}");
    loop {}
}
