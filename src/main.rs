#![no_std]
#![no_main]


use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::{ClockControl, Clocks},
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    systimer, systimer::SystemTimer,
    Delay, Rtc, Rmt, Rng,
    rmt::{PulseCode, TxChannel, TxChannelConfig, TxChannelCreator},
};


use heapless;
use core::fmt::Write;

use embedded_svc::io::*;
use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{AccessPointInfo, ClientConfiguration, Configuration, Wifi};

use esp_wifi;
use esp_wifi::wifi::{WifiMode, WifiError};
use esp_wifi::wifi_interface::{WifiStack, Socket};
use esp_wifi::wifi::utils::create_network_interface;
use smoltcp;
use smoltcp::wire::{IpAddress, Ipv4Address};

use ringbuf::{StaticRb, ring_buffer::Rb};

// parameters
const VERBOSE: bool = true;

const SHORT_BUFFER_SIZE : usize = 11;  // should be odd for median to be right

const BUFFER_SIZE: usize = 16;
const RISE_WAIT_TIME_MS: u32 = 10;
const SENSE_WAIT_TIME_MS: u32 = 10;
const N_RETRY_CONNECT: usize = 10;
const RETRY_DELAY_MS: usize = 100;
const BUFFER_RESET_INTERVAL_TICKS: u64 = 3600 * SystemTimer::TICKS_PER_SECOND;  // once per hour

// this is the fraction of the stored buffer that the median must be larger than to trigger
const THRESHOLD_FRAC: [u32; 2] = [4, 3];
// if a new value is smaller than this fraction of the largest value in the long buffer, it is added to the long buffer
const STORE_FRAC: [u32; 2] = [11, 10];

const SSID: &str = env!("SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

const SEND_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 1, 38);
const SEND_PORT: u16 = 65434;

// real consts
const SENSE_WAIT_TIME_TICKS: u64 = SENSE_WAIT_TIME_MS as u64 * SystemTimer::TICKS_PER_SECOND/1000;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    //let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clocks = ClockControl::configure(system.clock_control, hal::clock::CpuClock::Clock160MHz).freeze();
    let mut clock_control = system.peripheral_clock_control;

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    if VERBOSE { println!("Starting up!"); }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);


    //setup for neopixel
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &mut clock_control, &clocks).unwrap();
    let mut npx_channel = rmt
    .channel0
    .configure(
        io.pins.gpio2.into_push_pull_output(),
        TxChannelConfig {
            clk_divider: 4,  // gives 20 MHz
            idle_output_level: false,
            idle_output: false,
            carrier_modulation: false,
            carrier_high: 1,
            carrier_low: 1,
            carrier_level: false,
        },
    )
    .unwrap();
    npx_channel = neopixel_transmit(npx_channel, 25, 0, 25).unwrap();


    // setup the wifi
    let wifi_timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let wifi_rng = Rng::new(peripherals.RNG);
    let (radio_wifi, _) = peripherals.RADIO.split();
    let mut wifi_socket_set_entries: [smoltcp::iface::SocketStorage; 3] = Default::default();
    let res = setup_wifi(VERBOSE, wifi_timer, wifi_rng, system.radio_clock_control, &clocks, radio_wifi, &mut wifi_socket_set_entries);
    if res.is_err() {
        neopixel_transmit(npx_channel, 25, 12, 0).unwrap();
        panic!("Error while setting up wifi: {:?}", res.err());
    }
    let wifi_stack = res.unwrap();
    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    //let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);
    let mut rx_meta = [smoltcp::socket::udp::PacketMetadata::EMPTY; 10];
    let mut tx_meta = [smoltcp::socket::udp::PacketMetadata::EMPTY; 10];
    let mut socket = wifi_stack.get_udp_socket(&mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    socket.bind(SEND_PORT).unwrap();
    
    npx_channel = neopixel_transmit(npx_channel, 0, 0, 0).unwrap();

    // set up the touch monitoring

    //A0 = GPIO4 -> charging pin
    let mut charge_pin = io.pins.gpio4.into_push_pull_output();
    charge_pin.set_drive_strength(hal::gpio::DriveStrength::I40mA);

    //A3 = GPIO0 -> sense pin
    let sense_pin = io.pins.gpio0.into_floating_input();
    
    let mut delay = Delay::new(&clocks);

    let mut short_buffer = [0u32; SHORT_BUFFER_SIZE];
    let mut short_buffer_index = 0;

    let mut long_buffer = StaticRb::<u32, BUFFER_SIZE>::default();
    let mut long_buffer_sum = 0u32;

    let mut last_buffer_reset = SystemTimer::now();

    loop {
        socket.work();
        if (SystemTimer::now() - last_buffer_reset) > BUFFER_RESET_INTERVAL_TICKS {
            long_buffer.clear();
            long_buffer_sum = 0;
            // last_buffer_reset is reset to system time just below
        }
        if long_buffer.len() == long_buffer.capacity() {
            npx_channel = neopixel_transmit(npx_channel, 0, 25, 0).unwrap();
        } else {
            npx_channel = neopixel_transmit(npx_channel, 25, 25, 0).unwrap();
            last_buffer_reset = SystemTimer::now();
        }

        let mut fall_time: Option<u32> = None;

        charge_pin.set_high().unwrap();
        delay.delay_ms(RISE_WAIT_TIME_MS);

        
        let start_time = critical_section::with(|_cs| {
            let start_time1 = SystemTimer::now();
            charge_pin.set_low().unwrap();
            let start_time2 = SystemTimer::now();
            (start_time1 + start_time2) / 2
        });

        // poll until time is up or sense pin is low
        while (SystemTimer::now() - start_time) < SENSE_WAIT_TIME_TICKS {

            let mut sense_low = false;
            let t = critical_section::with(|_cs| {
                let time1 = SystemTimer::now();
                sense_low = sense_pin.is_low().unwrap();
                let time2 = SystemTimer::now(); 

                (time1 + time2)/2
            });

            if sense_low {
                fall_time = Some((t - start_time) as u32);
                break;
            }
        }

        if fall_time != None {
            short_buffer[short_buffer_index] = fall_time.unwrap();
            short_buffer_index += 1;

            if short_buffer_index >= short_buffer.len() {
                short_buffer_index = 0;  // reset median after this

                short_buffer.sort_unstable();
                let median = short_buffer[short_buffer.len()/2];

                if long_buffer.len() < long_buffer.capacity() {
                    if VERBOSE { println!("stored {} at init", median); }
                    long_buffer.push_overwrite(median);
                    long_buffer_sum += median;
                } else {
                    // add the median element in only if at least one of the existing buffer elements is larger  or equal
                    let threshold_median = (median * STORE_FRAC[1]) / STORE_FRAC[0];
                    'checkforstore: {
                        for elem in long_buffer.iter() {
                            if threshold_median <= (*elem) {
                                if VERBOSE {
                                    println!("stored {} because {} <= {}", median, threshold_median, *elem);
                                    println!("long buffer: {:?}", long_buffer.iter().collect::<heapless::Vec<&u32, BUFFER_SIZE>>());
                                }
                                let popped_val = long_buffer.push_overwrite(median).unwrap();
                                long_buffer_sum = long_buffer_sum - popped_val + median;
                                break 'checkforstore;
                            }
                        }
                        if VERBOSE {
                            println!("did not store {}, too large", median);
                            println!("long buffer: {:?}", long_buffer.iter().collect::<heapless::Vec<&u32, BUFFER_SIZE>>());
                        }
                    }

                    // if the median meets the threshold, trigger
                    if ((median * THRESHOLD_FRAC[1]) / THRESHOLD_FRAC[0]) > long_buffer_sum / long_buffer.len() as u32 {
                        npx_channel = neopixel_transmit(npx_channel, 25, 0, 0).unwrap();
                        if VERBOSE {
                            println!("triggered, because {} > {}, med:{}", (median * THRESHOLD_FRAC[1]) / THRESHOLD_FRAC[0], long_buffer_sum / long_buffer.len() as u32, median);
                        }
                        //delay.delay_ms(250u32);

                        
                        // UDP send
                        let mut s: heapless::String<500> = heapless::String::new();
                        write!(s, "triggered with fall time {}, long buffer:{:?}", median, long_buffer.iter().collect::<heapless::Vec<&u32, BUFFER_SIZE>>()).unwrap();
                        socket.send(IpAddress::Ipv4(SEND_ADDRESS), SEND_PORT, s.as_bytes()).unwrap();

                        // TCP send
                        // socket.open(IpAddress::Ipv4(SEND_ADDRESS), SEND_PORT).unwrap();
                        // write!(socket, "triggered with fall time {}", median).unwrap();
                        //socket.flush().unwrap();

                        let wait_end = SystemTimer::now() + SystemTimer::TICKS_PER_SECOND/4; // 250 ms
                        while SystemTimer::now() < wait_end {
                            socket.work();
                        }

                        // socket.disconnect();
                    } else {
                        let mut s: heapless::String<500> = heapless::String::new();
                        write!(s, "did not trigger with fall time {}, long buffer:{:?}", median, long_buffer.iter().collect::<heapless::Vec<&u32, BUFFER_SIZE>>()).unwrap();
                        socket.send(IpAddress::Ipv4(SEND_ADDRESS), SEND_PORT, s.as_bytes()).unwrap();
                        let wait_end = SystemTimer::now() + SystemTimer::TICKS_PER_SECOND/4; // 250 ms
                        while SystemTimer::now() < wait_end {
                            socket.work();
                        }
                        
                    }
                }
            }
        }
    }
}

fn neopixel_transmit(channel: hal::rmt::Channel0<0>, r: u8, g: u8, b: u8) -> Result<hal::rmt::Channel0<0>, (hal::rmt::Error, hal::rmt::Channel0<0>)>  {
    
    let mut data = [PulseCode::default(); 25];
    for i in 0..8 { data[i] = neopixel_val_to_pulsecode(g, 7 - i); }
    for i in 0..8 { data[i+8] = neopixel_val_to_pulsecode(r, 7 - i); }
    for i in 0..8 { data[i+16] = neopixel_val_to_pulsecode(b, 7 - i); }

    channel.transmit(&data).wait()
}

fn neopixel_val_to_pulsecode(val: u8, i: usize) -> PulseCode {
    if val >> i & 1 == 1 {
        PulseCode {
            level1: true,
            length1: 14, //700 ns
            level2: false,
            length2: 12, //600 ns
        }
    } else {
        PulseCode {
            level1: true,
            length1: 7, //350 ns
            level2: false,
            length2: 16, //800 ns
        }
    }
}



fn setup_wifi<'a>(verbose: bool,
              wifi_timer: systimer::Alarm<systimer::Target, 0>, 
              wifi_rng: Rng, 
              radio_clock_control: hal::system::RadioClockControl, 
              clocks: &'a Clocks<'a>, 
              radio_wifi: hal::radio::Wifi,
              socket_set_entries: &'a mut [smoltcp::iface::SocketStorage<'a>; 3]) 
              -> Result<WifiStack<'a>, WifiError>
              {
    let init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        wifi_timer,
        wifi_rng,
        radio_clock_control,
        clocks,
    ).unwrap();

    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, radio_wifi, WifiMode::Sta, socket_set_entries).unwrap();
    let wifi_stack = WifiStack::new(iface, device, sockets, esp_wifi::current_millis);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: WIFI_PASSWORD.into(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    if verbose {
        println!("wifi_set_configuration returned {:?}", res);
    }

    controller.start().unwrap();
    if verbose {
        println!("is wifi started: {:?}", controller.is_started());    
        println!("Start Wifi Scan");

        let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
        match res {
            Ok((res, count)) => {
                println!("Found {} APs", count);
                for ap in res {
                    println!("{:?}", ap);
                }
            }
            Err(err) => {
                return Err(err);
            }
        }

        println!("{:?}", controller.get_capabilities());
        
    }

    let mut nerr = 0;
    'outer: loop {
        let conn_res = controller.connect();

        // wait to get connected
        if verbose {
            println!("wifi_connect result: {:?}, wait to get connected", conn_res);
        }
        'inner: loop {
            
            let res = controller.is_connected();
            match res {
                Ok(connected) => {
                    if connected {
                        break 'outer;
                    }
                }
                Err(err) => {
                    nerr += 1;
                    if nerr < N_RETRY_CONNECT {
                        if verbose {
                            println!("Got on error on try {} while trying to connect: {:?}, waiting {} ms", nerr, err, RETRY_DELAY_MS);
                        }
                        let target_time = SystemTimer::now() + RETRY_DELAY_MS as u64 * SystemTimer::TICKS_PER_SECOND/1000;
                        while SystemTimer::now() < target_time { }
                        break 'inner;
                    } else {
                        return Err(err);
                    }
                }
            }
        }
        if verbose {
            println!("{:?}", controller.is_connected());
        }
    }

    // wait for getting an ip address
    if verbose {
        println!("Wait to get an ip address");
    }
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            if verbose {
                println!("got ip {:?}", wifi_stack.get_ip_info());
            }
            break;
        }
    }


    Ok(wifi_stack)
}
