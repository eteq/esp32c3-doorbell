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

//#[macro_use]
//extern crate alloc;
//#[global_allocator]
//static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

use heapless;

//use critical_section::Mutex;

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

use micromath::F32Ext;

// parameters
const VERBOSE: bool = true;

const BUFFER_SIZE: usize = 16;
const RISE_WAIT_TIME_MS: u32 = 10;
const SENSE_WAIT_TIME_MS: u32 = 10;
const N_RETRY_CONNECT: usize = 10;
const RETRY_DELAY_MS: usize = 100;

const SSID: &str = env!("SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

const SEND_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 1, 38);
const SEND_PORT: u16 = 65432;

// real consts
const SENSE_WAIT_TIME_TICKS: u64 = SENSE_WAIT_TIME_MS as u64 * SystemTimer::TICKS_PER_SECOND/1000;
const DIV_TICKS_TO_US: u64 = SystemTimer::TICKS_PER_SECOND / 1000000u64;

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

    println!("Hello world!");

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
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    npx_channel = neopixel_transmit(npx_channel, 0, 0, 0).unwrap();

    // set up the touch monitoring

    //A0 = GPIO4 -> charging pin
    let mut charge_pin = io.pins.gpio4.into_push_pull_output();
    charge_pin.set_drive_strength(hal::gpio::DriveStrength::I40mA);

    //A3 = GPIO0 -> sense pin
    let sense_pin = io.pins.gpio0.into_floating_input();

    //critical_section::with(|cs| SENSE_PIN.borrow_ref_mut(cs).replace(sense_pin));
    
    let mut delay = Delay::new(&clocks);

    let mut rb = StaticRb::<u32, BUFFER_SIZE>::default();
    let mut sum = 0u32;
    let mut sumsq = 0u32;
    let mut mean = 0.0;
    let mut std = 0.0;

    npx_channel = neopixel_transmit(npx_channel, 25, 25, 0).unwrap();
    loop {
        let mut fall_time: Option<u64> = None;

        if rb.len() == rb.capacity() {
            npx_channel = neopixel_transmit(npx_channel, 0, 25, 0).unwrap();
        } 

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
                fall_time = Some((t - start_time)/DIV_TICKS_TO_US);
                break;
            }
        }

        if fall_time != None {
            let newval = fall_time.unwrap() as u32;

            if (rb.capacity() == rb.len()) & (newval as f32 > (2.0*mean+3.0*std)) {
                npx_channel = neopixel_transmit(npx_channel, 25, 0, 0).unwrap();
                println!("triggered with value {} mean:{},std:{}", newval, mean, std);
                delay.delay_ms(200u32);
            } else {
                println!("not triggered with value {} mean:{},std:{}", newval, mean, std);
                let popval = rb.push_overwrite(newval);
                (mean, std, sum, sumsq) = accumulate_stats(newval, popval, sum, sumsq, rb.len());
            }

            

        }

    }
}

fn accumulate_stats(newval: u32, popvalo: Option<u32>, oldsum: u32, oldsumsq: u32, n: usize) -> (f32, f32, u32, u32) {
    if n == 1 {
        return (newval as f32, 0.0, newval, newval*newval)
    } 
    let popval = match popvalo {
        None =>  {
            0
        }
        Some(val) => {
            val
        }
    };

    let newsum = oldsum - popval + newval;
    let newsumsq = oldsumsq - popval*popval + newval*newval;
    let newmean = newsum as f32/n as f32;
    let newstd = ((newsumsq as f32)/(n as f32) - newmean*newmean).sqrt();

    (newmean, newstd, newsum, newsumsq)
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
