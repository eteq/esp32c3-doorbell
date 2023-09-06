#![no_std]
#![no_main]

use core::cell::{RefCell, Cell};
use critical_section::Mutex;
use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    gpio::{Event, Gpio0, Input, Floating, IO},
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    riscv,
    timer::TimerGroup,
    systimer::SystemTimer,
    Delay, Rtc, Rmt,
    rmt::{PulseCode, TxChannel, TxChannelConfig, TxChannelCreator},
    adc,
    adc::{AdcConfig, Attenuation, ADC, ADC1},
};


static LAST_FALL: Mutex<Cell<Option<u64>>> = Mutex::new(Cell::new(None));
static SENSE_PIN: Mutex<RefCell<Option<Gpio0<Input<Floating>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
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
    npx_channel = neopixel_transmit(npx_channel, 0, 0, 0).unwrap();

    //A0 = GPIO4 -> charging pin
    let mut charge_pin = io.pins.gpio4.into_push_pull_output();
    charge_pin.set_drive_strength(hal::gpio::DriveStrength::I40mA);

    //A3 = GPIO0 -> sense pin
    let mut sense_pin = io.pins.gpio0.into_floating_input();


    // set up ADC
    let analog = peripherals.APB_SARADC.split();

    let mut adc1_config = AdcConfig::new();

    // You can try any of the following calibration methods by uncommenting them
    // type AdcCal = ();
    // type AdcCal = adc::AdcCalBasic<ADC1>;
    // type AdcCal = adc::AdcCalLine<ADC1>;
    type AdcCal = adc::AdcCalCurve<ADC1>;

    // GPIO1 = A2 on qtpy
    let mut adc1pin = adc1_config.enable_pin_with_cal::<_, AdcCal>(io.pins.gpio1.into_analog(), 
                                                    Attenuation::Attenuation11dB);

    let mut adc1 = ADC::<ADC1>::adc(
        &mut clock_control,
        analog.adc1,
        adc1_config,
    ).unwrap();

    // Now setup interrupt on sense pin
    sense_pin.listen(Event::FallingEdge);

    // needed for clearing interrupt
    critical_section::with(|cs| SENSE_PIN.borrow_ref_mut(cs).replace(sense_pin));
    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    unsafe {
        riscv::interrupt::enable();
    }
    
    let mut delay = Delay::new(&clocks);

    let mut adc_times = [0u64; 1000];
    let mut adc_data = [0u16; 1000];
    loop {

        npx_channel = neopixel_transmit(npx_channel, 25, 0, 0).unwrap();
        charge_pin.set_high().unwrap();
        delay.delay_ms(100u32);

        npx_channel = neopixel_transmit(npx_channel, 0, 25, 0).unwrap();
        
        let start_time = critical_section::with(|cs| {
            let start_time1 = SystemTimer::now();
            charge_pin.set_low().unwrap();
            let start_time2 = SystemTimer::now();
            LAST_FALL.borrow(cs).set(None);
            (start_time1 + start_time2) / 2
        });

        for i in 0..400 {
            let adc_time = SystemTimer::now();
            let adc_val: u16 = nb::block!(adc1.read(&mut adc1pin)).unwrap();
            adc_times[i] = adc_time;
            adc_data[i] = adc_val;
        }

        let dtus = (SystemTimer::now() - start_time)/(SystemTimer::TICKS_PER_SECOND/1000000);
        if dtus < 300000 { delay.delay_us(300000 - dtus as u32); }

        let fall_time = critical_section::with(|cs| {
            let val = LAST_FALL.borrow(cs).get();
            LAST_FALL.borrow(cs).set(None);
            val
        });

        match fall_time {
            Some(t) => println!("fall time: {} ticks, {} us", (t - start_time), (t - start_time) / (SystemTimer::TICKS_PER_SECOND/1000000)),
            None => println!("fall time: None"),
        }
        println!("adc times: {:?}", adc_times);
        println!("adc values: {:?}", adc_data);
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

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        SENSE_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

            if LAST_FALL.borrow(cs).get().is_none() {
                LAST_FALL.borrow(cs).set(Some(SystemTimer::now()));
            }
    });
}