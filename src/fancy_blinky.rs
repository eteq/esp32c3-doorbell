#![no_std]
#![no_main]

use core::cell::{RefCell, Cell};
use critical_section::Mutex;
use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    gpio::{Event, Gpio9, Input, PullUp, IO},
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    riscv,
    timer::TimerGroup,
    Delay, Rtc, Rmt,
    rmt::{PulseCode, TxChannel, TxChannelConfig, TxChannelCreator}
};

static IUPS: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));
static BUTTON: Mutex<RefCell<Option<Gpio9<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));

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

    // Set GPIO9 as an input
    let mut button = io.pins.gpio9.into_pull_up_input();
    button.listen(Event::RisingEdge);

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    unsafe {
        riscv::interrupt::enable();
    }

    let mut delay = Delay::new(&clocks);
    loop {
        npx_channel = neopixel_transmit(npx_channel, 0, 0, 0).unwrap();
        delay.delay_ms(500u32);

        let iups = critical_section::with(|cs| IUPS.borrow(cs).get());
        if iups % 2 == 0 {
            npx_channel = neopixel_transmit(npx_channel, 0, 25, 10).unwrap();
        } else {
            npx_channel = neopixel_transmit(npx_channel, 25, 0, 0).unwrap();
        }
        
        delay.delay_ms(500u32);
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
        println!("GPIO interrupt");
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

        let iups = IUPS.borrow(cs);
        iups.set(iups.get() + 1);
    });

}