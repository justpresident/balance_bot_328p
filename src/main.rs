#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::mode::{Floating, Input};
use arduino_hal::port::Pin;
use arduino_hal::prelude::*;
use arduino_hal::adc::channel;

pub struct Wheel {
    pub counter: u32,
    pub counter_pin: Pin<Input<Floating>, Dynamic>,
}

pub struct Device {
    pub left_wheel: Wheel,
    pub right_wheel: Wheel,
}

impl Device {
}

static mut LEFT_COUNTER: i32 = 0;

//This function is called on change of pin 2
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT2() {
    unsafe {
        LEFT_COUNTER += 1;
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Enable the PCINT2 pin change interrupt
    dp.EXINT.pcicr.write(|w| unsafe { w.bits(0b100) });
    // Enable pin change interrupts on PCINT18 which is pin PD2 (= d2)
    dp.EXINT.pcmsk2.write(|w| w.bits(0b100));

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let mut led = pins.d13.into_output();

    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());


    let a0 = pins.a0.into_analog_input(&mut adc);
    let a1 = pins.a1.into_analog_input(&mut adc);
    let a2 = pins.a2.into_analog_input(&mut adc);
    let a3 = pins.a3.into_analog_input(&mut adc);
    let a4 = pins.a4.into_analog_input(&mut adc);
    let a5 = pins.a5.into_analog_input(&mut adc);


    // d0 and d1 are RX and TX
    let d2 = pins.d2.into_floating_input().downgrade();
    let d3 = pins.d3.into_floating_input();
    let d4 = pins.d4.into_floating_input().downgrade();
    let d5 = pins.d5.into_floating_input();
    let d6 = pins.d6.into_floating_input();
    let d7 = pins.d7.into_floating_input();
    let d8 = pins.d8.into_floating_input();
    let d9 = pins.d9.into_floating_input();
    let d10 = pins.d10.into_floating_input();
    let d11 = pins.d11.into_floating_input();
    let d12 = pins.d12.into_floating_input();

    let device = Device{
        left_wheel : Wheel{counter : 0, counter_pin : d2},
        right_wheel : Wheel{counter: 0, counter_pin : d4},
    };

    unsafe { avr_device::interrupt::enable() };

    loop {
        ufmt::uwriteln!(&mut serial, "" ).unwrap_infallible();
        let values = [
            a0.analog_read(&mut adc),
            a1.analog_read(&mut adc),
            a2.analog_read(&mut adc),
            a3.analog_read(&mut adc),
            a4.analog_read(&mut adc),
            a5.analog_read(&mut adc),
        ];

        for (i, v) in values.iter().enumerate() {
            ufmt::uwrite!(&mut serial, "A{}: {}\t", i, v).unwrap_infallible();
        }
        ufmt::uwriteln!(&mut serial, "" ).unwrap_infallible();


        let dvalues = [
            device.left_wheel.counter_pin.is_high(),
            d3.is_high(),
            device.right_wheel.counter_pin.is_high(),
            d5.is_high(),
            d6.is_high(),
            d7.is_high(),
            d8.is_high(),
            d9.is_high(),
            d10.is_high(),
            d11.is_high(),
            d12.is_high(),
        ];

        for (i, v) in dvalues.iter().enumerate() {
            ufmt::uwrite!(&mut serial, "D{}: {}\t", i+2, v).unwrap_infallible();
        }
        ufmt::uwriteln!(&mut serial, "" ).unwrap_infallible();

        let (vbg, gnd, tmp, adc6, adc7) = (
            adc.read_blocking(&channel::Vbg),
            adc.read_blocking(&channel::Gnd),
            adc.read_blocking(&channel::Temperature),
            adc.read_blocking(&channel::ADC6),
            adc.read_blocking(&channel::ADC7),
        );
        ufmt::uwriteln!(&mut serial, "ADC6: {}, ADC7: {}", adc6, adc7).unwrap_infallible();


        ufmt::uwriteln!(&mut serial, "Vbandgap: {}", vbg).unwrap_infallible();
        ufmt::uwriteln!(&mut serial, "Ground: {}", gnd).unwrap_infallible();
        ufmt::uwriteln!(&mut serial, "Temperature: {}", tmp).unwrap_infallible();
        unsafe {
            ufmt::uwriteln!(&mut serial, "Left counter: {}", LEFT_COUNTER).unwrap_infallible();
        }
        led.set_low();
        arduino_hal::delay_ms(300);
        led.set_high();
    }
}

#[cfg(not(doc))]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // disable interrupts - firmware has panicked so no ISRs should continue running
    avr_device::interrupt::disable();

    // get the peripherals so we can access serial and the LED.
    //
    // SAFETY: Because main() already has references to the peripherals this is an unsafe
    // operation - but because no other code can run after the panic handler was called,
    // we know it is okay.
    let dp = unsafe { arduino_hal::Peripherals::steal() };
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    // Print out panic location
    ufmt::uwriteln!(&mut serial, "Firmware panic!\r").unwrap_infallible();
    if let Some(loc) = info.location() {
        ufmt::uwriteln!(
            &mut serial,
            "  At {}:{}:{}\r",
            loc.file(),
            loc.line(),
            loc.column(),
        )
        .unwrap_infallible();
    }

    // Blink LED rapidly
    let mut led = pins.d13.into_output();
    loop {
        led.toggle();
        arduino_hal::delay_ms(100);
    }
}

