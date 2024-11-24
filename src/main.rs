#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::RefCell;

use arduino_hal::hal::port::{PB1, PB2};
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler};
use arduino_hal::{prelude::*, simple_pwm::Timer1Pwm};
use arduino_hal::adc::channel;
use avr_device::interrupt;
use console::println;
use drivetrain::{Drivetrain, Wheel, TB6612};

mod console;
mod drivetrain;

pub struct Device {
    pub drivetrain: Drivetrain<TB6612<Timer1Pwm,PB2>, TB6612<Timer1Pwm,PB1>>,
}

impl Device {
}

static DEVICE: interrupt::Mutex<RefCell<Option<Device>>> = interrupt::Mutex::new(RefCell::new(None));

macro_rules! with_device {
    ($a:ident, $t:block) => {
        {
            interrupt::free(|cs| {
                let dev_ref = DEVICE.borrow(cs).borrow();
                let &$a = &dev_ref.as_ref().unwrap();
                $t
            })
        }
    }
}
macro_rules! device {
    ($($t:tt)*) => {
        interrupt::free(|cs| {
            DEVICE.borrow(cs).borrow().as_ref().unwrap().$($t)*
        })
    }
}
macro_rules! device_mut {
    ($($t:tt)*) => {
        interrupt::free(|cs| {
            DEVICE.borrow(cs).borrow_mut().as_mut().unwrap().$($t)*
        })
    }
}

#[interrupt(atmega328p)]
fn INT0() {
    device_mut!(drivetrain.left_wheel.counter += 1);
}


//This function is called on change of pin 2
#[interrupt(atmega328p)]
fn PCINT2() {
    device_mut!(drivetrain.right_wheel.counter += 1);
}

#[arduino_hal::entry]
fn main() -> ! {
    // TODO: 
    //  1. Move all the logic from main into methods of Device
    //  2. Make dp and pins members of device
    //  3. Implement display trait for all Subdevices and print them instead of raw pin values
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Enable interrupt for the left wheel counter
    // We can only use port level pin-change interrupt PCINT2
    // If we use it for multiple pins on a port we'd not be able to
    // know which pin triggered it
    dp.EXINT.pcicr.write(|w| w.pcie().bits(0b100) );
    // Enable pin change interrupts on PCINT20 which is pin PD4 (= d4)
    dp.EXINT.pcmsk2.write(|w| w.pcint().bits(0b10000));

    // Enable interrupt for the right wheel encoder
    // We can use normal external interrupt INT0
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x01));
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());


    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    console::set_console(serial);


    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let a0 = pins.a0.into_analog_input(&mut adc);
    let a1 = pins.a1.into_analog_input(&mut adc);
    let a2 = pins.a2.into_analog_input(&mut adc);
    let a3 = pins.a3.into_analog_input(&mut adc);
    let a4 = pins.a4.into_analog_input(&mut adc);
    let a5 = pins.a5.into_analog_input(&mut adc);


    let timer1 = Timer1Pwm::new(dp.TC1, Prescaler::Prescale64);
    // d0 and d1 are RX and TX
    let d2 = pins.d2.into_floating_input().downgrade();
    let _d3 = pins.d3.into_floating_input();
    let d4 = pins.d4.into_floating_input().downgrade();
    let _d5 = pins.d5.into_floating_input();
    let d6 = pins.d6.into_output().downgrade();
    let d7 = pins.d7.into_output().downgrade();
    let mut d8 = pins.d8.into_output().downgrade();
    let mut d9 = pins.d9.into_output().into_pwm(&timer1);
    let mut d10 = pins.d10.into_output().into_pwm(&timer1);
    let _d11 = pins.d11.into_floating_input();
    let d12 = pins.d12.into_output().downgrade();
    let d13 = pins.d13.into_output().downgrade();

    // Enable TB6612
    d8.set_high();

    // Enable pwm on pins
    d9.enable();
    d10.enable();

    interrupt::free(|cs| {
        *DEVICE.borrow(cs).borrow_mut() = Some(Device{
            drivetrain: Drivetrain {
                left_wheel : Wheel{
                    counter : 0,
                    counter_pin : d2,
                    motor: TB6612 {
                        pin_a: d12,
                        pin_b: d13,
                        pin_pwm: d10,
                    },
                },
                right_wheel : Wheel{
                    counter: 0,
                    counter_pin : d4,
                    motor: TB6612 {
                        pin_a: d7,
                        pin_b: d6,
                        pin_pwm: d9,
                    },
                },
            },
        });
    });
    unsafe { avr_device::interrupt::enable() };

    loop {
        console::println!("");
        let values = [
            a0.analog_read(&mut adc),
            a1.analog_read(&mut adc),
            a2.analog_read(&mut adc),
            a3.analog_read(&mut adc),
            a4.analog_read(&mut adc),
            a5.analog_read(&mut adc),
        ];

        for (i, v) in values.iter().enumerate() {
            console::print!("A{}: {}\t", i, v);
        }
        console::println!("");

        let (vbg, gnd, tmp, adc6, adc7) = (
            adc.read_blocking(&channel::Vbg),
            adc.read_blocking(&channel::Gnd),
            adc.read_blocking(&channel::Temperature),
            adc.read_blocking(&channel::ADC6),
            adc.read_blocking(&channel::ADC7),
        );
        console::println!("ADC6: {}, ADC7: {}", adc6, adc7);
        console::println!("Vbandgap: {}, Ground: {}, Temperature: {}", vbg, gnd, tmp);
        with_device!(dev, { println!("{}",&dev.drivetrain) });
        arduino_hal::delay_ms(300);
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
        ufmt::uwriteln!(&mut serial,
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

