#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
extern crate static_assertions as sa;

use core::cell::RefCell;

use arduino_hal::hal::port::{PB1, PB2, PC0, PC1, PC2};
use arduino_hal::port::mode;
use arduino_hal::port::Pin;
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler};
use arduino_hal::{prelude::*, simple_pwm::Timer1Pwm};
use avr_device::interrupt;
use control::with_control_state_mut;
use drivetrain::{Drivetrain, Encoder, OnePinEncoder, Wheel, TB6612};
use mpu6050_dmp::address::Address;
use mpu6050_dmp::sensor;
mod console;
mod drivetrain;
mod millis;
mod control;

use crate::control::CONTROL_STATE;

// Timer0 is used for tracking time
// Timer1 is used for PWM for TB6612
pub struct Device {
    pub gyro: sensor::Mpu6050<arduino_hal::I2c>,
    pub drivetrain: Drivetrain<OnePinEncoder<mode::Floating>, TB6612<Timer1Pwm,PB2>, OnePinEncoder<mode::Floating>, TB6612<Timer1Pwm,PB1>>,
    pub leds: Leds<PC0, PC1, PC2>,
}

impl Device {
}

pub struct Leds<R,G,B> {
    red: Pin<mode::Output, R>,
    green: Pin<mode::Output, G>,
    blue: Pin<mode::Output, B>,
}

static DEVICE: interrupt::Mutex<RefCell<Option<Device>>> = interrupt::Mutex::new(RefCell::new(None));

#[allow(unused_macros)]
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
#[allow(unused_imports)]
pub(crate) use with_device;

#[allow(unused_macros)]
macro_rules! with_device_mut {
    ($a:ident, $t:block) => {
        {
            interrupt::free(|cs| {
                let mut dev_ref = DEVICE.borrow(cs).borrow_mut();
                let $a = &mut dev_ref.as_mut().unwrap();
                $t
            })
        }
    }
}
#[allow(unused_imports)]
pub(crate) use with_device_mut;
#[allow(unused_macros)]
macro_rules! device {
    ($($t:tt)*) => {
        interrupt::free(|cs| {
            DEVICE.borrow(cs).borrow().as_ref().unwrap().$($t)*
        })
    }
}
#[allow(unused_macros)]
macro_rules! device_mut {
    ($($t:tt)*) => {
        interrupt::free(|cs| {
            DEVICE.borrow(cs).borrow_mut().as_mut().unwrap().$($t)*
        })
    }
}

#[interrupt(atmega328p)]
fn INT0() {
    device_mut!(drivetrain.left_wheel.encoder.tick(););
}


//This function is called on change of pin4
#[interrupt(atmega328p)]
fn PCINT2() {
    device_mut!(drivetrain.right_wheel.encoder.tick(););
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Enable an interrupt for the left wheel encoder
    // We can use normal external interrupt INT0 and configure it as rising edge
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x11));
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());


    // Enable interrupt for the right wheel counter
    // We can only use port level pin-change interrupt PCINT2
    // If we use it for multiple pins on a port we'd not be able to
    // know which pin triggered it
    dp.EXINT.pcicr.write(|w| w.pcie().bits(0b100) );
    // Enable pin change interrupts on PCINT20 which is pin PD4 (= d4)
    dp.EXINT.pcmsk2.write(|w| w.pcint().bits(0b10000));


    control::init(dp.TC2);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    console::set_console(serial);

    //let adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let timer1 = Timer1Pwm::new(dp.TC1, Prescaler::Prescale64);

    // d0 and d1 are RX and TX
    // d3 and d5 are for HC-SR04 distance meter
    let _d3 = pins.d3.into_floating_input();
    //let _d5 = pins.d5.into_floating_input();
    // d11 is for beeper output
    let mut _d11 = pins.d11.into_floating_input().downgrade();

    // Enable TB6612
    let mut d8 = pins.d8.into_output().downgrade();
    d8.set_high();

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );
    let gyro = sensor::Mpu6050::new(i2c, Address::default()).unwrap();

    interrupt::free(|cs| {
        *DEVICE.borrow(cs).borrow_mut() = Some(Device{
            gyro,
            drivetrain: Drivetrain {
                left_wheel : Wheel{
                    encoder: OnePinEncoder::new(
                        pins.d2.into_floating_input().downgrade(),
                    ),
                    motor: TB6612::new(
                        pins.d12.into_output().downgrade(),
                        pins.d13.into_output().downgrade(),
                        pins.d10.into_output().into_pwm(&timer1),
                    ),
                },
                right_wheel : Wheel{
                    encoder: OnePinEncoder::new(
                        pins.d4.into_floating_input().downgrade(),
                    ),
                    motor: TB6612::new(
                        pins.d7.into_output().downgrade(),
                        pins.d6.into_output().downgrade(),
                        pins.d9.into_output().into_pwm(&timer1),
                    ),
                },
            },
            leds: Leds {
                red: pins.a0.into_output(),
                green: pins.a1.into_output(),
                blue: pins.a2.into_output(),
            },
        });
    });

    device_mut!(leds.red.set_high());
    device_mut!(leds.green.set_high());
    device_mut!(leds.blue.set_high());

    millis::init(dp.TC0);

    unsafe { avr_device::interrupt::enable() };

    // Stabilize sensor readings before enabling control
    arduino_hal::delay_ms(2000);
    control::enable();

    loop {
        arduino_hal::delay_ms(300);
        print_debug();
    }
}

#[allow(dead_code)]
pub fn print_debug() {
    let loop_begin = millis::get();
    let last_state = with_control_state_mut!(state, {
        state
    });

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let loop_end = millis::get();

    with_device!(dev, { console::println!("{}",&dev.drivetrain) });
    let accel_val = &last_state.accel_raw;
    let gyro_val = &last_state.gyro_raw;
    console::println!("Accel: {} {} {}", accel_val.x(), accel_val.y(), accel_val.z());
    console::println!("Gyro: {} {} {}", gyro_val.x(), gyro_val.y(), gyro_val.z());
    console::println!("Angles: {} {} {}",
        (last_state.angle_raw*100.0) as i32,
        (last_state.angle_ax_raw*100.0) as i32,
        (last_state.angle_filtered*100.0) as i32,
    );

    console::println!("Control: {} {} {}", last_state.angle_control_output as i32, last_state.speed_control_output as i32, last_state.speed_integral as i32);
    console::println!("Millis: {}, control_interval={}ms, print: {}ms", loop_begin, last_state.control_interval_ms, millis::get() - loop_end);
}

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

