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
use distance::HcSr04;
use drivetrain::{Drivetrain, Encoder, OnePinEncoder, Wheel, TB6612};
use mpu6050_dmp::address::Address;
use mpu6050_dmp::sensor;
mod console;
mod control;
mod distance;
mod drivetrain;
mod millis;

use crate::control::CONTROL_STATE;

// Timer0 is used for tracking time
// Timer1 is used for PWM for TB6612
pub struct Device {
    pub gyro: sensor::Mpu6050<arduino_hal::I2c>,
    pub drivetrain: Drivetrain<OnePinEncoder<mode::Floating>, TB6612<Timer1Pwm,PB2>, OnePinEncoder<mode::Floating>, TB6612<Timer1Pwm,PB1>>,
    pub distance_sensor: HcSr04,
    pub leds: Leds<PC0, PC1, PC2>,
}

impl Device {
}

pub struct Leds<R,G,B> {
    pub red: Pin<mode::Output, R>,
    pub green: Pin<mode::Output, G>,
    pub blue: Pin<mode::Output, B>,
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


// Track previous PIND state to detect which pin changed
static mut PREV_PIND: u8 = 0;
// Right wheel encoder pin mask (PD4)
const ENCODER_RIGHT_MASK: u8 = 1 << 4;
// HC-SR04 echo pin mask (PD5)
const ECHO_PIN_MASK: u8 = 1 << 5;

// This function is called on change of PD4 (encoder) or PD5 (echo)
#[interrupt(atmega328p)]
fn PCINT2() {
    // Read current port state
    let pind = unsafe { (*avr_device::atmega328p::PORTD::ptr()).pind.read().bits() };
    let prev = unsafe { PREV_PIND };
    let changed = pind ^ prev;
    unsafe { PREV_PIND = pind };

    // Right wheel encoder
    if (changed & ENCODER_RIGHT_MASK) != 0 {
        device_mut!(drivetrain.right_wheel.encoder.tick(););
    }

    // HC-SR04 echo pin changed
    if (changed & ECHO_PIN_MASK) != 0 {
        device_mut!(distance_sensor.handle_echo_change(););
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Enable an interrupt for the left wheel encoder
    // We can use normal external interrupt INT0 and configure it as rising edge
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x11));
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());


    // Enable interrupt for the right wheel counter and echo pin
    // We use port level pin-change interrupt PCINT2 for both
    dp.EXINT.pcicr.write(|w| w.pcie().bits(0b100) );
    // Enable pin change interrupts on:
    // - PCINT20 (PD4 = d4) for right wheel encoder
    // - PCINT21 (PD5 = d5) for HC-SR04 echo
    dp.EXINT.pcmsk2.write(|w| w.pcint().bits(0b110000));


    control::init(dp.TC2);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    console::set_console(serial);

    //let adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let timer1 = Timer1Pwm::new(dp.TC1, Prescaler::Prescale64);

    // d0 and d1 are RX and TX
    // d3 (trigger) and d5 (echo) are for HC-SR04 distance meter
    let trigger_pin = pins.d3.into_output().downgrade();
    let echo_pin = pins.d5.into_floating_input().downgrade();
    let distance_sensor = HcSr04::new(trigger_pin, echo_pin);
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
            distance_sensor,
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

    let mut last_debug_print = millis::get();
    loop {
        // Update distance sensor (non-blocking, handles its own timing)
        // Returns Some(distance) when a new measurement is ready
        let maybe_distance = device_mut!(distance_sensor.update());
        if let Some(distance_cm) = maybe_distance {
            control::set_target_speed_from_distance(distance_cm);
        }


        let cur_time = millis::get();
        if  millis::get() - last_debug_print > 300 {
            print_debug();
            last_debug_print = cur_time;
        }

        arduino_hal::delay_ms(10);
    }
}

#[allow(dead_code)]
pub fn print_debug() {

    static mut PRINT_TIME_MS: u32 = 0;
    let loop_begin = millis::get();
    let last_state = with_control_state_mut!(state, {
        state
    });

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

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

    // Distance sensor debug info
    console::println!("Dist: raw={}cm smooth={}cm target={}",
        last_state.last_distance_cm,
        last_state.smoothed_distance as i32,
        last_state.target_speed as i32);

    // SAFETY: static used in only one function in one thread
    console::println!("Millis: {}, control_interval={}ms, print: {}ms", loop_begin, last_state.control_interval_ms, unsafe {PRINT_TIME_MS});
    // TODO: add a fence here 
    let loop_end = millis::get();
    // SAFETY: static used in only one function in one thread
    unsafe {
        PRINT_TIME_MS = loop_end - loop_begin;
    }
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

