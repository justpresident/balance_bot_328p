use arduino_hal::pac::tc2::tccr2b::CS2_A;
use avr_device::interrupt;
use arduino_hal::clock::Clock;

use crate::console;
use crate::millis;
use crate::with_device_mut;
use crate::DEVICE;

const PRESCALER: u32 = 1024;
const TIMER_COUNTS: u32 = millis::calc_overflow(arduino_hal::DefaultClock::FREQ, 100, PRESCALER);
sa::const_assert!(TIMER_COUNTS < 255);


pub fn init(tc2: arduino_hal::pac::TC2) {
    const CLOCK_SOURCE: CS2_A = CS2_A::PRESCALE_1024;
    // Configure the timer for the above interval (in CTC mode)
    // and enable its interrupt.
    tc2.tccr2a.write(|w| w.wgm2().ctc());
    tc2.ocr2a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc2.tccr2b.write(|w| w.cs2().variant(CLOCK_SOURCE));
    tc2.timsk2.write(|w| w.ocie2a().set_bit());
}

#[avr_device::interrupt(atmega328p)]
fn TIMER2_COMPA() {
    let (_accel_val, _gyro_val) = with_device_mut!(dev, {
        (dev.gyro.accel().unwrap(), dev.gyro.gyro().unwrap())
    });
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    console::print!(".");
}
