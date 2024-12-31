use core::cell::RefCell;
use arduino_hal::pac::tc2::tccr2b::CS2_A;
use avr_device::interrupt;
use arduino_hal::clock::Clock;
use mpu6050_dmp::accel::Accel;
use mpu6050_dmp::gyro::Gyro;
use core::mem;

use crate::millis;
use crate::with_device_mut;
use crate::DEVICE;

const PRESCALER: u32 = 1024;
const UPDATE_FREQ: u32 = 100;
const TIMER_COUNTS: u32 = millis::calc_overflow(arduino_hal::DefaultClock::FREQ, UPDATE_FREQ, PRESCALER);
sa::const_assert!(TIMER_COUNTS <= 255);

pub struct ControlState {
    pub last_millis: u32,
    pub control_time: u32,
    pub accel_raw: Accel,
    pub gyro_raw: Gyro,
    pub angle_raw: f32,
    pub angle_ax_raw: f32,
}

pub static CONTROL_STATE: interrupt::Mutex<RefCell<mem::MaybeUninit<ControlState>>> = interrupt::Mutex::new(RefCell::new(mem::MaybeUninit::uninit()));

#[allow(unused_macros)]
macro_rules! with_control_state_mut {
    ($a:ident, $t:block) => {
        {
            interrupt::free(|cs| {
                let binding = CONTROL_STATE.borrow(cs);
                let mut state_cell = binding.borrow_mut();
                let $a = unsafe { state_cell.as_mut_ptr().as_mut().unwrap() };
                $t
            })
        }
    }
}
#[allow(unused_imports)]
pub(crate) use with_control_state_mut;

pub fn init(tc2: arduino_hal::pac::TC2) {
    const CLOCK_SOURCE: CS2_A = CS2_A::PRESCALE_1024;
    // Configure the timer for the above interval (in CTC mode)
    // and enable its interrupt.
    tc2.tccr2a.write(|w| w.wgm2().ctc());
    tc2.ocr2a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc2.tccr2b.write(|w| w.cs2().variant(CLOCK_SOURCE));
    tc2.timsk2.write(|w| w.ocie2a().set_bit());

    avr_device::interrupt::free(|cs| {
        let binding = CONTROL_STATE.borrow(cs);
        let mut state_cell = binding.borrow_mut();
        *state_cell = mem::MaybeUninit::new(ControlState{
            last_millis: 0,
            control_time: 0,
            accel_raw : Accel::new(0, 0, 0),
            gyro_raw : Gyro::new(0, 0, 0),
            angle_raw: 0.0,
            angle_ax_raw: 0.0,
        })
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER2_COMPA() {
    let (accel_val, gyro_val) = with_device_mut!(dev, {
        (dev.gyro.accel().unwrap(), dev.gyro.gyro().unwrap())
    });
    let last_millis = millis::get();
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    let angle = libm::atan2f(accel_val.y().into(), accel_val.z().into()) * 57.3;
    let angle_ax = libm::atan2f(accel_val.x().into(), accel_val.z().into()) * 57.3;

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let control_time = millis::get() - last_millis;
    with_control_state_mut!(state, {
        state.angle_raw = angle;
        state.angle_ax_raw = angle_ax;
        state.accel_raw = accel_val;
        state.gyro_raw = gyro_val;
        state.last_millis = last_millis;
        state.control_time = control_time;
    });
}
