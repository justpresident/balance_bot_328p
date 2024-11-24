use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::mode::{Floating, Input, Output, PwmOutput};
use arduino_hal::port::Pin;
use embedded_hal::pwm::SetDutyCycle;
use arduino_hal::simple_pwm::PwmPinOps;

pub enum Direction {
    Forward,
    Backwards,
}

pub struct Wheel<T> {
    pub counter: u32,
    pub counter_pin: Pin<Input<Floating>, Dynamic>,
    pub motor: T,
}

pub trait MotorDriver {
    fn control(&mut self, direction: Direction, power: u8);
}

pub struct TB6612<Timer, PinPwm>
where Pin<PwmOutput<Timer>, PinPwm>: SetDutyCycle
    {
    pub pin_a: Pin<Output, Dynamic>,
    pub pin_b: Pin<Output, Dynamic>,
    pub pin_pwm: Pin<PwmOutput<Timer>, PinPwm>,
}


impl<Timer, PinPwm: PwmPinOps<Timer, Duty=u8>> MotorDriver for TB6612<Timer, PinPwm> {
    fn control(&mut self, direction: Direction, power: u8) {
        match direction {
            Direction::Forward => {
                self.pin_a.set_high();
                self.pin_b.set_low();
            },
            Direction::Backwards => {
                self.pin_a.set_low();
                self.pin_b.set_high();
            },
        }
        self.pin_pwm.set_duty_cycle_percent(power).unwrap();
    }
}
