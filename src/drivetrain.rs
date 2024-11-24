use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::mode::{Floating, Input, Output, PwmOutput};
use arduino_hal::port::Pin;
use embedded_hal::pwm::SetDutyCycle;
use arduino_hal::simple_pwm::PwmPinOps;
use ufmt::uDisplay;

pub enum Direction {
    Forward,
    Backwards,
}

pub struct Drivetrain<L,R>
where L:uDisplay, R:uDisplay {
    pub left_wheel: Wheel<L>,
    pub right_wheel: Wheel<R>,
}

impl<L: uDisplay, R:uDisplay> uDisplay for Drivetrain<L,R>
{
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        ufmt::uwrite!(f, "L: {}, R: {}", self.left_wheel, self.right_wheel)
    }
}

pub struct Wheel<T> {
    pub counter: u32,
    pub counter_pin: Pin<Input<Floating>, Dynamic>,
    pub motor: T,
}

impl<T:uDisplay> uDisplay for Wheel<T> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        ufmt::uwrite!(f, "{}, counter: {}", self.motor, self.counter)
    }
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

impl<Timer,PinPwm: PwmPinOps<Timer, Duty=u8>> uDisplay for TB6612<Timer,PinPwm> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        let state = match (self.pin_a.is_set_high(), self.pin_b.is_set_high()) {
            (false, false) => "Free",
            (false, true) => "Backwards",
            (true, false) => "Forward",
            (true, true) => "Brakes",
        };
        ufmt::uwrite!(f, "{}({})", state, self.pin_pwm.get_duty())
    }
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
