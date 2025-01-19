
use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::mode::{Input, InputMode, Output, PwmOutput};
use arduino_hal::port::Pin;
use arduino_hal::simple_pwm::PwmPinOps;
use embedded_hal::digital::InputPin;
use embedded_hal::pwm::SetDutyCycle;
use ufmt::{uDisplay,derive::uDebug};


#[derive(uDebug)]
pub enum Direction {
    Free,
    Forward,
    Backwards,
    Brakes,
}

pub struct Drivetrain<EL, L, ER, R>
where EL:Encoder+uDisplay, L:uDisplay, ER:Encoder+uDisplay, R:uDisplay {
    pub left_wheel: Wheel<EL,L>,
    pub right_wheel: Wheel<ER,R>,
}

impl<EL:Encoder+uDisplay, L: uDisplay, ER:Encoder+uDisplay, R:uDisplay> uDisplay for Drivetrain<EL, L, ER, R>
{
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        ufmt::uwrite!(f, "L: {}, R: {}", self.left_wheel, self.right_wheel)
    }
}

pub struct Wheel<ET,T>
where ET: Encoder + uDisplay {
    pub encoder: ET,
    pub motor: T,
}

impl<ET:Encoder+uDisplay, T:MotorDriver+uDisplay> Wheel<ET,T> {
    pub fn control(&mut self, power: i16) {
        if power >= 0 {
            self.encoder.set_direction(Direction::Backwards);
            self.motor.control_raw(Direction::Backwards, power as u16);
        } else {
            self.encoder.set_direction(Direction::Forward);
            self.motor.control_raw(Direction::Forward, (-1*power) as u16);
        }
    }
    pub fn brake(&mut self) {
        self.motor.control_raw(crate::drivetrain::Direction::Brakes, 0);
        self.motor.control_raw(crate::drivetrain::Direction::Brakes, 0);
    }
}

impl<ET:Encoder+uDisplay,T:uDisplay> uDisplay for Wheel<ET,T> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        ufmt::uwrite!(f, "{}, encoder: {}", self.motor, self.encoder)
    }
}

pub trait Encoder{
    fn tick(&mut self);
    fn set_direction(&mut self, direction: Direction);
}

pub struct OnePinEncoder<T> {
    pub pin: Pin<Input<T>, Dynamic>,
    pub direction: Direction,
    pub counter: i32,
}

impl<T> OnePinEncoder<T>
where T: InputMode
{
    pub fn new(pin: Pin<Input<T>, Dynamic>) -> Self {
        Self { pin, direction:Direction::Forward, counter: 0}
    }
}
impl<T> Encoder for OnePinEncoder<T>
where T:InputMode
{
    fn tick(&mut self) {
        if !self.pin.is_high() {
            return;
        }
        match self.direction {
            Direction::Forward => self.counter += 1,
            Direction::Backwards => self.counter -= 1,
            _ => {},
        }
    }

    fn set_direction(&mut self, direction: Direction) {
        self.direction = direction;
    }
}
impl<T> uDisplay for OnePinEncoder<T> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        ufmt::uwrite!(f, "{:?}, counter={}", self.direction, self.counter)
    }
}

pub struct TwoPinEncoder<T> {
    pub pin_a: Pin<Input<T>, Dynamic>,
    pub pin_b: Pin<Input<T>, Dynamic>,
    pub direction: Direction,
    pub counter: i32,
    pub inversed: bool,
}

impl<T> TwoPinEncoder<T>
where T: InputMode
{
    #[allow(dead_code)]
    pub fn new(pin_a: Pin<Input<T>, Dynamic>, pin_b: Pin<Input<T>, Dynamic>, inversed: bool) -> Self {
        Self { pin_a, pin_b, direction:Direction::Forward, counter: 0, inversed }
    }
}
impl<T> Encoder for TwoPinEncoder<T>
where T: InputMode
{
    fn tick(&mut self) {
        let mut b = self.pin_b.is_high();
        let a = self.pin_a.is_high();
        if !a {
            return;
        }

        if self.inversed {
            b = !b;
        }

        self.direction = match b {
            false => {
                self.counter += 1;
                Direction::Forward
            },
            true => {
                self.counter -= 1;
                Direction::Backwards
            },
        };
    }

    fn set_direction(&mut self, _direction: Direction) {
        // Encoder identifies its direction automatically
    }
}

impl<T> uDisplay for TwoPinEncoder<T> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        ufmt::uwrite!(f, "{:?}, counter={}", self.direction, self.counter)
    }
}


pub trait MotorDriver {
    #[allow(dead_code)]
    fn control_raw(&mut self, direction: Direction, power: u16);
}

pub struct TB6612<Timer, PinPwm>
where Pin<PwmOutput<Timer>, PinPwm>: SetDutyCycle
    {
    pub pin_a: Pin<Output, Dynamic>,
    pub pin_b: Pin<Output, Dynamic>,
    pub pin_pwm: Pin<PwmOutput<Timer>, PinPwm>,
}

impl<Timer, PinPwm> TB6612<Timer, PinPwm>
where Pin<PwmOutput<Timer>, PinPwm>: SetDutyCycle, PinPwm: PwmPinOps<Timer>
{
    pub fn new(pin_a: Pin<Output, Dynamic>, pin_b: Pin<Output, Dynamic>, mut pin_pwm: Pin<PwmOutput<Timer>, PinPwm>) -> Self {
        pin_pwm.enable();
        Self { pin_a, pin_b, pin_pwm }
    }
}

impl<Timer,PinPwm: PwmPinOps<Timer, Duty=u8>> uDisplay for TB6612<Timer,PinPwm> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        let state = match (self.pin_a.is_set_high(), self.pin_b.is_set_high()) {
            (false, false) => Direction::Free,
            (false, true) => Direction::Backwards,
            (true, false) => Direction::Forward,
            (true, true) => Direction::Brakes,
        };
        ufmt::uwrite!(f, "{:?}({})", state, self.pin_pwm.get_duty())
    }
}

impl<Timer, PinPwm: PwmPinOps<Timer, Duty=u8>> MotorDriver for TB6612<Timer, PinPwm> {
    fn control_raw(&mut self, direction: Direction, mut power: u16) {
        if power > 255 {
            power = 255;
        }
        match direction {
            Direction::Free => {
                self.pin_a.set_low();
                self.pin_b.set_low();
            },
            Direction::Forward => {
                self.pin_a.set_high();
                self.pin_b.set_low();
            },
            Direction::Backwards => {
                self.pin_a.set_low();
                self.pin_b.set_high();
            },
            Direction::Brakes => {
                self.pin_a.set_high();
                self.pin_b.set_high();
            }
        }
        self.pin_pwm.set_duty_cycle_fraction(power, 255).unwrap();
    }
}
