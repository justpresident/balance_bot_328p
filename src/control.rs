use core::cell::RefCell;
use core::i16;
use arduino_hal::pac::tc2::tccr2b::CS2_A;
use avr_device::interrupt;
use arduino_hal::clock::Clock;
use mpu6050_dmp::accel::Accel;
use mpu6050_dmp::gyro::Gyro;
use core::mem;

use crate::drivetrain::Encoder;
use crate::millis;
use crate::with_device_mut;
use crate::DEVICE;

const CONTROL_INTERVAL_MS: u32 = 5;
const SPEED_CONTROL_INTERVAL_MS: u32 = 40;
const PRESCALER: u32 = 1024;
const UPDATE_FREQ: u32 = 70;
const TIMER_COUNTS: u32 = millis::calc_overflow(arduino_hal::DefaultClock::FREQ, UPDATE_FREQ, PRESCALER);
sa::const_assert!(TIMER_COUNTS <= 255);

pub struct ControlState {
    enabled: bool,
    pub update_interval: u32,
    pub control_interval: u32,
    pub last_update: u32,
    pub last_control: u32,
    // Raw sensor readings
    pub accel_raw: Accel,
    pub gyro_raw: Gyro,
    // Angles computed from raw accelerometer readings
    pub angle_raw: f32,
    pub angle_ax_raw: f32,
    // Gyro rates computed from raw gyro readings
    pub gyro_rate_x_raw: f32,
    pub gyro_rate_y_raw: f32,
    pub gyro_rate_z_raw: f32,

    kalman_filter: KalmanFilter,
    // Filtered angle value
    pub angle_filtered: f32,
    pub speed_counter: f32, // speed in ticks/ms
    pub cur_speed: f32,
    pub angle_control_output: f32,
    pub speed_control_output: f32,

    // Add speed integral for proper PI control
    speed_integral: f32,
    // Add previous angle for derivative calculation
    prev_angle: f32,
}

pub static CONTROL_STATE: interrupt::Mutex<RefCell<mem::MaybeUninit<ControlState>>> =
                          interrupt::Mutex::new(RefCell::new(mem::MaybeUninit::uninit()));

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
            enabled: false,
            update_interval: 0,
            control_interval: 0,
            last_update: 0,
            last_control: 0,
            accel_raw : Accel::new(0, 0, 0),
            gyro_raw : Gyro::new(0, 0, 0),
            angle_raw: 0.0,
            angle_ax_raw: 0.0,
            gyro_rate_x_raw: 0.0,
            gyro_rate_y_raw: 0.0,
            gyro_rate_z_raw: 0.0,
            kalman_filter: KalmanFilter::new(),
            angle_filtered: 0.0,
            speed_counter: 0.0,
            cur_speed: 0.0,
            angle_control_output: 0.0,
            speed_control_output: 0.0,
            speed_integral: 0.0,
            prev_angle: 0.0,
        })
    });
}

pub fn clamp<T: PartialOrd>(val: T, min: T, max: T) -> T {
    if val < min {
        min
    } else if val > max {
        max
    } else {
        val
    }
}

struct KalmanFilter {
    q_angle: f32,
    q_bias: f32,
    r_measure: f32,
    angle: f32,
    bias: f32,
    rate: f32,
    p: [[f32; 2]; 2],
}

impl KalmanFilter {
    fn new() -> Self {
        KalmanFilter {
            q_angle: 0.001,
            q_bias: 0.005,
            r_measure: 0.5,
            angle: 0.0,
            bias: 0.0,
            rate: 0.0,
            p: [[1.0, 0.0], [0.0, 1.0]],
        }
    }

    fn get_angle(&mut self, new_angle: f32, new_rate: f32, dt: f32) -> f32 {
        // Clamp dt to reasonable bounds
        let dt = clamp(dt, 0.001, 0.1);

        // Predict
        self.rate = new_rate - self.bias;
        self.angle += dt * self.rate;

        // Update covariance matrix
        self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_angle);
        self.p[0][1] -= dt * self.p[1][1];
        self.p[1][0] -= dt * self.p[1][1];
        self.p[1][1] += self.q_bias * dt;

        // Update step
        let s = self.p[0][0] + self.r_measure;

        // Avoid division by zero
        if -1e6 < s && s < 1e-6 {
            return self.angle;
        }

        let k = [self.p[0][0] / s, self.p[1][0] / s];

        let y = new_angle - self.angle;
        self.angle += k[0] * y;
        self.bias += k[1] * y;

        let p00_temp = self.p[0][0];
        let p01_temp = self.p[0][1];

        self.p[0][0] -= k[0] * p00_temp;
        self.p[0][1] -= k[0] * p01_temp;
        self.p[1][0] -= k[1] * p00_temp;
        self.p[1][1] -= k[1] * p01_temp;

        // Clamp angle to reasonable range
        self.angle = clamp(self.angle, -90.0, 90.0);

        self.angle
    }
}

pub fn enable() {
    with_control_state_mut!(state, {
        state.enabled = true;
        // Reset integrators when enabling
        state.speed_integral = 0.0;
        state.kalman_filter.angle = state.angle_raw;  // Initialize filter with current angle
    });
}

fn stop() {
    with_device_mut!(dev, {
        dev.drivetrain.left_wheel.brake();
        dev.drivetrain.right_wheel.brake();
    });
}

fn update_angle_control(state: &mut ControlState, _dt: f32) {
    const KP_BALANCE: f32 = 55.0;
    const KD_BALANCE: f32 = 0.75;
    const ANGLE_ZERO: f32 = -0.17;
    const ANGULAR_VELOCITY_ZERO: f32 = 0.0;


    // PD control for angle
    state.angle_control_output = KP_BALANCE * (state.angle_filtered - ANGLE_ZERO)
        + KD_BALANCE * (state.gyro_rate_x_raw - ANGULAR_VELOCITY_ZERO);

    state.prev_angle = state.angle_filtered;
}

fn update_speed_control(state: &mut ControlState, dt: f32) {
    const KP_SPEED: f32 = 2.0;
    const KI_SPEED: f32 = 0.05;
    const MAX_INTEGRAL: f32 = 100.0; // Integral windup protection

    // Low-pass filter for speed
    state.cur_speed = state.cur_speed * 0.7 + (state.speed_counter * dt) * 0.3;

    // Update integral with windup protection
    state.speed_integral += state.cur_speed * dt;
    state.speed_integral = clamp(state.speed_integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    state.speed_control_output = KP_SPEED * state.cur_speed + KI_SPEED * state.speed_integral;

    // Clamp output
    state.speed_control_output = clamp(state.speed_control_output, -3550.0, 3550.0)
}

fn balance(state: &mut ControlState, dt: f32) {
    if !state.enabled {
        return;
    }

    // Get angle control output
    update_angle_control(state, dt);
    // Get speed control output
    update_speed_control(state, dt);

    // Combine angle and speed control
    let pwm_left = state.angle_control_output - state.speed_control_output;
    let pwm_right = state.angle_control_output - state.speed_control_output;

    // Safety limits - stop if angle is too extreme
    if state.angle_filtered < -30.0 || state.angle_filtered > 30.0 {
        stop();
        // Reset integrators when stopped
        state.speed_integral = 0.0;
    } else {
        // Clamp PWM values to reasonable range
        let pwm_left = clamp(pwm_left as i16, -255, 255);
        let pwm_right = clamp(pwm_right as i16, -255, 255);

        with_device_mut!(dev, {
            dev.drivetrain.left_wheel.control(pwm_left);
            dev.drivetrain.right_wheel.control(pwm_right);
        });
    }
}

#[avr_device::interrupt(atmega328p)]
fn TIMER2_COMPA() {
    let cur_millis = millis::get();
    let (accel_val, gyro_val, speed_counter) = with_device_mut!(dev, {

        (
            dev.gyro.accel().unwrap(),
            dev.gyro.gyro().unwrap(),
            dev.drivetrain.left_wheel.encoder.get_speed(SPEED_CONTROL_INTERVAL_MS)
                + dev.drivetrain.right_wheel.encoder.get_speed(SPEED_CONTROL_INTERVAL_MS),
        )
    });

    // Convert to degrees - check if this conversion is correct for your sensor orientation
    let angle = libm::atan2f(accel_val.y().into(), accel_val.z().into()) * 57.295779;
    let angle_ax = libm::atan2f(accel_val.x().into(), accel_val.z().into()) * 57.295779;

    // Gyro conversion - verify these offsets and scale factors for your specific MPU6050
    let gyro_rate_x_raw = (gyro_val.x() as f32 - 128.1) / 131.0;
    let gyro_rate_y_raw = gyro_val.y() as f32 / 131.0;
    let gyro_rate_z_raw = (-1.0 * gyro_val.z() as f32) / 131.0;

    with_control_state_mut!(state, {
        state.angle_raw = angle;
        state.angle_ax_raw = angle_ax;
        state.accel_raw = accel_val;
        state.gyro_raw = gyro_val;
        state.gyro_rate_x_raw = gyro_rate_x_raw;
        state.gyro_rate_y_raw = gyro_rate_y_raw;
        state.gyro_rate_z_raw = gyro_rate_z_raw;
        state.speed_counter = speed_counter;

        // Only run control loop at specified interval
        if cur_millis - state.last_control < CONTROL_INTERVAL_MS {
            return;
        }

        state.control_interval = cur_millis - state.last_control;
        state.last_control = cur_millis;
        let dt = (state.control_interval as f32) * 0.001;

        // Update Kalman filter
        state.angle_filtered = state.kalman_filter.get_angle(state.angle_raw, state.gyro_rate_x_raw, dt);

        // Run balance control
        balance(state, dt);
    });
}
