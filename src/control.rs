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
const UPDATE_FREQ: u32 = 100;
const TIMER_COUNTS: u32 = millis::calc_overflow(arduino_hal::DefaultClock::FREQ, UPDATE_FREQ, PRESCALER);
sa::const_assert!(TIMER_COUNTS <= 255);

pub struct ControlState {
    enabled: bool,
    pub control_interval_ms: u32,
    pub last_control_ms: u32,
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
    pub speed_counter: i32, // speed in ticks/ms
    pub cur_speed: f32,
    pub angle_control_output: f32,
    pub speed_control_output: f32,

    // Add speed integral for proper PI control
    pub speed_integral: f32,
    // Add previous angle for derivative calculation
    prev_angle: f32,

    // Distance-based speed control
    pub target_speed: f32,
    pub last_distance_cm: u16,
    pub smoothed_distance: f32,  // Exponential moving average of distance
    // Configurable thresholds and speeds (can be tuned at runtime)
    pub distance_close_threshold: u16,   // cm - drive backwards if closer
    pub distance_medium_threshold: u16,  // cm - drive forwards if between close and medium
    pub speed_backward: f32,             // target speed when too close
    pub speed_forward: f32,              // target speed when in medium range

    // Fallen state tracking - triggers reset on recovery
    fallen: bool,
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

const fn get_timer2_prescaler() -> CS2_A {
    match PRESCALER {
        8 => CS2_A::PRESCALE_8,
        32 => CS2_A::PRESCALE_32,
        64 => CS2_A::PRESCALE_64,
        128 => CS2_A::PRESCALE_128,
        256 => CS2_A::PRESCALE_256,
        1024 => CS2_A::PRESCALE_1024,
        _ => panic!(),
    }
}

pub fn init(tc2: arduino_hal::pac::TC2) {
    // Configure the timer for the above interval (in CTC mode)
    // and enable its interrupt.
    tc2.tccr2a.write(|w| w.wgm2().ctc());
    tc2.ocr2a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc2.tccr2b.write(|w| w.cs2().variant(get_timer2_prescaler()));
    tc2.timsk2.write(|w| w.ocie2a().set_bit());

    avr_device::interrupt::free(|cs| {
        let binding = CONTROL_STATE.borrow(cs);
        let mut state_cell = binding.borrow_mut();
        *state_cell = mem::MaybeUninit::new(ControlState{
            enabled: false,
            control_interval_ms: 0,
            last_control_ms: 0,
            accel_raw : Accel::new(0, 0, 0),
            gyro_raw : Gyro::new(0, 0, 0),
            angle_raw: 0.0,
            angle_ax_raw: 0.0,
            gyro_rate_x_raw: 0.0,
            gyro_rate_y_raw: 0.0,
            gyro_rate_z_raw: 0.0,
            kalman_filter: KalmanFilter::new(),
            angle_filtered: 0.0,
            speed_counter: 0,
            cur_speed: 0.0,
            angle_control_output: 0.0,
            speed_control_output: 0.0,
            speed_integral: 0.0,
            prev_angle: 0.0,
            // Distance-based speed control defaults
            target_speed: 0.0,
            last_distance_cm: 0,
            smoothed_distance: 100.0,  // Start with "far" assumption
            distance_close_threshold: 20,   // < 20cm = backwards
            distance_medium_threshold: 60,  // 20-60cm = forwards
            speed_backward: -5.0,
            speed_forward: 5.0,
            // Fallen state
            fallen: false,
        })
    });
}

/// Trigger a CPU reset using the watchdog timer.
/// This performs a full hardware reset, clearing all state.
fn software_reset() -> ! {
    avr_device::interrupt::disable();

    unsafe {
        let wdt = &(*avr_device::atmega328p::WDT::ptr());
        // Enable watchdog change enable (WDCE) and watchdog enable (WDE)
        wdt.wdtcsr.write(|w| w.wdce().set_bit().wde().set_bit());
        // Set WDE with minimum timeout (16ms), clear WDCE
        wdt.wdtcsr.write(|w| w.wde().set_bit());
    }

    // Wait for watchdog to trigger reset
    loop {
        core::hint::spin_loop();
    }
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

/// Set target speed based on measured distance.
/// - distance < close_threshold: drive backwards (Blue LED)
/// - close_threshold <= distance < medium_threshold: drive forwards (Green LED)
/// - distance >= medium_threshold: stay stationary (Red LED)
pub fn set_target_speed_from_distance(distance_cm: u16) {
    // EMA smoothing factor (0.0-1.0, lower = more smoothing)
    const DISTANCE_ALPHA: f32 = 0.25;

    // Determine mode and set target speed
    let mode = with_control_state_mut!(state, {
        state.last_distance_cm = distance_cm;

        // Apply exponential moving average smoothing
        state.smoothed_distance = DISTANCE_ALPHA * (distance_cm as f32)
            + (1.0 - DISTANCE_ALPHA) * state.smoothed_distance;

        let smoothed = state.smoothed_distance as u16;

        if smoothed < state.distance_close_threshold {
            // Too close - drive backwards
            state.target_speed = state.speed_backward;
            0u8 // backwards
        } else if smoothed < state.distance_medium_threshold {
            // Medium range - drive forwards
            state.target_speed = state.speed_forward;
            1u8 // forwards
        } else {
            // Far enough - stay stationary
            state.target_speed = 0.0;
            2u8 // stationary
        }
    });

    // Set LED color based on mode (accent LED is active-low: low = on)
    with_device_mut!(dev, {
        match mode {
            0 => {
                // Backwards - Blue
                dev.leds.red.set_high();
                dev.leds.green.set_high();
                dev.leds.blue.set_low();
            }
            1 => {
                // Forwards - Green
                dev.leds.red.set_high();
                dev.leds.green.set_low();
                dev.leds.blue.set_high();
            }
            _ => {
                // Stationary - Red
                dev.leds.red.set_low();
                dev.leds.green.set_high();
                dev.leds.blue.set_high();
            }
        }
    });
}

fn stop() {
    with_device_mut!(dev, {
        dev.drivetrain.left_wheel.brake();
        dev.drivetrain.right_wheel.brake();
    });
}

fn update_angle_control(state: &mut ControlState, _dt: f32) {
    const KP_BALANCE: f32 = 23.0;
    const KD_BALANCE: f32 = 0.48;
    const ANGLE_ZERO: f32 = -0.17;
    const ANGULAR_VELOCITY_ZERO: f32 = 0.0;


    // PD control for angle
    state.angle_control_output = KP_BALANCE * (state.angle_filtered - ANGLE_ZERO)
        + KD_BALANCE * (state.gyro_rate_x_raw - ANGULAR_VELOCITY_ZERO);

    state.prev_angle = state.angle_filtered;
}

fn update_speed_control(state: &mut ControlState, dt: f32) {
    const KP_SPEED: f32 = 7.52;
    const KI_SPEED: f32 = 12.398;
    const MAX_INTEGRAL: f32 = 3550.0; // Integral windup protection

    // Low-pass filter for speed
    state.cur_speed = state.cur_speed * 0.7 + state.speed_counter as f32  * 0.3;

    // Speed error: difference between current speed and target speed
    let speed_error = state.cur_speed - state.target_speed;

    // Update integral with windup protection
    state.speed_integral += speed_error * dt;
    state.speed_integral *= 0.99;
    state.speed_integral = clamp(state.speed_integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    state.speed_control_output = KP_SPEED * speed_error + KI_SPEED * state.speed_integral;
}

fn balance(state: &mut ControlState, dt: f32) {
    if !state.enabled {
        return;
    }

    const FALLEN_THRESHOLD: f32 = 30.0;
    const RECOVERY_THRESHOLD: f32 = 10.0;

    // Get angle control output
    update_angle_control(state, dt);
    // Get speed control output
    update_speed_control(state, dt);

    let is_fallen = state.angle_filtered < -FALLEN_THRESHOLD || state.angle_filtered > FALLEN_THRESHOLD;
    let is_upright = state.angle_filtered > -RECOVERY_THRESHOLD && state.angle_filtered < RECOVERY_THRESHOLD;

    if is_fallen {
        // Robot has fallen - stop motors and mark as fallen
        stop();
        state.speed_integral = 0.0;
        state.fallen = true;
    } else if state.fallen && is_upright {
        // Robot was fallen but is now upright - trigger full CPU reset
        // This ensures a clean state when the robot is placed back on surface
        stop();
        software_reset();
    } else {
        // Combine angle and speed control
        let pwm_left = state.angle_control_output - state.speed_control_output;
        let pwm_right = state.angle_control_output - state.speed_control_output;

        // Normal operation - apply motor control
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
        if cur_millis - state.last_control_ms < CONTROL_INTERVAL_MS {
            return;
        }

        state.control_interval_ms = cur_millis - state.last_control_ms;
        state.last_control_ms = cur_millis;
        let dt = (state.control_interval_ms as f32) * 0.001;

        // Update Kalman filter
        state.angle_filtered = state.kalman_filter.get_angle(state.angle_raw, state.gyro_rate_x_raw, dt);

        // Run balance control
        balance(state, dt);
    });
}
