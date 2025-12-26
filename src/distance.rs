use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::mode::{Floating, Input, InputMode, Output};
use arduino_hal::port::Pin;

use crate::millis;

/// Type alias for common floating input configuration
pub type HcSr04 = DistanceSensor<Floating>;

/// HC-SR04 Ultrasonic Distance Sensor
pub struct DistanceSensor<T: InputMode> {
    trigger_pin: Pin<Output, Dynamic>,
    pub echo_pin: Pin<Input<T>, Dynamic>,
    // Measurement state
    measuring: bool,
    echo_start_ms: u32,
    last_distance_cm: Option<u16>,
    last_trigger_ms: u32,
    // Debug counters
    pub trigger_count: u16,
    pub rising_edge_count: u16,
    pub falling_edge_count: u16,
    pub last_duration_ms: u32,
}

impl<T: InputMode> DistanceSensor<T> {
    pub fn new(
        trigger_pin: Pin<Output, Dynamic>,
        echo_pin: Pin<Input<T>, Dynamic>,
    ) -> Self {
        Self {
            trigger_pin,
            echo_pin,
            measuring: false,
            echo_start_ms: 0,
            last_distance_cm: None,
            last_trigger_ms: 0,
            trigger_count: 0,
            rising_edge_count: 0,
            falling_edge_count: 0,
            last_duration_ms: 0,
        }
    }

    /// Call this from the pin-change interrupt when echo pin changes.
    /// Uses is_high() to read the current pin state.
    pub fn handle_echo_change(&mut self) {
        if !self.measuring {
            return;
        }

        let now = millis::get();

        if self.echo_pin.is_high() {
            // Rising edge - echo started
            self.echo_start_ms = now;
            self.rising_edge_count += 1;
        } else if self.echo_start_ms > 0 {
            // Falling edge - echo ended
            let duration_ms = now.wrapping_sub(self.echo_start_ms);
            self.last_duration_ms = duration_ms;
            self.falling_edge_count += 1;

            // Calculate distance: duration_ms * 17 ≈ distance in cm
            // (speed of sound: 1ms ≈ 17cm round trip)
            let distance_cm = if duration_ms == 0 {
                2u16 // Very close, < 1ms
            } else {
                (duration_ms * 17) as u16
            };

            if distance_cm >= 2 && distance_cm <= 400 {
                self.last_distance_cm = Some(distance_cm);
            }

            self.measuring = false;
            self.echo_start_ms = 0;
        }
    }

    /// Check if it's time for a new measurement and send trigger if so.
    /// Returns true if a trigger was sent.
    pub fn maybe_trigger(&mut self) -> bool {
        let now = millis::get();

        // Don't trigger if already measuring or too soon since last trigger
        if self.measuring {
            // Timeout check - reset if measuring for too long
            if now.wrapping_sub(self.last_trigger_ms) > 50 {
                self.measuring = false;
            }
            return false;
        }

        if now.wrapping_sub(self.last_trigger_ms) < 60 {
            return false;
        }

        // Send 10µs trigger pulse
        self.trigger_pin.set_low();
        arduino_hal::delay_us(2);
        self.trigger_pin.set_high();
        arduino_hal::delay_us(10);
        self.trigger_pin.set_low();

        self.measuring = true;
        self.last_trigger_ms = now;
        self.trigger_count += 1;

        true
    }

    /// Get the last measured distance in cm, if any.
    pub fn last_distance(&self) -> Option<u16> {
        self.last_distance_cm
    }

    /// Check if the echo pin is currently high.
    pub fn is_echo_high(&self) -> bool {
        self.echo_pin.is_high()
    }

    /// Update the sensor - triggers measurement if needed.
    /// Returns Some(distance_cm) when a new measurement is available.
    pub fn update(&mut self) -> Option<u16> {
        // Try to trigger a new measurement
        self.maybe_trigger();

        // Return last distance and clear it (so we only report each measurement once)
        self.last_distance_cm.take()
    }

    /// Get debug info: (trigger_count, rising_edge_count, falling_edge_count, last_duration_ms)
    pub fn debug_info(&self) -> (u16, u16, u16, u32) {
        (
            self.trigger_count,
            self.rising_edge_count,
            self.falling_edge_count,
            self.last_duration_ms,
        )
    }
}
