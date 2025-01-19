# balance_bot_328p

Rust implementation of Self-Balancing bot with atmega328p from AliExpress ![image](./image.jpg)

## *This project is not complete yet. Contributions are welcome!*

### What is implemented
* Devices defined and can be controlled/monitored
  
  * Gyroscope and Accelerometer - raw values can be read and plotted in realtime usind `./rave | ./scripts/plot_all.py` command
 
  * Motors - can be controlled to rotate Forward, Backwards and Brake
 
  * Encoders - On the stock board only 1 encoder pin is connected from each wheel, which allows to count rotations, but not direction.

### What is not implemented
* Using kalman filter to smooth measured values for Gyroscope and Accelerometer.
* Estimation of the device angle(axis X) using Accelerometer and Gyroscope. Here is a good article about it http://www.kerrywong.com/2012/03/08/a-self-balancing-robot-i/
* Implementing PID control loop for actually balancing the robot

## Build Instructions
1. Install prerequisites as described in the [`avr-hal` README] (`avr-gcc`, `avr-libc`, `avrdude`, [`ravedude`]).

2. Run `cargo build` to build the firmware.

3. Run `./rave flash` to flash the firmware to a connected board.  If `ravedude`
   fails to detect your board, check its documentation at
   <https://crates.io/crates/ravedude>.

4. `ravedude` will open a console session after flashing where you can interact
   with the UART console of your board.

[`avr-hal` README]: https://github.com/Rahix/avr-hal#readme
[`ravedude`]: https://crates.io/crates/ravedude

## License
Licensed under either of

 - Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
 - MIT license
   ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

