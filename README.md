# Helicopter control
This project is an implementation of a control model for a coaxial helicopter. It makes use of a custom control algorithm which collects data from multiple sensors and set-points from a remote controller. Data is saved locally and streamed to the remote controller.
A 3D real-time visualizer software is also provided.

## Hardware components
| Function | Component name |
|--|--|
| Helicopter controller | Waveshare pico-zero |
| Accelerometer | WT901B |
| Ultrasonic sensor | GY-US42 |
| RF module | NRF24L01+ SMD |
| Data logger | Sparkfun OpenLog |
| BEC | IFlight Micro BEC |
| ESC | Dual-way brushed ESC |
| Squash plate servos | Esky 8g servo |
| Motors | FK-180 SH |

## TODO:

 - [ ] Setup RF transmissions
 - [ ] Sensors communication
 - [ ] Actuators communication
 - [ ] Write to logger
 - [ ] Control algorithm
 - [ ] 3D visualizer

> Written with [StackEdit](https://stackedit.io/).
