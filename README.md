# Helicopter control
﻿This project is an implementation of a control system for a coaxial helicopter. It relies on a remote radio controller and collects data from multiple sensors which are saved locally and streamed to the controller.
﻿
# Hardware requirements
The major part of the source code is platform-dependent and hardware specific but it can be easily ported to other environments. The main boards are:
 - **Waveshare Pico Zero**, based on Raspberry Pi microcontroller RP2040, which is highly compatible with Raspberry Pi Pico
 - **RF-Nano**, an integration the official Arduino Nano board with the nRF24L01+ chip for radio transmission
 
Below is the list of the hardware components used in the project. Circuit schematics can be found [here](https://github.com/FedericoRubbi/helicopter_control/blob/master/wirings/circuit.png).

## Hardware components
| Function | Component name |
|--|--|
| Helicopter controller | Waveshare Pico Zero |
| Accelerometer | WT901B |
| Ultrasonic sensor | GY-US42 |
| RF module | NRF24L01+ |
| Data logger | OpenLog Datalogger |
| BEC | IFlight Micro BEC |
| ESC | DRV8833 |
| Squash plate servos | Esky 8g servo |
| Motors | Mabuchi FK-180 SH |
| Remote receiver | RF-Nano |

# Software requirements
The raspberry Pi foundation provides a [setup guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) to the environment and a complete [documentation of the SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) for the Pico Zero.
To build the project the Pico SDK, CMake, a  cross-platform tool used to build the software, and the GNU Embedded Toolchain for Arm are required.
Run the following commands to setup the dependecies:

    $ git clone https://github.com/FedericoRubbi/helicopter_control.git
    $ cd helicopter_control/
    $ git submodule update --init
    $ cd lib/pico-sdk/
    $ git submodule update --init
    $ sudo apt update
    $ sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
    $ sudo apt install libstdc++-arm-none-eabi-newlib			# only for Debian and Ubuntu
    
**Note**    

Make sure to add compile option `-DPICO_BOARD=waveshare_rp2040_zero` to select the right board configuration.
If using Microsoft Visual Code editor add `"cmake.configureArgs": ["-DPICO_BOARD=waveshare_rp2040_zero",]`  to the project file *.vscode/settings.json* by opening the command palette (CTRL+SHIFT+P) and selecting "*Preferences: Open Workspace Settings (JSON)*", then install CMake extension and  select kit "*GCC 10.3.1 arm-none-abi*" from the shortcut on the status bar.

To program the RF-Nano follow the installation of the Arduino-IDE with no further specific configuration on the  [official site](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE) and install RF24 library with the library manager.

    
# Source code organization
The main programs to be uploaded on transmitter and receiver are in order in *./src/* and *./receiver/*, while transmitter device libraries and interfaces are in *./lib/*.
## Program layout
```
├── lib                  # pico device libraries
│   ├── control          # physical control interface
│   ├── ins              # inertial navigation system
│   ├── pico-sdk
│   ├── RF24             # transmitter radio communication library
│   ├── test_module      # testing utilities
│   └── transmitter      # transmitter interface
├── remote               # remote receiver files
│   ├── receiver
│   └── src
├── scripts
├── src                  # main program
└── wirings
```

# Building and flashing

To build the project the command `make` is sufficient and the fastest method to load software onto the Pico Zero is by mounting it as a USB Mass Storage Device, dragging a file onto the board to program the flash. To do so connect it using a USB-C cable, hold down the *BOOTSEL* button first, then the *RESET* button and release them in the same order.
The script *./scripts/launch.sh* may speed up the process of copying the builded file on the board and opening the serial.

To build and upload the arduino sketch on the receiver RF-Nano board no specific configuration is needed, just press compile and upload buttons.

# Presentation document and video

# Project contributors
The project was developed by [Chiara Marangoni](https://github.com/chiamara02) and [Federico Rubbi](https://github.com/FedericoRubbi) for the course Embedded software for the IoT at the University of Trento.


