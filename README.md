# HapticCopiloting
Codebase for our ENPH 459 Capstone Project: Haptic Co-Piloting.

1 DOF haptic device featuring a BDC motor control system in live feedback with a virtual environment.

## Directories
- twiddlerino: C source code for the Twiddlerino's ESP32
- sim: MATLAB source code for modelling and simualtion of motor, sensors and controller

## System Diagram

![alt text](system_diagram.png "System Diagram")

## Firmware Dev Environment

Embedded development environment: [Platformio](https://platformio.org/) IDE plugin for VSCode

[Install platformio](https://platformio.org/)

config file: `twiddlerino/platformio.ini`

### Firmware source directory structure
`twiddlerino\src\.`

├───app
│   ├───control
│   └───filter
└───drivers

## Communication System (Serial)
### Telemetry

[Serial Studio](https://serial-studio.github.io/), an open source telemetry GUI, is used to record and visualize system data during testing of the controller.

Serial data frames are defined in a .JSON configuration file, see `twiddlerino/serial-studio-dashboard.json`.

The firmware running on the ESP32 periodically sends telemetry strings over a serial port. The strings are formatted as serial studio frames: `"/*TWIDDLERINO_TELEMETRY,%s,%s,...,%s*/"`.

![alt text](telemetry_gui.png "Telemetry GUI")

### Command
Several commands are available to set parameters over the serial interface and to control the state of the controller.
**Each command is a newline terminated string.**

`"stop\n"` - Stops controller (telemetry also stops)

`"reset\n"` - Resets controller (configuration reverts to default)

`"telemetry_enable\n"` - Enable controller telemetry stream over serial (rate==`TELEMETRY_SAMPLES_PER_LOOP `)

`"telemetry_disable\n"` - Disable controller telemetry stream over serial (rate==`TELEMETRY_SAMPLES_PER_LOOP `)

`"set_pid,<double>,<double>,<double>,\n"` - Set P, I, and D values of PID controller

`"set_impedance,<double>,<double>,<double>,\n"` - Set stiffness, damping, and inertia values of impedane/admittance controller

`"set_mode,<string>,\n"` - Sets control mode, the string values represents an enum. Refer to the `control_type_t` struct definition in `twiddlerino/src/app/control/twid_control.h`.
* modes: `position`, `velocity`, `torque`, `impedance`, `admittance`, `no_control`

`"set_setpoint,<double>,<double>,<double>,<double>,\n"` - Set position, velocity, acceleration and torque setpoints for the controller. 

`"set_dutycycle,<int>,\n"` - Sets motor duty cycle directly regardless of controller running.