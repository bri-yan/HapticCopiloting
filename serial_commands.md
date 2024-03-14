Several commands to set parameters over the serial interface and control flow.
Each command is a newline terminated string.

`"stop\n"` - Stops controller (telemetry also stops)

`"reset\n"` - Resets controller (configuration reverts to default)

`"set_pid,<double>,<double>,<double>,\n"` - Set P, I, and D values of PID controller

`"set_impedance,<double>,<double>,<double>,\n"` - Set stiffness, damping, and inertia values of impedane/admittance controller

`"set_mode,<string>,\n"` - Sets control mode, the string values represents an enum. Refer to the `control_type_t` struct definition in `twiddlerino/src/app/control/twid_control.h`.
* modes: `position`, `velocity`, `torque`, `impedance`, `admittance`

`"set_setpoint,<double>,<double>,<double>,<double>,\n"` - Set position, velocity, acceleration and torque setpoints for the controller.