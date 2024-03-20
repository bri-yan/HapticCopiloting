/**
 * @file twid32_config.h
 * @brief static default config for twid32
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

#ifndef TWID32_CONFIG_H_
#define TWID32_CONFIG_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//need to import controller config struct definition
#include "app/control/twid_control.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

//command and telemetry config
#define TELEMETRY_QUEUE_SIZE 500U
#define COMMAND_QUEUE_SIZE 10U
#define TELEMETRY_SAMPLES_PER_LOOP 20U

//hardware config
//encoder
#define ENCODER_DEFAULT_FILTER 200
#define ENCODER_VELOCITY_READ_TIMEOUT_US 100000
#define ENCODER_CPR 500

//serial port
#define UART_BAUD_RATE 500000U

//motor
#define MOTOR_MAX_SPEED_DEGS 60945.5207
#define MOTOR_PWM_FREQ 32000U
#define MOTOR_DUTY_CYCLE_RES_BITS 10U
#define MOTOR_DUTY_CYCLE_RES 1024

//macro for struct (partial) init to fill out default controller parameters
//CHANGE CONTROLLER DEFAULT CONFIG HERE!!!!
#define INIT_CONTROLLER_CONFIG(X) controller_config_t X = {\
    .control_type = control_type_t::POSITION_CTRL,\
    .setpoint_type = setpoint_type_t::CONSTANT_SETPOINT_MODE,\
    .init_setpoint = {.pos = 0.0, .vel = 0.0, .accel = 0.0, .torque = 0.0},\
    .controller_direction = controller_direction_t::NEGATIVE_FEEDBACK,\
    .sample_time_us = 1000,\
    .Kp = 0.5, .Ki = 0.01, .Kd = 0.01, .N = 1.0,\
    .velocity_filter_const = 0.01,\
    .current_filter_const = 0.1,\
    .motor_Kv = 0.025, .motor_Ke = 0.011, .motor_J = 1e-6,\
    .impedance = {.K = 1e-3, .B = 0.01, .J = 1e-6},\
    .output_hlim = MOTOR_DUTY_CYCLE_RES, .output_llim = -MOTOR_DUTY_CYCLE_RES\
}


#endif // TWID32_CONFIG_H_