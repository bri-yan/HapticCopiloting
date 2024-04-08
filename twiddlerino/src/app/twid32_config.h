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
// #include "app/control/control_types.h"

#include "twid32_pin_defs.h"

#include <FreeRTOSConfig.h>

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

//twid ids
#define TWID1_ID "CONTROL_ONE"
#define TWID2_ID "CONTROL_TWO"

//command and telemetry config
#define ENABLE_DEBUG_TELEMETRY_ON_INIT true
#define TELEMETRY_QUEUE_SIZE 500U
#define TELEMETRY_DEFAULT_SAMPLE_RATE 20U //telemetry sample rate in (control loops)/sample
#define FREE_RTOS_TELEM_SAMPLE_RATE 5000U //MS

#define ENABLE_SECOND_CONTROLLER 1

//hardware config
//encoder
#define ENCODER_DEFAULT_FILTER 200
#define ENCODER_CPR 500

//serial port
#define UART_BAUD_RATE 1000000U

//motor
#define MOTOR1_PWM_CHAN_DEFAULT 0
#define MOTOR2_PWM_CHAN_DEFAULT 2


//macro for struct (partial) init to fill out default controller parameters
//CHANGE CONTROLLER DEFAULT CONFIG HERE!!!!
#define INIT_CONTROLLER_CONFIG_PARTIAL(X) controller_config_t X = {\
    .control_type = control_type_t::POSITION_CTRL,\
    .setpoint_type = setpoint_type_t::CONSTANT_SETPOINT_MODE,\
    .init_setpoint = {.pos = 0.0, .vel = 0.0, .accel = 0.0, .torque = 0.0},\
    .controller_direction = controller_direction_t::NEGATIVE_FEEDBACK,\
    .sample_time_us = 1000,\
    .Kp = 0.5, .Ki = 0.01, .Kd = 0.01, .N = 1.0,\
    .velocity_filter_const = 0.01,\
    .current_filter_const = 0.1,\
    .motor_Kv = 2E-5, .motor_Ke = 0.009901879421645426, .motor_J = 1e-6,\
    .impedance = {.K = 1e-3, .B = 0.01, .J = 1e-6},\
    .output_hlim = MOTOR_DUTY_CYCLE_RES, .output_llim = -MOTOR_DUTY_CYCLE_RES,\
    .telemetry_sample_rate = TELEMETRY_DEFAULT_SAMPLE_RATE,\
}

//freertos config
#define CORE_CONTROL_TASK 1U
#define CORE_SENSOR_TASK 1U
#define CORE_SERIAL_READ_TASK 0U
#define CORE_SERIAL_WRITE_TASK 0U

#define TASK_PRIORITY_CONTROL (configMAX_PRIORITIES - 1)
#define TASK_PRIORITY_TELEMETRY (configMAX_PRIORITIES - 2)

#endif // TWID32_CONFIG_H_