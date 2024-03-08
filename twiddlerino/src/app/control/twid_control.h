/**
 * @file twid_control.h
 * @brief Control class for implementing the various controllers for the twiddlerino
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 *         Based on PID_v1 library but modified to use timer interrupts
 */

#ifndef TWID_CONTROL_H_
#define TWID_CONTROL_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include "Arduino.h"
#include "app/comms.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

typedef enum {
    POSITION_CTRL,
    VELOCITY_CTRL,
    TORQUE_CTRL,
    IMPEDANCE_CTRL,
    ADMITTANCE_CTRL
} control_type_t;

typedef enum {
    CONSTANT_SETPOINT_MODE,
    TRAJECTORY_MODE,
    NO_TRAJECTORY_MODE
} setpoint_type_t;

typedef struct {
    //telemetry
    QueueHandle_t* telem_queue_handle;

    //config
    control_type_t control_type;
    setpoint_type_t setpoint_type;
    uint32_t sample_time_us;
    double Kp;
    double Ki;
    double Kd;
    double torque_gain;
    double velocity_filter_const;
    double torque_Kv;
    double torque_Ke;

    int32_t output_hlim;
    int32_t output_llim;
} controller_config_t;

//macro for struct (partial) init to fill out default controller parameters
#define INIT_CONTROLLER_CONFIG(X) controller_config_t X = {\
    .control_type = control_type_t::POSITION_CTRL,\
    .setpoint_type = setpoint_type_t::CONSTANT_SETPOINT_MODE,\
    .sample_time_us = 1000,\
    .Kp = 0.5, .Ki = 0.0, .Kd = 0.0,\
    .torque_gain = 0.0,\
    .velocity_filter_const = 0.01,\
    .torque_Kv = 0.001,\
    .torque_Ke = 0.001,\
    .output_hlim = 255, .output_llim = -255\
    }

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

void tcontrol_configure(controller_config_t*);

void tcontrol_start();

void tcontrol_stop();

void tcontrol_reset();

bool tcontrol_is_running();

// void tcontrol_get_telem(telemetry_t*);

void tcontrol_update_setpoint(double);

void tcontrol_update_tunings(double, double, double);

#endif //TWID_CONTROL_H_