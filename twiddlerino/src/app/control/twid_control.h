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

//macro for struct (partial) init to fill out default controller parameters
//CHANGE CONTROLLER DEFAULT CONFIG HERE!!!!
#define INIT_CONTROLLER_CONFIG(X) controller_config_t X = {\
    .control_type = control_type_t::POSITION_CTRL,\
    .setpoint_type = setpoint_type_t::CONSTANT_SETPOINT_MODE,\
    .controller_direction = controller_direction_t::NEGATIVE_FEEDBACK,\
    .sample_time_us = 1000,\
    .Kp = 0.5, .Ki = 0.01, .Kd = 0.01, .N = 1.0,\
    .velocity_filter_const = 0.01,\
    .current_filter_const = 0.01,\
    .torque_Kv = 0.025,\
    .torque_Ke = 0.011,\
    .impedance = {.K = 1e-3, .B = 0.01, .J = 1e-6},\
    .output_hlim = 255, .output_llim = -255\
}

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


typedef enum {
    POSITIVE_FEEDBACK,
    NEGATIVE_FEEDBACK
} controller_direction_t;

//struct to contain virtual impedance params
typedef struct {
    double K;//stiffness
    double B;//damping
    double J;//intertia
} virtual_impedance_t;

typedef struct {
    //telemetry
    QueueHandle_t* telem_queue_handle;

    //config
    control_type_t control_type;
    setpoint_type_t setpoint_type;

    //pid params
    controller_direction_t controller_direction;
    uint32_t sample_time_us;
    double Kp;
    double Ki;
    double Kd;
    double N; //derivative filter coefficient , typically in range of 8 to 20
    double velocity_filter_const; //for velocity ewma filter
    double current_filter_const; //for current ewma filter
    double torque_Kv; //motor damping constant (empircal)
    double torque_Ke; //motor torque constant (empircal)
    virtual_impedance_t impedance;

    int32_t output_hlim;
    int32_t output_llim;
} controller_config_t;

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

void tcontrol_update_tunings(double kp, double ki, double kd, controller_direction_t dir);
void tcontrol_update_tunings(controller_config_t* config);

double tcontrol_get_kp();
double tcontrol_get_ki();
double tcontrol_get_kd();

#endif //TWID_CONTROL_H_