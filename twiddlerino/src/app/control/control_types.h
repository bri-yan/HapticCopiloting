/**
 * @file control_types.h
 * @brief Struct definitions useful for controller
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef TWID_CONTROL_TYPES_H_
#define TWID_CONTROL_TYPES_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include <stdint.h>

#include "Arduino.h"

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/**
 * @brief Control modes
 * 
 */
typedef enum {
    POSITION_CTRL,      //position pid control
    VELOCITY_CTRL,      //velocity pid control
    TORQUE_CTRL,        //torque pid control
    IMPEDANCE_CTRL,     //cascaded impedance control with torque pid control inner loop
    ADMITTANCE_CTRL,    //cascaded admittance control with position pid control inner loop
    NO_CTRL             //no control, telemetry only
} control_type_t;

/**
 * @brief Setpoint type
 * 
 */
typedef enum {
    CONSTANT_SETPOINT_MODE,
    TRAJECTORY_MODE,
    NO_TRAJECTORY_MODE
} setpoint_type_t;

/**
 * @brief Control direction
 * 
 */
typedef enum {
    POSITIVE_FEEDBACK,
    NEGATIVE_FEEDBACK
} controller_direction_t;

/**
 * @brief Virtual impedance params
 * 
 */
typedef struct {
    double K;//stiffness
    double B;//damping
    double J;//intertia
} virtual_impedance_t;

/**
 * @brief Control setpoint
 * 
 */
typedef struct {
    double pos;
    double vel;
    double accel;
    double torque;
} setpoint_t;


/**
 * @brief Control Loop telemetry
 * 
 */
typedef struct {
    //timing
    uint32_t timestamp_ms;
    uint32_t loop_dt;
    uint32_t read_dt;
    uint32_t control_dt;
    uint32_t telemetry_dt;

    //control variables
    setpoint_t setpoint;
    double error;
    double position; //position in deg
    double velocity; //velocity in deg/s
    double filtered_velocity; //low pass filtered velocity
    double current; //in amps
    double filtered_current; //low pass filtered current
    double torque_net; //net torque
    double torque_control; //expected control torque
    double torque_external; //estimated external torque
    double pwm_duty_cycle;
    uint32_t pwm_frequency; //in Hz
    double current_sps; //current sensor ADC samples/second

    //controller params
    double Kp;
    double Ki;
    double Kd;
    virtual_impedance_t impedance;

    //other
    bool pid_success_flag;
} telemetry_t;

/**
 * @brief Controller configuration
 */
typedef struct {
    //telemetry
    QueueHandle_t* telem_queue_handle;

    //config
    control_type_t control_type;
    setpoint_type_t setpoint_type;
    setpoint_t init_setpoint;

    //pid params
    controller_direction_t controller_direction;
    uint32_t sample_time_us;
    double Kp;
    double Ki;
    double Kd;
    double N; //derivative filter coefficient , typically in range of 8 to 20
    double velocity_filter_const; //for velocity ewma filter
    double current_filter_const; //for current ewma filter
    double motor_Kv; //motor damping constant (empircal)
    double motor_Ke; //motor torque constant (empircal)
    double motor_J; //motor intertia Kgm^2 (empircal)
    virtual_impedance_t impedance;

    int32_t output_hlim;
    int32_t output_llim;
} controller_config_t;

#endif // TWID_CONTROL_TYPES_H_