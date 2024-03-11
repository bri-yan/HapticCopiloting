/**
 * @file comms.h
 * @brief communications over serial
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef COMMS_H
#define COMMS_H

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include "app/control/twid_control.h"

//for int types
#include <stdint.h>

#include "Arduino.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define READ_TIMEOUT_US 200U

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

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
 * @brief Command Types
 * 
 */
typedef enum {
    CONFIG_TEST,
    START_TEST,
    ABORT_TEST,
    NA_CMD = -1
} cmd_type_t;

/**
 * @brief Test Config
 * 
 */
typedef struct {
    cmd_type_t cmd_type;

    //timing
    uint32_t sample_rate_us;
    int32_t test_duration_ms;

    double Kp;
    double Ki;
    double Kd;
    double set_point;
} test_config_t;

/**
 * @brief Packet for raw telemetry
 * 
 */
typedef union {
    telemetry_t telemetry_struct;
    char buffer[sizeof(telemetry_t)];
} telemetry_packet_t;


/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//initializes Twiddlerino
cmd_type_t decode_cmd(String *, test_config_t *);

/**
 * @brief Publish comma seperated telemetry string (utf-8) over serial
 * 
 */
uint32_t publish_telemetry(telemetry_t *);

/**
 * @brief Publish (utf-8) string encoded to be read by serial studio telemetry app
 * 
 */
uint32_t publish_telemetry_serial_studio(telemetry_t *telem);

// /**
//  * @brief Publish raw bytes over serials (saves space)
//  * 
//  */
// uint32_t publish_telemetry_raw(telemetry_t *);

String read_string_until(char terminator);

#endif //COMMS_H