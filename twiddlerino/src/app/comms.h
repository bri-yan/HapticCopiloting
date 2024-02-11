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

    //state variables
    double set_point;
    double position;
    double velocity;
    double current;
    double torque_external;
    double pwm_duty_cycle;

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

typedef union {
    telemetry_t telemetry_struct;
    char buffer[sizeof(telemetry_t)];
} telemetry_packet_t;


/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//initializes Twiddlerino
cmd_type_t decode_cmd(String *, test_config_t *);

uint32_t publish_telemetry(telemetry_t *);

uint32_t publish_telemetry_raw(telemetry_t *);

String read_string_until(char terminator);

#endif //COMMS_H