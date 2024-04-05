/**
 * @file comms.h
 * @brief communications over serial
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef COMMS_H_
#define COMMS_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include "app/control/control_types.h"
#include "app/control/twid_control.h"

//for int types
#include <stdint.h>

#include "Arduino.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/**
 * @brief Command Types
 * 
 */
typedef enum {
    NA_CMD = -1,
    STOP = 1,
    RESET= 2,
    REBOOT= 3,
    TELEM_ENABLE= 4,
    TELEM_DISABLE= 5,
    SET_SETPOINT= 6,
    SET_DUTYCYCLE= 7,
    SET_PID= 8,
    SET_IMPEDANCE= 9,
    SET_MODE= 10,
    SET_TELEMSAMPLERATE= 11,
    SET_MULTISETPOINT_POSITION = 12,
    SET_MULTISETPOINT_VELOCITY = 13,
    SET_MULTISETPOINT_ACCELERATION = 14,
    SET_MULTISETPOINT_TORQUE = 15,
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


/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Tries to interpret string as a command and executes the command
 *        Returns cmd_type_t > 0 if the command was intereted and executed successfully
 *        Otherwise cmd_type_t < 0
 */
cmd_type_t handle_command(String *);

/**
 * @brief read controller config strings, will return false if string is invalid/not recognized, 
            otherwise will fill config struct
 */
cmd_type_t decode_config_cmd(String *, controller_config_t *);

/**
 * @brief Send ack message for cmd_type
 * 
 */
void ack_cmd(cmd_type_t);

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

/**
 * @brief Print controller config over serial port (if it is open)
 * 
 */
uint32_t print_controller_cfg(twid_controller_t*);

// read string until terminator
String read_string_until(char terminator);

// extract doubles from string
void extract_doubles(String *, double*, uint16_t);

void reset_sent_count();

#endif //COMMS_H_