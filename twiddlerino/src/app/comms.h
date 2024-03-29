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

//deprecated - read test commands
cmd_type_t decode_test_cmd(String *, test_config_t *);

// read controller config strings, will return false if string is invalid/not recognized, 
// otherwise will fill config struct
bool decode_config_cmd(String *, controller_config_t *);

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
uint32_t print_controller_cfg();

// read string until terminator
String read_string_until(char terminator);

// extract doubles from string
void extract_doubles(String *, double*, uint16_t);

#endif //COMMS_H_