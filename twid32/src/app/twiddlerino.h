/**
 * @file twiddlerino.h
 * @brief Twiddlerino main file
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef TWIDDLERINO_H_
#define TWIDDLERINO_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include <stdint.h>

//initializes Twiddlerino
void twiddlerino_setup();

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//for int types
#include <stdint.h>

#include "Arduino.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define MOTOR_PWM_FREQ 32000U
#define MOTOR_DUTY_CYCLE_RES 8

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
} control_telemetry_t;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//initializes Twiddlerino
void twiddlerino_setup();

char* encode_telemetry(control_telemetry_t *telemtry);

#endif //TWIDDLERINO_H_

