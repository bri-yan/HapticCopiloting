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

//for int types
#include <stdint.h>

#include "Arduino.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define MOTOR_PWM_FREQ 32000U

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/**
 * @brief Twiddlerino Startup State
 * 
 */
typedef enum {
    IDLE,
    RUN_CONTROLLER_DEFAULT,
    RUN_TELEMETRY_ONLY,
    RUN_AWAIT_COMMANDS,
    RUN_TCONTROL_DEFAULT
} startup_type_t;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//initializes Twiddlerino
void twiddlerino_setup(startup_type_t startup_type = startup_type_t::IDLE);

#endif //TWIDDLERINO_H_

