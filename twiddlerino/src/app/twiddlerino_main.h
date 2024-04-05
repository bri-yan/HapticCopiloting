/**
 * @file twiddlerino_main.h
 * @brief Twiddlerino main file
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef TWIDDLERINO_MAIN_H_
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

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//initializes Twiddlerino
void twiddlerino_setup();

void enable_telemetry_publisher();

void disable_telemetry_publisher();

#endif //TWIDDLERINO_H_

