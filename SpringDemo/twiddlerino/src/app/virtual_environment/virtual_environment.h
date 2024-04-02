/**
 * @file virtual_environment.h
 * @brief Code related to haptic virtual environment interface
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

#ifndef VIRTUAL_ENVIRONMENT_H_
#define VIRTUAL_ENVIRONMENT_H_

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
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Process a game command string and return true if successfull
 * 
 */
bool game_process_command(String *);

/**
 * @brief Start game interface to send data back
 * 
 */
void game_interface_init();

void game_interface_stop();

void game_interface_restart();

#endif // VIRTUAL_ENVIRONMENT_H_