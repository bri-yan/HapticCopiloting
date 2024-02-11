/**
 * @file current_sensor.h
 * @brief Current sensing driver, using adc
 * @author Gavin Pringle and Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

#ifndef CURRENT_SENSOR_H_
#define CURRENT_SENSOR_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//for int types
#include <stdint.h>


/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define SHUNT_RESISTOR_OHM 1U

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//initializes motor
void current_sensor_init();

double current_sensor_read();

double current_sensor_read_voltage();


#endif // CURRENT_SENSOR_H_