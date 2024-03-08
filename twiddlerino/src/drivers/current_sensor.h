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
#define CUR_SENS_BUFFER_SIZE 1028U

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Init current sensor (connect to adc over I2C protocol)
 *
 */
void current_sensor_init();

/**
 * @brief Returns latest current read in amps, -1 otherwise
 *
 */
double current_sensor_read();

/**
 * @brief Returns latest voltage read in volts, -1 otherwise
 *
 */
double current_sensor_read_voltage();

/**
 * @brief Returns samples per second rate of sensor ADC
 *
 */
uint16_t current_sensor_sps();


#endif // CURRENT_SENSOR_H_