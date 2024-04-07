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

#include "Adafruit_ADS1X15.h"

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/**
 * @brief Motor States
 * 
 */
typedef enum {
    CURRENT_SENSOR_1 = 0,
    CURRENT_SENSOR_2 = 1
} curr_sens_adc_channel_t;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Init current sensor (connect to adc over I2C protocol)
 *        Starts a background task to periodically read external ADC register over i2c.
 */
void current_sensor_init();

double current_sensor_get_volts(curr_sens_adc_channel_t);

double current_sensor_get_current(curr_sens_adc_channel_t);

uint16_t current_sensor_sps();


#endif // CURRENT_SENSOR_H_