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

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

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
} ads115_adc_channel_t;

typedef struct {
    adc_channel_t channel;

    double last_reading_volts;
    double zero;
    bool is_zeroed;

    //adc calibration characteristics here, filled upon init
    esp_adc_cal_characteristics_t adc_cal;
} curr_sens_adc_context_t;

extern curr_sens_adc_context_t *curr_sens_1_handle;
extern curr_sens_adc_context_t *curr_sens_2_handle;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Init current sensor (connect to adc over I2C protocol)
 *        Starts a background task to periodically read external ADC register over i2c.
 */
void ads115_sens_init();

double ads115_get_volts(ads115_adc_channel_t);

double current_sensor_get_current(ads115_adc_channel_t);

uint16_t ads115_sens_get_sps();

/**
 * @brief Calibrate and init internal adc to read analog current sensor values
 */
void curr_sens_adc_init();

//volts in mv 
double curr_sens_adc_get_volts(curr_sens_adc_context_t* handle);

void curr_sens_adc_zero(curr_sens_adc_context_t* handle);

double curr_sens_convert_current(curr_sens_adc_context_t* handle, double volts);

#endif // CURRENT_SENSOR_H_