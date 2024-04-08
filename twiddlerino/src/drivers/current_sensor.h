/**
 * @file current_sensor.h
 * @brief Current sensing driver, using adc
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

#ifndef CURRENT_SENSOR_H_
#define CURRENT_SENSOR_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//for int types
#include <stdint.h>

#include "drivers/twid_adc.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/


/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

typedef struct {
    adc_channel_t channel;

    int32_t last_raw;

    bool is_zeroed;
    double zero_volts;

    double last_volts;
    double last_amps;
} current_sens_contex_t;


/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

extern current_sens_contex_t current_sens_1_handle;
extern current_sens_contex_t current_sens_2_handle;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

void current_sensor_init_all();

bool current_sensor_get_volts(current_sens_contex_t*, double*);

bool current_sensor_get_amps(current_sens_contex_t*, double*);

void current_sensor_volts_to_amps(double, double, double*);



#endif // CURRENT_SENSOR_H_