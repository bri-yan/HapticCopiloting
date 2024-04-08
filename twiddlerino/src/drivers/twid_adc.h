/**
 * @file twid_adc.h
 * @brief Continous adc using dma
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

#ifndef TWID_ADC_H_
#define TWID_ADC_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//for int types
#include <stdint.h>

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc_cal.h"

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

void twid_adc_init();

bool twid_adc_get_raw(adc_channel_t channel, int32_t* raw);

void twid_adc_raw_to_millivolts(int32_t raw, int32_t* volts);

#endif // TWID_ADC_H_