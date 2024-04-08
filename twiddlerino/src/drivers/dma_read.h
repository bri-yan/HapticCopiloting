#ifndef DMA_READ_H_
#define DMA_READ_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/


/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

void dma_config();

bool dma_read(uint32_t* reading, adc_channel_t chann);

#endif //