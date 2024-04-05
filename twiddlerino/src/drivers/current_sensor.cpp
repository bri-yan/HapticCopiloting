/**
 * @file current_sensor.h
 * @brief Current sensing driver, using adc
 * @author Gavin Pringle and Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 *   https://www.ti.com/lit/ds/symlink/ads1115.pdf?ts=1711845972450&ref_url=https%253A%252F%252Fwww.google.com%252F
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "drivers/current_sensor.h"

#include "app/twid32_pin_defs.h"
#include "app/twid32_config.h"

//ads drivers
#include "Adafruit_ADS1X15.h"
#include "SPI.h"
#include "esp32-hal-i2c.h"
#include "esp32-hal-i2c-slave.h"

#include "freertos/task.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

//read adc voltage
//blocking
double ads_read(curr_sens_adc_channel_t);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/
static const char* TAG = "current_sensor";

//motor pwm timer channel
static Adafruit_ADS1115 ads;

static double zeros[4] = {0.0};
static bool zeroed_on_startup[4] = {false};

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/


void current_sensor_init() {
  //note ads.begin() implicity uses i2c address 72U
  //                and default scl/sda i2c pins gpio 22 (scl) and gpio21 (sda)

  if (!ads.begin()) {
    if (Serial && Serial.availableForWrite()) {
      ESP_LOGW(TAG, "Current Sensor: Failed to init ADS115.\n");
    }
  }

  //set i2c clock to fast mode
  i2cSetClock(0, 400000U);
  uint32_t freq = 0;
  i2cGetClock(0, &freq);
  ESP_LOGI(TAG, "i2c frequency: %lu\n", freq);

  ads.setDataRate(RATE_ADS1115_860SPS);
  ads.setGain(adsGain_t::GAIN_TWOTHIRDS);
}

double current_sensor_get_volts(curr_sens_adc_channel_t chan) {
  return (ads_read(chan) + zeros[chan]);
}

double current_sensor_get_current(curr_sens_adc_channel_t chan) {
  return (ads_read(chan) + zeros[chan])/CURRENT_SENS_VOLTS_PER_AMP;
}

uint16_t current_sensor_sps() {
  return ads.getDataRate();
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

double ads_read(curr_sens_adc_channel_t chan){
  ESP_LOGV(TAG, "current sensor adc read channel %i", chan);
  //not sure how long this takes
  auto sens =  ads.computeVolts(ads.readADC_SingleEnded(MUX_BY_CHANNEL[chan]));
  if (!zeroed_on_startup[chan]) {
    zeros[chan] = sens;
    zeroed_on_startup[chan] = true;
  }
  return sens;
}