/**
 * @file current_sensor.h
 * @brief Current sensing driver, using adc
 * @author Gavin Pringle and Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "drivers/current_sensor.h"

//ads drivers
#include "Adafruit_ADS1X15.h"
#include "SPI.h"

//ring buffer for measurements
#include "freertos/ringbuf.h"

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

double ads_read();

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

//motor pwm timer channel
static Adafruit_ADS1115 ads;
static RingbufHandle_t cur_sens_buffer;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/


void current_sensor_init() {
  //note ads.begin() implicity uses i2c address 72U
  //                and default scl/sda i2c pins gpio 22 (scl) and gpio21 (sda)
  if (!ads.begin()) {
    if (Serial && Serial.availableForWrite()) {
      Serial.printf("Failed to init ADS.");
    }
  }

  ads.setDataRate(RATE_ADS1115_860SPS);
  ads.startADCReading(MUX_BY_CHANNEL[0], true);

  //init the measurement buffer
  cur_sens_buffer = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
}

double current_sensor_read() {
  return ads_read() / SHUNT_RESISTOR_OHM;
}


double current_sensor_read_voltage() {
  return ads_read();
}

uint16_t current_sensor_sps() {
  return ads.getDataRate();
}


/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

double ads_read(){
  return ads.computeVolts(ads.getLastConversionResults());
}