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

//read adc voltage and convert to current in amps
//applies scaling, and offset
double ads_read();

double get_latest();

void TaskReadCurrentSensor(void *pvParameters);

void IRAM_ATTR adc_read_isr();

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

//motor pwm timer channel
static Adafruit_ADS1115 ads;

//freertos current sensor read task
static TaskHandle_t xCurrentSensTask;

static volatile double latest_reading = 0.0; // in amps

static bool zeroed_on_startup = false;
static double zero = 0.0;

static SemaphoreHandle_t xSensMutex;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/


void current_sensor_init() {
  xSensMutex = xSemaphoreCreateMutex();
  //note ads.begin() implicity uses i2c address 72U
  //                and default scl/sda i2c pins gpio 22 (scl) and gpio21 (sda)

  //setup alert pin
  pinMode(PIN_CURRENT_SENS_ADC_ALERT, INPUT);
  //read adc on every altert falling edge
  //https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html?highlight=pullup#attachinterrupt
  //https://www.ti.com/lit/ds/symlink/ads1115.pdf?ts=1711845972450&ref_url=https%253A%252F%252Fwww.google.com%252F
  attachInterrupt(PIN_CURRENT_SENS_ADC_ALERT, adc_read_isr, FALLING);

  xSemaphoreTake(xSensMutex, portMAX_DELAY);
  if (!ads.begin()) {
    if (Serial && Serial.availableForWrite()) {
      Serial.printf("Current Sensor: Failed to init ADS115.\n");
    }
  }

  //set i2c clock to fast mode
  i2cSetClock(0, 400000U);
  uint32_t freq = 0;
  i2cGetClock(0, &freq);
  Serial.printf("[current_sensor_init] i2c frequency: %lu\n", freq);

  ads.setDataRate(RATE_ADS1115_860SPS);
  ads.startADCReading(MUX_BY_CHANNEL[0], true);
  zeroed_on_startup = false;
  latest_reading = 0.0;
  xSemaphoreGive(xSensMutex);

  // xTaskCreatePinnedToCore(
  //   TaskReadCurrentSensor
  //   ,  "Current_Sensor_Read_Task"
  //   ,  8192
  //   ,  NULL
  //   ,  TASK_PRIORITY_SENSOR
  //   ,  &xCurrentSensTask
  //   ,  CORE_SENSOR_TASK
  // );
}

double current_sensor_get_volts() {
  return (get_latest() - zero);
}

double current_sensor_get_current() {
  return (get_latest() - zero)/CURRENT_SENS_VOLTS_PER_AMP;
}

uint16_t current_sensor_sps() {
  return ads.getDataRate();
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

double get_latest() {
  if (xPortInIsrContext() == pdTRUE) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(xSensMutex, &xHigherPriorityTaskWoken);
    auto sens = latest_reading;
    xSemaphoreGiveFromISR(xSensMutex, &xHigherPriorityTaskWoken);
    return sens;
  } else {
    xSemaphoreTake(xSensMutex, portMAX_DELAY);
    auto sens = latest_reading;
    xSemaphoreGive(xSensMutex);
    return sens;
  }
}

double ads_read(){
  xSemaphoreTake(xSensMutex, portMAX_DELAY);
  auto sens = ads.computeVolts(ads.getLastConversionResults());
  xSemaphoreGive(xSensMutex);
  return sens;
}

void IRAM_ATTR adc_read_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreTakeFromISR(xSensMutex, &xHigherPriorityTaskWoken);
  latest_reading = ads.computeVolts(ads.getLastConversionResults());
  if(!zeroed_on_startup) {
    zero = latest_reading;
    zeroed_on_startup = true;
  }
  xSemaphoreGiveFromISR(xSensMutex, &xHigherPriorityTaskWoken);
}

void TaskReadCurrentSensor(void *pvParameters) {
  for(;;) 
  {
    latest_reading = ads_read();
    vTaskDelay( 0 );
  }
}