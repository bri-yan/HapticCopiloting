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

#include "SPI.h"
#include "esp32-hal-i2c.h"
#include "esp32-hal-i2c-slave.h"

//ads115 adc driver
#include "Adafruit_ADS1X15.h"

//esp adc driver
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "freertos/task.h"

#include "drivers/dma_read.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define ADC_DEFAULT_VREF    1100

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

//read adc voltage
//blocking
double ads_read(ads115_adc_channel_t);

//get latest voltage reading non-blocking
double get_latest(ads115_adc_channel_t);

//continiously updates latest readings
void TaskReadCurrentSensor(void *pvParameters);

void adc_calibrate(curr_sens_adc_context_t* handle);

static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/
static const char* TAG = "current_sensor";

//ads115 related variables
static Adafruit_ADS1115 ads;
static double zeros[4] = {0.0};
static bool zeroed_on_startup[4] = {false};
static volatile double latest_readings[4] = {0.0};
static TaskHandle_t xCurrentSensTask;
static SemaphoreHandle_t xSensMutex;

static curr_sens_adc_context_t curr_sens_1 = {
  .channel = CURRENT_SENS_1_ADC_CHAN,
  .last_reading_volts = 0,
  .zero = 0,
  .is_zeroed = false,
};

static curr_sens_adc_context_t curr_sens_2 = {
  .channel = CURRENT_SENS_2_ADC_CHAN,
  .last_reading_volts = 0,
  .zero = 0,
  .is_zeroed = false,
};

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

curr_sens_adc_context_t *curr_sens_1_handle = &curr_sens_1;
curr_sens_adc_context_t *curr_sens_2_handle = &curr_sens_2;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

//use internal adc instead of ads115
void curr_sens_adc_init() {
    esp_err_t error_code;

    dma_config();

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));

    auto cal_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, ADC_DEFAULT_VREF, &(curr_sens_1.adc_cal));
    curr_sens_2.adc_cal = curr_sens_1.adc_cal;
    if (cal_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value\n");
    } else if (cal_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG,"Characterized using eFuse Vref\n");
    } else {
        ESP_LOGI(TAG,"Characterized using Default Vref of %i mV\n", ADC_DEFAULT_VREF);
    }

    //start dma
    ESP_ERROR_CHECK(adc_digi_start());
    ESP_LOGI(TAG,"Stared the adc dma\n");

    curr_sens_adc_zero(&curr_sens_1);
    curr_sens_adc_zero(&curr_sens_2);
}

void curr_sens_adc_zero(curr_sens_adc_context_t* handle) {
  ESP_LOGI(TAG, "Zerod esp adc 1 on channel %i", handle->channel);
  handle->zero = curr_sens_adc_get_volts(handle);
  handle->is_zeroed = true;
}

double curr_sens_convert_current(curr_sens_adc_context_t* handle, double volts) {
  double current_amps = (((volts - handle->zero))/CURRENT_SENS_VOLTS_PER_AMP);
  return current_amps;
}

double curr_sens_adc_get_volts(curr_sens_adc_context_t* handle) {
  uint32_t start =  micros();
  ESP_LOGV(TAG, "adc read start time: %lu", start);

  uint32_t reading = 0;
  if (dma_read(&reading, handle->channel)) {
    uint32_t volts_mv = esp_adc_cal_raw_to_voltage(reading, &(handle->adc_cal));
    double volts = volts_mv*1e-3;
    ESP_LOGV(TAG, "adc read value: %lf, conversion time: %lu", volts, micros()-start);
    return volts;
  } else {
      ESP_LOGV(TAG, "adc read invalid, conversion time: %lu", micros()-start);
  }
  return 0;
}

void ads115_sens_init() {

  xSensMutex = xSemaphoreCreateMutex();

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

double ads115_get_volts(ads115_adc_channel_t chan) {
  return (get_latest(chan) - zeros[chan]);
}

double current_sensor_get_current(ads115_adc_channel_t chan) {
  return (get_latest(chan) - zeros[chan])/CURRENT_SENS_VOLTS_PER_AMP;
}

uint16_t ads115_sens_get_sps() {
  return ads.getDataRate();
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

double get_latest(ads115_adc_channel_t chan) {
  if (xPortInIsrContext() == pdTRUE) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(xSensMutex, &xHigherPriorityTaskWoken);
    auto sens = latest_readings[chan];
    xSemaphoreGiveFromISR(xSensMutex, &xHigherPriorityTaskWoken);
    return sens;
  } else {
    xSemaphoreTake(xSensMutex, portMAX_DELAY);
    auto sens = latest_readings[chan];
    xSemaphoreGive(xSensMutex);
    return sens;
  }
}

void TaskReadCurrentSensor(void *pvParameters) {
  ESP_LOGI(TAG, "ads115 current sensor read task started");
  for(;;) 
  {
    uint32_t start_time = micros();
    auto chan1 = ads_read(ads115_adc_channel_t::CURRENT_SENSOR_1);
    ESP_LOGD(TAG, "read channel 1 dt: %lu us",micros()-start_time);
    vTaskDelay( 0 );
    start_time = micros();
    auto chan2 = ads_read(ads115_adc_channel_t::CURRENT_SENSOR_2);
    ESP_LOGD(TAG, "read channel 1 dt: %lu us",micros()-start_time);

    xSemaphoreTake(xSensMutex, portMAX_DELAY);
    latest_readings[ads115_adc_channel_t::CURRENT_SENSOR_1] = chan1;
    latest_readings[ads115_adc_channel_t::CURRENT_SENSOR_2] = chan2;
    xSemaphoreGive(xSensMutex);
  
    vTaskDelay( 1 );
  }
}

double ads_read(ads115_adc_channel_t chan){
  ESP_LOGV(TAG, "ads155 current sensor adc read channel %i", chan);
  //not sure how long this takes
  auto sens = ads.computeVolts(ads.readADC_SingleEnded(MUX_BY_CHANNEL[chan]));
  if (!zeroed_on_startup[chan]) {
    ESP_LOGI(TAG, "zeroed current sens adc reading on channel %i", chan);
    zeros[chan] = sens;
    zeroed_on_startup[chan] = true;
  }
  return sens;
}