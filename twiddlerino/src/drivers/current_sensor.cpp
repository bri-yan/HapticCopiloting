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

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define ADC_DEFAULT_VREF    1100

#define TIMES              256
#define GET_UNIT(x)        ((x>>3) & 0x1)
#define ADC_RESULT_BYTE                 2
#define ADC_CONV_LIMIT_EN               1                       //For ESP32, this should always be set to 1
#define ADC_OUTPUT_TYPE                 ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_USE_OUTPUT_TYPE1    1
#define ADC_CONV_MODE                   ADC_CONV_SINGLE_UNIT_1
#define ADC_DEFAULT_ATTENUATION ADC_ATTEN_DB_11
#define ADC_DEFAULT_MULTISAMPLE_NUM 1U

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

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

static curr_sens_adc_context_t curr_sens_1 =   {
    .input_pin = PIN_CURRENT_SENS_1_ANALOG,
    .channel = CURRENT_SENS_1_ADC_CHAN,
    .width = adc_bits_width_t::ADC_WIDTH_BIT_12,
    .atten = ADC_DEFAULT_ATTENUATION,
    .adc_unit = ADC_UNIT_1,
    .zero = 0,
    .is_zeroed = false,
    .multisample_num = ADC_DEFAULT_MULTISAMPLE_NUM,
};

static curr_sens_adc_context_t curr_sens_2 =   {
      .input_pin = PIN_CURRENT_SENS_2_ANALOG,
      .channel = CURRENT_SENS_2_ADC_CHAN,
      .width = adc_bits_width_t::ADC_WIDTH_BIT_12,
      .atten = ADC_DEFAULT_ATTENUATION,
      .adc_unit = ADC_UNIT_1,
      .zero = 0,
      .is_zeroed = false,
      .multisample_num = ADC_DEFAULT_MULTISAMPLE_NUM,
};
//dma configs
static uint16_t adc1_chan_mask = BIT(curr_sens_1.channel);
static uint16_t adc2_chan_mask = 0;

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
  //calibrate each adc
  adc_calibrate(curr_sens_1_handle);
  adc_calibrate(curr_sens_2_handle);

  //init dma
  adc_digi_init_config_t adc_dma_config = {
      .max_store_buf_size = 1024,
      .conv_num_each_intr = TIMES,
      .adc1_chan_mask = adc1_chan_mask,
      .adc2_chan_mask = adc2_chan_mask,
  };
  ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

  adc_digi_pattern_config_t adc_digi_patterns[2];
  adc_digi_configuration_t adc_digi_config = {
      .conv_limit_en = true,
      .conv_limit_num = 1,
      .pattern_num = 2,
      .adc_pattern = adc_digi_patterns,
      .sample_freq_hz= 10000, //hz
      .conv_mode = adc_digi_convert_mode_t::ADC_CONV_SINGLE_UNIT_1,
      .format = adc_digi_output_format_t::ADC_DIGI_OUTPUT_FORMAT_TYPE1
  };

  curr_sens_adc_context_t *adc_ctxs[] = {&curr_sens_1, &curr_sens_2};
  //fill adc_digi_pattern_config_t for each defined adc
  for(int i = 0; i < 2; i++) {
    adc_digi_patterns[i] = {
      .atten = ADC_ATTEN_11db,
      .channel = (uint8_t)adc_ctxs[i]->channel,
      .unit = ADC_UNIT_1,
      .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
      };
  }

  ESP_ERROR_CHECK(adc_digi_controller_configure(&adc_digi_config));

  adc_digi_start();
}

void curr_sens_adc_zero(curr_sens_adc_context_t* handle) {
  ESP_LOGI(TAG, "Zerod esp adc %i on channel %i", handle->adc_unit, handle->channel);
  handle->zero = curr_sens_adc_get_volts(handle);
  handle->is_zeroed = true;
}

double curr_sens_convert_current(curr_sens_adc_context_t* handle, double volts) {
  double current_amps = (((volts - handle->zero))/CURRENT_SENS_VOLTS_PER_AMP);
  return current_amps;
}

double curr_sens_adc_get_volts(curr_sens_adc_context_t* handle) {
  ESP_LOGV(TAG, "adc read start time: %lu", micros());
  uint32_t ret_num = 0;
  uint8_t result[TIMES] = {0};

  uint32_t reading = 0;
  uint16_t cnt = 0;

  auto ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
  if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
    for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
        if (p->type1.channel == handle->channel) {
          reading+=p->type1.data;
          cnt++;
          ESP_LOGV(TAG, "dma: raw reading for channel %i : %i", p->type1.channel, p->type1.data);
        }
    }
  }

  reading/=cnt;
  uint32_t volts_mv = esp_adc_cal_raw_to_voltage(reading, &(handle->adc_cal));
  double volts = volts_mv*1e-3;

  ESP_LOGV(TAG, "adc read value: %lf, conversion time: %lu", volts, micros());
  return volts;
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

void adc_calibrate(curr_sens_adc_context_t* handle) {
    ESP_LOGI(TAG, "calibrating adc %i with attentuation type %i and bit width type %i",handle->adc_unit, handle->atten, SOC_ADC_DIGI_MAX_BITWIDTH);
  
    if (handle->adc_unit == ADC_UNIT_1) {
        adc1_config_width(handle->width);
        adc1_config_channel_atten((adc1_channel_t)handle->channel, handle->atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)handle->channel, handle->atten);
    }

    auto cal_type = esp_adc_cal_characterize(handle->adc_unit, handle->atten, handle->width, ADC_DEFAULT_VREF, &(handle->adc_cal));
    if (cal_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value\n");
    } else if (cal_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG,"Characterized using eFuse Vref\n");
    } else {
        ESP_LOGI(TAG,"Characterized using Default Vref of %i mV\n", ADC_DEFAULT_VREF);
    }
}

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