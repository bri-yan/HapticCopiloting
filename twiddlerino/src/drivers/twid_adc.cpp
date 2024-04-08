/**
 * @file twid_adc.cpp
 * @brief Continous adc using dma
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_continuous.html
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "drivers/twid_adc.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "esp_timer.h"
/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define ADC_DEFAULT_VREF    1100

#define READ_LEN                    16

#define ADC_UNIT                    ADC_UNIT_1
#define ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN                   ADC_ATTEN_DB_11
#define ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)

#define ADC_READ_TASK_CORE 1

#define TWID_NUM_ADC_CHANNELS 2

#define READ_TIMEOUT_US 500
/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

// static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
static void calibrate_adc();

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/
static const char* TAG = "twid_ADC";

static adc_channel_t channel[TWID_NUM_ADC_CHANNELS] = {ADC_CHANNEL_6, ADC_CHANNEL_7}; //GPIO 34 AND 35
static volatile double lastest_readings[TWID_NUM_ADC_CHANNELS] = {0.0};

// static TaskHandle_t s_task_handle;

static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle;//calibration

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void twid_adc_init() {
    calibrate_adc();

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &adc_handle);

    // adc_continuous_evt_cbs_t cbs = {
    //     .on_conv_done = s_conv_done_cb,
    // };
    // ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    ESP_LOGI(TAG, "called adc_continuous_start");
}

bool twid_adc_get_raw(adc_channel_t channel, int32_t* raw) {

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[READ_LEN] = {0};
    memset(result, 0xcc, READ_LEN);

    *raw = 0;
    uint32_t cnt = 0;

    int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() < start + READ_TIMEOUT_US) {
        ret = adc_continuous_read(adc_handle, result, READ_LEN, &ret_num, 0);
        if (ret == ESP_OK) {
            ESP_LOGV("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                uint32_t chan_num = ADC_GET_CHANNEL(p);
                uint32_t data = ADC_GET_DATA(p);

                if (chan_num == channel) {
                    ESP_LOGV(TAG, "Channel: %"PRIu32", Value: %"PRIx32, chan_num, data);
                    *raw+=data;
                    cnt++;
                }
            }

            if(cnt > 0) {
                *raw = (int32_t)(*raw / cnt);
                return true;
            }
        }
    }

    return false;
}

void twid_adc_raw_to_millivolts(int32_t raw, int32_t *volts) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, (int*)volts));
    ESP_LOGV(TAG, "ADC%d converted raw %li to  Cali Voltage: %li mV", ADC_UNIT_1 + 1, raw, *volts);
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    ESP_LOGI(TAG, "adc_continuous_new_handle called successfully");

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 100 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    *out_handle = handle;

    ESP_LOGI(TAG, "adc_continuous_config called successfully");
}

static void calibrate_adc() {
    ESP_LOGI(TAG, "calibration scheme version is Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = adc_bitwidth_t::ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle));
    ESP_LOGI(TAG, "ADC calibration scheme created successfully.");
}
