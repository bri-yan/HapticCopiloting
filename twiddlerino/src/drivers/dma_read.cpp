/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "drivers/dma_read.h"

#define TIMES              256
#define GET_UNIT(x)        ((x>>3) & 0x1)
#define ADC_RESULT_BYTE                 2
#define ADC_CONV_LIMIT_EN               1                       //For ESP32, this should always be set to 1
#define ADC_OUTPUT_TYPE                 ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_USE_OUTPUT_TYPE1    1
#define ADC_CONV_MODE                   ADC_CONV_SINGLE_UNIT_1
static uint16_t adc1_chan_mask = BIT(6)|BIT(7);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
static const char *TAG = "ADC DMA";

static bool check_valid_data(const adc_digi_output_data_t *data);
static void continuous_adc_config(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num);

void dma_config() {
    esp_err_t error_code;

    continuous_adc_config(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
    
    ESP_ERROR_CHECK(error_code);
}

static void continuous_adc_config(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = ADC_CONV_LIMIT_EN,
        .conv_limit_num = 250,
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}

static bool check_valid_data(const adc_digi_output_data_t *data)
{
    if (data->type1.channel >= SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
        return false;
    }

    return true;
}

bool dma_read(uint32_t* reading, adc_channel_t chann)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

    ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        if (ret == ESP_ERR_INVALID_STATE) {
            /**
             * @note 1
             * Issue:
             * As an example, we simply print the result out, which is super slow. Therefore the conversion is too
             * fast for the task to handle. In this condition, some conversion results lost.
             *
             * Reason:
             * When this error occurs, you will usually see the task watchdog timeout issue also.
             * Because the conversion is too fast, whereas the task calling `adc_digi_read_bytes` is slow.
             * So `adc_digi_read_bytes` will hardly block. Therefore Idle Task hardly has chance to run. In this
             * example, we add a `vTaskDelay(1)` below, to prevent the task watchdog timeout.
             *
             * Solution:
             * Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes`
             */
        }

        ESP_LOGV("TASK:", "ret is %x, ret_num is %d", ret, ret_num);
        uint32_t cnt = 0;
        for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
            if (p->type1.channel = chann) {
                ESP_LOGV(TAG, "Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel, p->type1.data);
                *reading += p->type1.data;
                cnt++;
            } else {
                ESP_LOGV(TAG, "Invalid data");
            }
        }

        if(cnt > 0) {
            *reading /= cnt;
            return true;
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
        /**
         * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
         * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
         */
        ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
    }

    return false;
}