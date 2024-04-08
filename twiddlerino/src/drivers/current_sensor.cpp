/**
 * @file current_sensor.h
 * @brief Current sensing driver, using esp32 adc DMA
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_continuous.html
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "drivers/current_sensor.h"

#include "esp_log.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define CURRENT_SENS_VOLTS_PER_AMP 0.136 //mV/Amp sensitivity on the hall effect sensor

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

current_sens_contex_t current_sens_1_handle {
    .channel = ADC_CHANNEL_6,
    .last_raw = 0,
    .is_zeroed = false,
    .zero_volts = 0.0,
    .last_volts = 0.0,
    .last_amps = 0.0,
};

current_sens_contex_t current_sens_2_handle {
    .channel = ADC_CHANNEL_7,
    .last_raw = 0,
    .is_zeroed = false,
    .zero_volts = 0.0,
    .last_volts = 0.0,
    .last_amps = 0.0,
};

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

static void zero(current_sens_contex_t*);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/
static const char* TAG = "current_sensor";

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void current_sensor_init_all() {
    ESP_LOGI(TAG, "current sensor init");
    twid_adc_init();
    zero(&current_sens_1_handle);
    zero(&current_sens_2_handle);
}

bool current_sensor_get_volts(current_sens_contex_t* ctx, double* volts) {
    bool status = false;
    int32_t raw;
    status = twid_adc_get_raw(ctx->channel, &raw);
    if(!status) {
        ESP_LOGI(TAG, "failed get raw reading on adc cahnn %i", ctx->channel);
        return false;
    }

    int32_t mv;
    twid_adc_raw_to_millivolts(raw, &mv);
    *volts = 1.0e-3 * mv;

    ctx->last_volts = *volts;
    ctx->last_raw = raw;
    return true;
}

bool current_sensor_get_amps(current_sens_contex_t* ctx, double* amps) {
    double volts;
    if (!current_sensor_get_volts(ctx, &volts)){
        return false;
    }

    current_sensor_volts_to_amps(volts, ctx->zero_volts, amps);
    ctx->last_amps = *amps;
    return true;
}

void current_sensor_volts_to_amps(double volts, double zero, double* amps) {
    *amps = (volts - zero)/CURRENT_SENS_VOLTS_PER_AMP;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void zero(current_sens_contex_t* ctx) {
    bool status = false;
    status = twid_adc_get_raw(ctx->channel, &ctx->last_raw);
    if(!status) {
        ESP_LOGI(TAG, "failed get raw reading on adc cahnn %i", ctx->channel);
        return;
    }
    int32_t mv;
    twid_adc_raw_to_millivolts(ctx->last_raw, &mv);
    ctx->zero_volts = mv * 1.0e-3;
    ctx->is_zeroed = true;

    ctx->last_volts = ctx->zero_volts;
    current_sensor_volts_to_amps(ctx->last_volts, ctx->zero_volts, &ctx->last_amps);

    ESP_LOGI(TAG, "zeroed current sensor on adc chan %i", ctx->channel);
}