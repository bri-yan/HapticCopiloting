/**
 * @file Encoder.cpp
 * @brief Encoder driver for esp32 using pcnt peripherial to decode quaderature signals
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/pcnt.html
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//encoder lib header
#include "drivers/encoder.h"


#include "esp_log.h"
#include "driver/pcnt.h"
#include "sys/lock.h"
#include "hal/pcnt_hal.h"
#include "freertos/portmacro.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

//for dealing with accessing shared variables between an isr and get_count function
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

//to ensure we can attach interrupt to only 1 thread
#define LOCK_ACQUIRE() _lock_acquire(&isr_service_install_lock)
#define LOCK_RELEASE() _lock_release(&isr_service_install_lock)

//16 bit counter limit
#define PCNT_H_LIM 32766
#define PCNT_L_LIM -32766

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

encoder_context_t encoder1_handle;
encoder_context_t encoder2_handle;

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

//this interrupt is called on configured count events
//in our case, every time count increments
static void encoder_pcnt_overflow_interrupt_handler(void *arg);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/
static const char* TAG = "encoder";

// A flag to identify if pcnt isr service has been installed.
static bool is_pcnt_isr_service_installed = false;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static _lock_t isr_service_install_lock;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void encoder_init(encoder_context_t* encoder_ctx) {
    ESP_LOGI(TAG, "Encoder innit for unit %i",encoder_ctx->pcnt_unit);
    //init value
    encoder_ctx->encoder_accu_cnt = 0;

    //configure quaderature pins as input
    gpio_config_t io_conf  = {
      .pin_bit_mask = BIT64(encoder_ctx->quad_pin_a)|BIT64(encoder_ctx->quad_pin_a),
      .mode = gpio_mode_t::GPIO_MODE_INPUT,
      .pull_up_en = (gpio_pullup_t) 0,
      .pull_down_en = (gpio_pulldown_t)0,
      .intr_type = gpio_int_type_t::GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "Quaderature GPIO pins initialized (QuadA: gpio %i, QuadB: gpio %i)",encoder_ctx->quad_pin_a,encoder_ctx->quad_pin_b);

    //config channel 0 counter
    pcnt_config_t dev_config = {
        .pulse_gpio_num = encoder_ctx->quad_pin_a,
        .ctrl_gpio_num = encoder_ctx->quad_pin_b,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = PCNT_H_LIM,
        .counter_l_lim = PCNT_L_LIM,
        .unit = encoder_ctx->pcnt_unit,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&dev_config);

    //config channel 1 counter
    dev_config.pulse_gpio_num = encoder_ctx->quad_pin_b;
    dev_config.ctrl_gpio_num = encoder_ctx->quad_pin_a;
    dev_config.channel = PCNT_CHANNEL_1;

    //disable because we are counting 2 (half) quaderature
    dev_config.pos_mode = PCNT_COUNT_DIS;
    dev_config.neg_mode = PCNT_COUNT_DIS;
    dev_config.lctrl_mode = PCNT_MODE_DISABLE;
    dev_config.lctrl_mode = PCNT_MODE_DISABLE;
    ESP_ERROR_CHECK(pcnt_unit_config(&dev_config));

    //pause and reset count value
    ESP_ERROR_CHECK(pcnt_counter_pause(encoder_ctx->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_counter_clear(encoder_ctx->pcnt_unit));

    // register interrupt handler and update flag
    // lock for thread safety
    LOCK_ACQUIRE();
    if (!is_pcnt_isr_service_installed) {
        ESP_ERROR_CHECK(pcnt_isr_service_install(0));
        is_pcnt_isr_service_installed = true;
    }
    LOCK_RELEASE();

    //attach interrupt handler
    ESP_ERROR_CHECK(pcnt_isr_handler_add(encoder_ctx->pcnt_unit, encoder_pcnt_overflow_interrupt_handler, encoder_ctx));

    //configure events for interrupt
    //trigger on limit overflow
    ESP_ERROR_CHECK(pcnt_event_enable(encoder_ctx->pcnt_unit, PCNT_EVT_H_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(encoder_ctx->pcnt_unit, PCNT_EVT_L_LIM));
    //trigger on decrement / increment
    ESP_ERROR_CHECK(pcnt_set_event_value(encoder_ctx->pcnt_unit, PCNT_EVT_THRES_0, -1));
    ESP_ERROR_CHECK(pcnt_set_event_value(encoder_ctx->pcnt_unit, PCNT_EVT_THRES_1, 1));
    ESP_ERROR_CHECK(pcnt_event_enable(encoder_ctx->pcnt_unit, PCNT_EVT_THRES_0));
    ESP_ERROR_CHECK(pcnt_event_enable(encoder_ctx->pcnt_unit, PCNT_EVT_THRES_1));

    //set filter value
    ESP_ERROR_CHECK(pcnt_set_filter_value(encoder_ctx->pcnt_unit, encoder_ctx->filter));
    if(encoder_ctx->filter) {
        ESP_ERROR_CHECK(pcnt_filter_enable(encoder_ctx->pcnt_unit));
    } else {
        ESP_ERROR_CHECK(pcnt_filter_disable(encoder_ctx->pcnt_unit));
    }
    ESP_LOGI(TAG, "Pulse counter peripheral unit %i configured",encoder_ctx->pcnt_unit);

    //enable interrupt and resume count
    ESP_ERROR_CHECK(pcnt_intr_enable(encoder_ctx->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_counter_resume(encoder_ctx->pcnt_unit));
    ESP_LOGI(TAG, "Pulse counter peripheral unit %i interrupt enabled and resumed", encoder_ctx->pcnt_unit);
}

void encoder_pause(encoder_context_t* encoder_ctx)
{
    pcnt_counter_pause(encoder_ctx->pcnt_unit);
}

void encoder_terminate(encoder_context_t* encoder_ctx)
{
    ESP_LOGD(TAG, "encoder_terminate called");
    //disable all things enabled in setup
    pcnt_set_filter_value(encoder_ctx->pcnt_unit, 0);
    pcnt_filter_disable(encoder_ctx->pcnt_unit);
    pcnt_counter_pause(encoder_ctx->pcnt_unit);
    pcnt_counter_clear(encoder_ctx->pcnt_unit);
    pcnt_event_disable(encoder_ctx->pcnt_unit, PCNT_EVT_H_LIM);
    pcnt_event_disable(encoder_ctx->pcnt_unit, PCNT_EVT_L_LIM);
    pcnt_isr_handler_remove(encoder_ctx->pcnt_unit);
    pcnt_isr_service_uninstall();
    pcnt_intr_disable(encoder_ctx->pcnt_unit);
    encoder_ctx->encoder_accu_cnt = 0;
}

void encoder_clear_count(encoder_context_t* encoder_ctx)
{
    pcnt_counter_pause(encoder_ctx->pcnt_unit);
    pcnt_counter_clear(encoder_ctx->pcnt_unit);
    _ENTER_CRITICAL();
    ESP_LOGD(TAG, "encoder clear count called (current count: %lli)", encoder_ctx->encoder_accu_cnt);
    encoder_ctx->encoder_accu_cnt = 0;
    _EXIT_CRITICAL();
    pcnt_counter_resume(encoder_ctx->pcnt_unit);
}

int64_t encoder_get_count(encoder_context_t* encoder_ctx)
{
    int16_t cval;
    int64_t rval;
    pcnt_get_counter_value(encoder_ctx->pcnt_unit, &cval);
    _ENTER_CRITICAL();
    rval = encoder_ctx->encoder_accu_cnt + cval;
    _EXIT_CRITICAL();

    return rval;
}

double encoder_get_angle(encoder_context_t* encoder_ctx)
{
    return ((encoder_get_count(encoder_ctx) / (ENCODER_CPR*2.0)) * 360);
}

double encoder_get_angle_rad(encoder_context_t* encoder_ctx)
{
    return encoder_get_angle(encoder_ctx) * DEGREES_TO_RADIANS;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void encoder_pcnt_overflow_interrupt_handler(void *arg)
{
    uint32_t status = 0;
    encoder_context_t *encoder_ctx = (encoder_context_t *)arg;

    _ENTER_CRITICAL();

    ESP_ERROR_CHECK(pcnt_get_event_status(encoder_ctx->pcnt_unit, &status));

    //check each event and increment count accordingly
    if (status & PCNT_EVT_H_LIM) {
        encoder_ctx->encoder_accu_cnt += PCNT_H_LIM;
        ESP_ERROR_CHECK(pcnt_counter_clear(encoder_ctx->pcnt_unit));
    } else if (status & PCNT_EVT_L_LIM) {
        encoder_ctx->encoder_accu_cnt += PCNT_L_LIM;
        ESP_ERROR_CHECK(pcnt_counter_clear(encoder_ctx->pcnt_unit));
    } else if (status & PCNT_EVT_THRES_0 || status & PCNT_EVT_THRES_1) {
        int16_t cval;
		ESP_ERROR_CHECK(pcnt_get_counter_value(encoder_ctx->pcnt_unit, &cval));
		encoder_ctx->encoder_accu_cnt += cval;
		ESP_ERROR_CHECK(pcnt_counter_clear(encoder_ctx->pcnt_unit));
    }

    _EXIT_CRITICAL();
}