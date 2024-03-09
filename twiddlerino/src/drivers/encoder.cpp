/**
 * @file Encoder.cpp
 * @brief Encoder driver for esp32 using pcnt peripherial to decode quaderature signals
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//encoder lib header
#include "drivers/encoder.h"

#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_compiler.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include "sys/lock.h"
#include "hal/pcnt_hal.h"

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
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

//this interrupt is called on configured count events
//in our case, every time count increments
static void encoder_pcnt_overflow_interrupt_handler(void *arg);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

// A flag to identify if pcnt isr service has been installed.
static bool is_pcnt_isr_service_installed = false;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static _lock_t isr_service_install_lock;

static pcnt_unit_t encoder_pcnt_unit = PCNT_UNIT_0;

//encoder count
//This is updated by an interrupt and likely not atomic!
//use lock/mutex when reading/writing to this variable
static volatile int64_t encoder_accu_cnt = 0;
static volatile int64_t encoder_last_cnt = 0;
static volatile uint64_t encoder_last_time = 0;
static volatile double encoder_cnt_velocity = 0.0;
static volatile double encoder_cnt_last_velocity = 0.0;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void encoder_init(pcnt_unit_t unit, gpio_num_t quad_pin_a, gpio_num_t quad_pin_b, uint16_t filter) {

    //init value
    encoder_accu_cnt = 0;
    encoder_last_cnt = 0;
    encoder_last_time = 0;
    encoder_cnt_velocity = 0.0;
    encoder_cnt_last_velocity = 0.0;

    encoder_pcnt_unit = unit;

    //pins
    pinMode(quad_pin_a, INPUT_PULLUP);
    pinMode(quad_pin_b, INPUT_PULLUP);

    //config channel 0 counter
    pcnt_config_t dev_config = {
        .pulse_gpio_num = quad_pin_a,
        .ctrl_gpio_num = quad_pin_b,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = PCNT_H_LIM,
        .counter_l_lim = PCNT_L_LIM,
        .unit = encoder_pcnt_unit,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&dev_config);

    //config channel 1 counter
    dev_config.pulse_gpio_num = quad_pin_b;
    dev_config.ctrl_gpio_num = quad_pin_a;
    dev_config.channel = PCNT_CHANNEL_1;

    //disable because we are counting 2 (half) quaderature
    dev_config.pos_mode = PCNT_COUNT_DIS;
    dev_config.neg_mode = PCNT_COUNT_DIS;
    dev_config.lctrl_mode = PCNT_MODE_DISABLE;
    dev_config.lctrl_mode = PCNT_MODE_DISABLE;
    pcnt_unit_config(&dev_config);

    //pause and reset count value
    pcnt_counter_pause(encoder_pcnt_unit);
    pcnt_counter_clear(encoder_pcnt_unit);

    // register interrupt handler and update flag
    // lock for thread safety
    LOCK_ACQUIRE();
    if (!is_pcnt_isr_service_installed) {
        pcnt_isr_service_install(0);
        is_pcnt_isr_service_installed = true;
    }
    LOCK_RELEASE();

    //attach interrupt handler
    pcnt_isr_handler_add(encoder_pcnt_unit, encoder_pcnt_overflow_interrupt_handler, NULL);

    //configure events for interrupt
    //trigger on limit overflow
    pcnt_event_enable(encoder_pcnt_unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(encoder_pcnt_unit, PCNT_EVT_L_LIM);
    //trigger on decrement / increment
    pcnt_set_event_value(encoder_pcnt_unit, PCNT_EVT_THRES_0, -1);
    pcnt_set_event_value(encoder_pcnt_unit, PCNT_EVT_THRES_1, 1);
    pcnt_event_enable(encoder_pcnt_unit, PCNT_EVT_THRES_0);
    pcnt_event_enable(encoder_pcnt_unit, PCNT_EVT_THRES_1);

    //set filter value
    pcnt_set_filter_value(encoder_pcnt_unit, filter);
    if(filter) {
        pcnt_filter_enable(encoder_pcnt_unit);
    } else {
        pcnt_filter_disable(encoder_pcnt_unit);
    }

    //enable interrupt and resume count
    pcnt_intr_enable(encoder_pcnt_unit);
    pcnt_counter_resume(encoder_pcnt_unit);
}

void encoder_pause()
{
    pcnt_counter_pause(encoder_pcnt_unit);
}

void encoder_terminate()
{
    //disable all things enabled in setup
    pcnt_set_filter_value(encoder_pcnt_unit, 0);
    pcnt_filter_disable(encoder_pcnt_unit);
    pcnt_counter_pause(encoder_pcnt_unit);
    pcnt_counter_clear(encoder_pcnt_unit);
    pcnt_event_disable(encoder_pcnt_unit, PCNT_EVT_H_LIM);
    pcnt_event_disable(encoder_pcnt_unit, PCNT_EVT_L_LIM);
    pcnt_isr_handler_remove(encoder_pcnt_unit);
    pcnt_isr_service_uninstall();
    pcnt_intr_disable(encoder_pcnt_unit);
    encoder_cnt_velocity = 0.0;
    encoder_cnt_last_velocity = 0.0;
    encoder_accu_cnt = 0;
    encoder_last_cnt = 0;
    encoder_last_time = 0;
}

void encoder_clear_count()
{
    pcnt_counter_pause(encoder_pcnt_unit);
    pcnt_counter_clear(encoder_pcnt_unit);
    _ENTER_CRITICAL();
    encoder_cnt_velocity = 0.0;
    encoder_cnt_last_velocity = 0.0;
    encoder_accu_cnt = 0;
    encoder_last_cnt = 0;
    encoder_last_time = 0;
    _EXIT_CRITICAL();
    pcnt_counter_resume(encoder_pcnt_unit);
}

int64_t encoder_get_count()
{
    int16_t cval;
    int64_t rval;
    pcnt_get_counter_value(encoder_pcnt_unit, &cval);
    _ENTER_CRITICAL();
    rval = encoder_accu_cnt + cval;
    _EXIT_CRITICAL();

    return rval;
}

double encoder_get_angle()
{
    return ((encoder_get_count() / (ENCODER_CPR*2.0)) * 360);
}

double encoder_get_velocity()
{
    _ENTER_CRITICAL();
    if (micros() - encoder_last_time > ENCODER_VELOCITY_READ_TIMEOUT_US)  {
        //if no new encoder ticks after timeout, set velocity to 0
        encoder_cnt_velocity = 0.0;
    } else if (encoder_accu_cnt == encoder_last_cnt 
    && encoder_cnt_velocity == encoder_cnt_last_velocity) {
        encoder_cnt_last_velocity = encoder_cnt_velocity;
        encoder_cnt_velocity = 0.0;
    }
    _EXIT_CRITICAL();
    return (encoder_cnt_velocity / (ENCODER_CPR*2.0)) * 360;
}


double encoder_get_angle_rad()
{
    return encoder_get_angle() * DEGREES_TO_RADIANS;
}

double encoder_get_velocity_rad()
{
    return encoder_get_velocity() * DEGREES_TO_RADIANS;
}


/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void encoder_pcnt_overflow_interrupt_handler(void *arg)
{
    uint32_t status = 0;

    _ENTER_CRITICAL();

    pcnt_get_event_status(encoder_pcnt_unit, &status);

    //check each event and increment count accordingly
    if (status & PCNT_EVT_H_LIM) {
        encoder_accu_cnt += PCNT_H_LIM;
        pcnt_counter_clear(encoder_pcnt_unit);
    } else if (status & PCNT_EVT_L_LIM) {
        encoder_accu_cnt += PCNT_L_LIM;
        pcnt_counter_clear(encoder_pcnt_unit);
    } else if (status & PCNT_EVT_THRES_0 || status & PCNT_EVT_THRES_1) {
        int16_t cval;
		pcnt_get_counter_value(encoder_pcnt_unit, &cval);
		encoder_accu_cnt += cval;
		pcnt_counter_clear(encoder_pcnt_unit);
    }

    int64_t dcount = encoder_accu_cnt - encoder_last_cnt;
    encoder_last_cnt = encoder_accu_cnt;
    uint64_t dt = micros() - encoder_last_time;
    encoder_last_time = micros();
    encoder_cnt_last_velocity = encoder_cnt_velocity;
    encoder_cnt_velocity = ( ( dcount * 1.0e6 ) / dt );

    _EXIT_CRITICAL();
}