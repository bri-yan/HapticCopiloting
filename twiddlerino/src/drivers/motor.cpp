/**
 * @file motor.cpp
 * @brief Motor driver
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html#
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "drivers/motor.h"

#include "esp_log.h"

#include "esp_system.h"

#include "driver/gpio.h"

#include "driver/ledc.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define LEDC_TIMER                  LEDC_TIMER_0
#define LEDC_MODE                   LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_FREQUENCY         (20000)

#define MOTOR_MAX_SPEED_RPM 100000

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

motor_driver_context_t motor1_handle;
motor_driver_context_t motor2_handle;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/
static const char* TAG = "motor";

void motor_init(motor_driver_context_t* mctx) {
    ESP_LOGI(TAG, "Configured motor for handle name %s",mctx->handle_name);
    //check that motor context looks fine
    if(mctx  == NULL || mctx->dir_pin == gpio_num_t::GPIO_NUM_NC 
    || mctx->power_pin == gpio_num_t::GPIO_NUM_NC 
    || mctx->pwm_channel > 15 || mctx->state != motor_state_t::MOTOR_NOT_INITIALIZED) {
      esp_system_abort("[motor_init] invalid motor context.");
    }

    // MOTOR direction pin configure
    gpio_config_t io_conf  = {
      .pin_bit_mask = BIT64(mctx->dir_pin),
      .mode = gpio_mode_t::GPIO_MODE_INPUT,
      .pull_up_en = (gpio_pullup_t) 0,
      .pull_down_en = (gpio_pulldown_t)0,
      .intr_type = gpio_int_type_t::GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "Configured motor direction pin as output (gpio pin %i)",mctx->dir_pin);

    //pwm timer config using ledc driver
    ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .duty_resolution = MOTOR_DUTY_CYCLE_RES_BITS,
      .timer_num = LEDC_TIMER,
      .freq_hz = MOTOR_PWM_FREQUENCY,
      .clk_cfg = ledc_clk_cfg_t::LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = mctx->power_pin,
        .speed_mode     = LEDC_MODE,
        .channel        = mctx->pwm_channel,
        .intr_type      = ledc_intr_type_t::LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "Configured motor power pin (gpio pin %i) pwm signal on pwm channel %i", mctx->power_pin, mctx->pwm_channel);
    ESP_LOGI(TAG, "Configured pwm channel %i to %i Hz and duty cycle resolution of %i bits", mctx->pwm_channel, MOTOR_PWM_FREQUENCY, MOTOR_DUTY_CYCLE_RES_BITS);

    ledc_set_duty(LEDC_MODE, mctx->pwm_channel, 0);
    ledc_update_duty(LEDC_MODE, mctx->pwm_channel);

    motor_set_state(mctx ,motor_state_t::MOTOR_LOW);
}

//stop power to motor
void motor_stop(motor_driver_context_t* mctx) {
    ESP_LOGD(TAG,"motor_stop called");
    ledc_set_duty(LEDC_MODE, mctx->pwm_channel, 0);
    ledc_update_duty(LEDC_MODE, mctx->pwm_channel);
    mctx->state = MOTOR_NOT_INITIALIZED;
}

motor_state_t motor_get_state(motor_driver_context_t* mctx) {
    return mctx->state;
}

uint32_t motor_get_frequency(motor_driver_context_t* mctx) {
    return ledc_get_freq(LEDC_MODE, LEDC_TIMER);
}

uint32_t motor_get_duty_cycle(motor_driver_context_t* mctx) {
    return ledc_get_duty(LEDC_MODE, mctx->pwm_channel);
}

int32_t motor_set_pwm(motor_driver_context_t* mctx, int32_t dc)
{
  ESP_LOGV(TAG,"%s set pwm to %li called", mctx->handle_name, dc);
  int32_t pwm_duty_cycle = dc;

  if(pwm_duty_cycle > MOTOR_DUTY_CYCLE_RES) {
    pwm_duty_cycle = MOTOR_DUTY_CYCLE_RES;
  } else if(pwm_duty_cycle < -MOTOR_DUTY_CYCLE_RES) {
    pwm_duty_cycle = -MOTOR_DUTY_CYCLE_RES;
  }

  if (pwm_duty_cycle == 0)
  {
    motor_set_state(mctx, MOTOR_LOW);
  }
  else if (pwm_duty_cycle < 0)
  {
    motor_set_state(mctx, MOTOR_DRIVE_CCW);
    ledc_set_duty(LEDC_MODE, mctx->pwm_channel, abs(pwm_duty_cycle));
    ledc_update_duty(LEDC_MODE, mctx->pwm_channel);
  } else
  {
    motor_set_state(mctx, MOTOR_DRIVE_CW);
    ledc_set_duty(LEDC_MODE, mctx->pwm_channel, abs(pwm_duty_cycle));
    ledc_update_duty(LEDC_MODE, mctx->pwm_channel);;
  }

  return pwm_duty_cycle;
}

motor_state_t motor_set_state(motor_driver_context_t* mctx, motor_state_t state)
{
   ESP_LOGV(TAG,"%s set_state to %i called", mctx->handle_name, state);
   switch(state)
   {
     case motor_state_t::MOTOR_DRIVE_CCW:
       gpio_set_level(mctx->dir_pin, 1);
       break;
     case motor_state_t::MOTOR_DRIVE_CW:
       gpio_set_level(mctx->dir_pin, 0);
       break;
     case motor_state_t::MOTOR_LOW:
        gpio_set_level(mctx->dir_pin, 0);
        ledc_set_duty(LEDC_MODE, mctx->pwm_channel, 0);
        ledc_update_duty(LEDC_MODE, mctx->pwm_channel);
        break;
     default:
      ESP_LOGE(TAG,"Incorrect motor state passed to motor_set_state (%i)", state);
      break;
   }

   return mctx->state;
}

void motor_safety_check(motor_driver_context_t* mctx, double speed) {
  if(speed > MOTOR_MAX_SPEED_RPM || speed < -MOTOR_MAX_SPEED_RPM) {
      motor_set_pwm(mctx, 0);
      motor_set_state(mctx, motor_state_t::MOTOR_LOW);
      ESP_LOGW(TAG, "Motor speed is too fast! %lf deg/s", speed);
      esp_system_abort("Aborting! Motor speed is too fast!");
  }
}

void motor_fast_stop(motor_driver_context_t* mctx) {
  motor_set_pwm(mctx, 0);
  motor_set_state(mctx, motor_state_t::MOTOR_LOW);
}