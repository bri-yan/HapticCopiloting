/**
 * @file motor.cpp
 * @brief Motor driver
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * Based on code from Oliver Schneider (oschneid@cs.ubc.ca) and Bereket Guta (bguta@cs.ubc.ca)
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "drivers/motor.h"

//config
#include "app/twid32_config.h"

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

    // MOTOR pins
    pinMode(mctx->dir_pin, OUTPUT);
    ESP_LOGI(TAG, "Configured motor direction pin as output (gpio pin %i)",mctx->dir_pin);

    //setup motor PWM output on MotorPowerPin
    //this uses the esp32 idf ledc driver
    //the ledc driver is writting to power leds 
    // but can also gen pwm signals for other purposes
    ledcSetup(mctx->pwm_channel, MOTOR_PWM_FREQ, MOTOR_DUTY_CYCLE_RES_BITS); //channel 0, 32khz, 8 bit (0-255) duty cycle resolution
    ledcAttachPin(mctx->power_pin, mctx->pwm_channel);//attack motor power pin to channel 0 timer
    ledcWrite(mctx->pwm_channel, 0);//start at duty cycle of 0 (analog voltage ~ 0)
    ESP_LOGI(TAG, "Configured motor power pin (gpio pin %i) pwm signal on pwm channel %i", mctx->power_pin, mctx->pwm_channel);
    ESP_LOGI(TAG, "Configured pwm channel %i to %lu Hz and duty cycle resolution of %i bits", mctx->pwm_channel, MOTOR_PWM_FREQ, MOTOR_DUTY_CYCLE_RES_BITS);

    motor_set_state(mctx ,motor_state_t::MOTOR_LOW);
}

//stop power to motor
void motor_stop(motor_driver_context_t* mctx) {
    ESP_LOGD(TAG,"motor_stop called");
    ledcWrite(mctx->pwm_channel, 0);
    ledcDetachPin(mctx->power_pin);
    mctx->state = MOTOR_NOT_INITIALIZED;
}

motor_state_t motor_get_state(motor_driver_context_t* mctx) {
    return mctx->state;
}

uint32_t motor_get_frequency(motor_driver_context_t* mctx) {
    return ledcReadFreq(mctx->pwm_channel);
}

uint32_t motor_get_duty_cycle(motor_driver_context_t* mctx) {
    return ledcRead(mctx->pwm_channel);
}

int32_t motor_set_pwm(motor_driver_context_t* mctx, int32_t dc)
{
  ESP_LOGV(TAG,"%s set pwm to %i called", mctx->handle_name, dc);
  int32_t pwm_duty_cycle = min(MOTOR_DUTY_CYCLE_RES, max(-MOTOR_DUTY_CYCLE_RES, dc));
  if (pwm_duty_cycle == 0)
  {
    motor_set_state(mctx, MOTOR_LOW);
  }
  else if (pwm_duty_cycle < 0)
  {
    motor_set_state(mctx, MOTOR_DRIVE_CCW);
    ledcWrite(mctx->pwm_channel, abs(pwm_duty_cycle));
  } else
  {
    motor_set_state(mctx, MOTOR_DRIVE_CW);
    ledcWrite(mctx->pwm_channel, abs(pwm_duty_cycle));
  }

  return pwm_duty_cycle;
}

motor_state_t motor_set_state(motor_driver_context_t* mctx, motor_state_t state)
{
   ESP_LOGV(TAG,"%s set_state to %i called", mctx->handle_name, state);
   switch(state)
   {
     case motor_state_t::MOTOR_DRIVE_CCW:
       digitalWrite(mctx->dir_pin, HIGH);
       break;
     case motor_state_t::MOTOR_DRIVE_CW:
        digitalWrite(mctx->dir_pin, LOW);
       break;
     case motor_state_t::MOTOR_LOW:
        digitalWrite(mctx->dir_pin, LOW);
        ledcWrite(mctx->pwm_channel, 0);
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