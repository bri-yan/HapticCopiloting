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

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

// Typedefs that are only used in this file, empty for now

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

//motor pwm timer channel
static const uint8_t motor_pwm_channel = 0;//harware timer channel for pwm signal

//motor control state variables
static motor_state_t motor_state = motor_state_t::MOTOR_NOT_INITIALIZED;

static gpio_num_t power_pin = GPIO_NUM_NC;
static gpio_num_t dir0_pin = GPIO_NUM_NC;;
static gpio_num_t dir1_pin = GPIO_NUM_NC;;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/


void motor_init(const gpio_num_t motor_power_pin, const gpio_num_t motor_dir0_pin, const gpio_num_t motor_dir1_pin) {
    power_pin = motor_power_pin;
    dir0_pin = motor_dir0_pin;
    dir1_pin = motor_dir1_pin;

    // MOTOR pins
    pinMode(dir0_pin, OUTPUT);
    pinMode(dir1_pin, OUTPUT);

    //setup motor PWM output on MotorPowerPin
    //this uses the esp32 idf ledc driver
    //the ledc driver is writting to power leds 
    // but can also gen pwm signals for other purposes
    ledcSetup(motor_pwm_channel, MOTOR_PWM_FREQ, MOTOR_DUTY_CYCLE_RES_BITS); //channel 0, 32khz, 8 bit (0-255) duty cycle resolution
    ledcAttachPin(power_pin, motor_pwm_channel);//attack motor power pin to channel 0 timer
    ledcWrite(motor_pwm_channel, 0);//start at duty cycle of 0 (analog voltage ~ 0)

    motor_set_state(motor_state_t::MOTOR_LOW);
}

//stop power to motor
void motor_stop() {
    ledcWrite(motor_pwm_channel, 0);
    ledcDetachPin(power_pin);
    power_pin = GPIO_NUM_NC;
    dir0_pin = GPIO_NUM_NC;
    dir1_pin = GPIO_NUM_NC;
    motor_state = MOTOR_NOT_INITIALIZED;
}

motor_state_t motor_get_state() {
    return motor_state;
}

uint32_t motor_get_frequency() {
    return ledcReadFreq(motor_pwm_channel);
}

uint32_t motor_get_duty_cycle() {
    return ledcRead(motor_pwm_channel);
}

int32_t motor_set_pwm(int32_t dc)
{
  int32_t pwm_duty_cycle = min(MOTOR_DUTY_CYCLE_RES, max(-MOTOR_DUTY_CYCLE_RES, dc));
  if (pwm_duty_cycle == 0)
  {
    motor_set_state(MOTOR_LOW);
  }
  else if (pwm_duty_cycle < 0)
  {
    motor_set_state(MOTOR_DRIVE_CCW);
    ledcWrite(motor_pwm_channel, abs(pwm_duty_cycle));
  } else
  {
    motor_set_state(MOTOR_DRIVE_CW);
    ledcWrite(motor_pwm_channel, abs(pwm_duty_cycle));
  }

  return pwm_duty_cycle;
}

motor_state_t motor_set_state(motor_state_t state)
{
   switch(state)
   {
     case motor_state_t::MOTOR_DRIVE_CCW:
       digitalWrite(dir0_pin, HIGH);
       digitalWrite(dir1_pin, LOW);
       break;
     case motor_state_t::MOTOR_DRIVE_CW:
        digitalWrite(dir0_pin, LOW);
        digitalWrite(dir1_pin, HIGH);
       break;
     case motor_state_t::MOTOR_LOW:
        digitalWrite(dir0_pin, LOW);
        digitalWrite(dir1_pin, LOW);
        break;
     default:
        if(Serial.availableForWrite()) {
            Serial.println("ERROR, incorrect motor direction!");
        }
   }

   return motor_state;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/