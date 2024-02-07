/**
 * @file motor.h
 * @brief Motor driver
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * Based on code from Oliver Schneider (oschneid@cs.ubc.ca) and Bereket Guta (bguta@cs.ubc.ca)
 * 
 */

#ifndef MOTOR_H_
#define MOTOR_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//for int types
#include <stdint.h>

#include "Arduino.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define MOTOR_PWM_FREQ 32000U
#define MOTOR_DUTY_CYCLE_RES 8

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/**
 * @brief Motor States
 * 
 */
typedef enum {
    MOTOR_LEFT, 
    MOTOR_RIGHT,
    MOTOR_LOW,
    MOTOR_NOT_INITIALIZED = -1,
} motor_state_t;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//initializes motor
void motor_init(const gpio_num_t motor_power_pin, const gpio_num_t motor_dir0_pin, const gpio_num_t motor_dir1_pin);

//stop power to motor
void motor_stop();

motor_state_t motor_get_state();

uint32_t motor_get_frequency();

uint32_t motor_get_duty_cycle();

//sets the duty cycle to the motor
void motor_set_pwm(int32_t dc);


#endif // MOTOR_H_