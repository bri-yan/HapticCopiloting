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

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

/**
 * @brief Motor States
 * 
 */
typedef enum {
    MOTOR_DRIVE_CCW, 
    MOTOR_DRIVE_CW,
    MOTOR_LOW,
    MOTOR_NOT_INITIALIZED = -1,
} motor_state_t;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Init motor on power and directions pins
 *
 */
void motor_init(const gpio_num_t motor_power_pin, const gpio_num_t motor_dir0_pin, const gpio_num_t motor_dir1_pin);

/**
 * @brief Sets motor state to low
 *
 */
void motor_stop();

motor_state_t motor_set_state(motor_state_t);

motor_state_t motor_get_state();

/**
 * @brief PWM frequency
 *
 */
uint32_t motor_get_frequency();

/**
 * @brief PWM duty cycle
 *
 */
uint32_t motor_get_duty_cycle();

/**
 * @brief Sets motor pwm duty cycle
 *  Drive direction is also set based on sign of duty cycle (drive left for dc < 0)
 */
int32_t motor_set_pwm(int32_t dc);

/**
 * @brief Stops motor and reset esp32 if speed exceeds MOTOR_MAX_SPEED_DEGS
 */
void motor_safety_check(double speed);


#endif // MOTOR_H_