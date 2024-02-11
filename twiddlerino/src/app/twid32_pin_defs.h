/**
 * @file twid32_pin_defs.h
 * @brief pin definitions for esp32 io
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

#ifndef TWID32_PIN_DEFS_H_
#define TWID32_PIN_DEFS_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include "hal/gpio_types.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define PIN_ENCODER_QUAD_A  gpio_num_t::GPIO_NUM_36  //Encoder quaderture signal A
#define PIN_ENCODER_QUAD_B  gpio_num_t::GPIO_NUM_39   //Encoder quaderture signal b

#define PIN_MOTOR_POWER     gpio_num_t::GPIO_NUM_13  //motor power (pwm) signal
#define PIN_MOTOR_DIR_0     gpio_num_t::GPIO_NUM_12  //motor bridge dir 0 pin
#define PIN_MOTOR_DIR_1     gpio_num_t::GPIO_NUM_14  //motor bridge dir 1 pin

#endif // TWID32_PIN_DEFS_H_