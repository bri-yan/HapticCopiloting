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

//encoder
#define PIN_ENCODER_QUAD_A  gpio_num_t::GPIO_NUM_25  //Encoder quaderture signal A
#define PIN_ENCODER_QUAD_B  gpio_num_t::GPIO_NUM_26   //Encoder quaderture signal b

//current sensor (communicates through i2c interface)
#define PIN_CURRENT_SENS_ADC_SCL  gpio_num_t::GPIO_NUM_22
#define PIN_CURRENT_SENS_ADC_SDA  gpio_num_t::GPIO_NUM_21

//motor
#define PIN_MOTOR_POWER     gpio_num_t::GPIO_NUM_16  //motor power (pwm) signal
#define PIN_MOTOR_DIR_0     gpio_num_t::GPIO_NUM_17  //motor bridge dir 0 pin
#define PIN_MOTOR_DIR_1     gpio_num_t::GPIO_NUM_18  //motor bridge dir 1 pin



#endif // TWID32_PIN_DEFS_H_