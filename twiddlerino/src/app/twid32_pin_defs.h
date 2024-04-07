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

//REF
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html#pin-layout

//encoders
#define PIN_ENCODER1_QUAD_A  gpio_num_t::GPIO_NUM_26  //Encoder 1 quaderture signal A
#define PIN_ENCODER1_QUAD_B  gpio_num_t::GPIO_NUM_25   //Encoder 1 quaderture signal B

#define PIN_ENCODER2_QUAD_A  gpio_num_t::GPIO_NUM_4  //Encoder 2 quaderture signal A
#define PIN_ENCODER2_QUAD_B  gpio_num_t::GPIO_NUM_27   //Encoder 2 quaderture signal B

//current sensor (communicates through i2c interface)
#define PIN_CURRENT_SENS_ADC_SCL  gpio_num_t::GPIO_NUM_22
#define PIN_CURRENT_SENS_ADC_SDA  gpio_num_t::GPIO_NUM_21
#define PIN_CURRENT_SENS_ADC_ALERT gpio_num_t::GPIO_NUM_19

#define PIN_CURRENT_SENS_1_ANALOG  gpio_num_t::GPIO_NUM_36 //this pin goes to channel ADC1_0
#define CURRENT_SENS_1_ADC_CHAN   adc_channel_t::ADC_CHANNEL_0

#define PIN_CURRENT_SENS_2_ANALOG  gpio_num_t::GPIO_NUM_34 //this pin goes to channel ADC2_5
#define CURRENT_SENS_2_ADC_CHAN   adc_channel_t::ADC_CHANNEL_6

//motors
#define PIN_MOTOR1_POWER     gpio_num_t::GPIO_NUM_16  //motor 1 power (pwm) signal
#define PIN_MOTOR1_DIR       gpio_num_t::GPIO_NUM_17  //motor 1 dir pin

#define PIN_MOTOR2_POWER     gpio_num_t::GPIO_NUM_23  //motor 2 power (pwm) signal
#define PIN_MOTOR2_DIR       gpio_num_t::GPIO_NUM_32  //motor 2 dir pin

#endif // TWID32_PIN_DEFS_H_