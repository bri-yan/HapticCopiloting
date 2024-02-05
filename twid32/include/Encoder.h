/**
 * @file Encoder.h
 * @brief Encoder driver for esp32 using pcnt peripherial to decode quaderature signals
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef ENCODER_H_
#define ENCODER_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//for int types
#include <stdint.h>

//for gpio types
#include "esp32-hal-gpio.h"

//pcnt
#include "driver/pcnt.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define ENCODER_CPR 500
#define DEGREES_TO_RADIANS 0.01745329251
#define RADIANS_TO_DEGREES 57.2957795131

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

// Public typedefs that may be used by other files

/******************************************************************************/
/*                       G L O B A L  V A R I A B L E S                       */
/******************************************************************************/

// Public global variables that may be used by other files
// (always declare extern)

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Init encoder
 * 
 * Inits pins and counter value as input pullup and setup up PCNT timer and interrupts for quaderatrue decoding
 * 
 * @param unit - counter peripheral unit number (PCNT_UNIT_0 up to PCNT_UNIT_7)
 * @param quadPinA, quadPinB - quaderature signals a and b physical gpio pins
 * @param filter - filter value to account for glitches/noise in quaderature signals, can be 0 for no filter
 *
 */
void encoder_init(pcnt_unit_t unit, gpio_num_t quadPinA, gpio_num_t quadPinB, uint16_t filter);

/**
 * @brief Pause the encoder counting
 *
 */
void encoder_pause();


/**
 * @brief Terminate encoder
 *
 */
void encoder_terminate();

/**
 * @brief Get encoder counter value
 *
 */
int64_t encoder_get_count();

/**
 * @brief Reset encoder count to 0
 */
void encoder_clear_count();

/**
 * @brief Returns encoder angle in degrees
 */
double encoder_get_angle();

/**
 * @brief Returns encoder angle in radians
 */
double encoder_get_angle_rad();


/******************************************************************************/
/*                       I N L I N E  F U N C T I O N S                       */
/******************************************************************************/

// Inline function declarations and implementions

#endif // ENCODER_H_