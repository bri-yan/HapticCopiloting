/**
 * @file encoder.h
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
#include "driver/gpio.h"

//pcnt
#include "driver/pcnt.h"

//config
#include "app/twid32_config.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define DEGREES_TO_RADIANS 0.01745329251
#define RADIANS_TO_DEGREES 57.2957795131

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

typedef struct {
    gpio_num_t quad_pin_a;
    gpio_num_t quad_pin_b;
    pcnt_unit_t pcnt_unit;
    uint16_t filter;

    //to be updated in ISRs
    //don't access this without proper thread safety
    volatile int64_t encoder_accu_cnt;
} encoder_context_t;

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

//encoder handles
extern encoder_context_t encoder1_handle;
extern encoder_context_t encoder2_handle;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Init encoder
 * 
 * Inits pins and counter value as input pullup and setup up PCNT timer and interrupts for quaderatrue decodingr
 *
 */
void encoder_init(encoder_context_t*);

/**
 * @brief Pause the encoder counting
 *
 */
void encoder_pause(encoder_context_t*);

/**
 * @brief Terminate encoder
 *
 */
void encoder_terminate(encoder_context_t*);

/**
 * @brief Get encoder counter value
 *
 */
int64_t encoder_get_count(encoder_context_t*);

/**
 * @brief Reset encoder count to 0
 */
void encoder_clear_count(encoder_context_t*);

/**
 * @brief Returns encoder angle in degrees
 */
double encoder_get_angle(encoder_context_t*);

/**
 * @brief Returns encoder angle in radians
 */
double encoder_get_angle_rad(encoder_context_t*);


#endif // ENCODER_H_