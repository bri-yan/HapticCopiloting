/**
 * @file motor.h
 * @brief Motor driver
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

#ifndef MOTOR_H_
#define MOTOR_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//for int types
#include <stdint.h>

#include "hal/gpio_types.h"

#include "driver/ledc.h"


/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define MOTOR_DUTY_CYCLE_RES_BITS   LEDC_TIMER_10_BIT
#define MOTOR_DUTY_CYCLE_RES        (1024) //10 BIT

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

typedef struct {
    gpio_num_t power_pin;
    gpio_num_t dir_pin;
    ledc_channel_t pwm_channel;
    motor_state_t state;
    const char* handle_name;
} motor_driver_context_t;

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

//motor contexts
extern motor_driver_context_t motor1_handle;
extern motor_driver_context_t motor2_handle;

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Init motor on power and directions pins
 *
 */
void motor_init(motor_driver_context_t*);

/**
 * @brief Sets motor state to low
 *
 */
void motor_stop(motor_driver_context_t*);

motor_state_t motor_set_state(motor_driver_context_t*, motor_state_t);

motor_state_t motor_get_state(motor_driver_context_t*);

/**
 * @brief PWM frequency
 *
 */
uint32_t motor_get_frequency(motor_driver_context_t*);

/**
 * @brief PWM duty cycle
 *
 */
uint32_t motor_get_duty_cycle(motor_driver_context_t*);

/**
 * @brief Sets motor pwm duty cycle
 *  Drive direction is also set based on sign of duty cycle (drive left for dc < 0)
 */
int32_t motor_set_pwm(motor_driver_context_t*, int32_t dc);

/**
 * @brief Stops motor and reset esp32 if speed exceeds MOTOR_MAX_SPEED_DEGS
 */
void motor_safety_check(motor_driver_context_t*, double speed);

void motor_fast_stop(motor_driver_context_t*);


#endif // MOTOR_H_