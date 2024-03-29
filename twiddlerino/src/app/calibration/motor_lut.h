/**
 * @file motor_lut.h
 * @brief Motor calibration look up tables
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef MOTOR_LUT_H_
#define MOTOR_LUT_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include <stdint.h>

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

/**
 * @brief Lookup expected motor current at no load for specific duty cycle
 *        Lookup table from calibration.
 *        1d interpolation is used.
 *        If duty cycle is out of range MOTOR_DUTY_CYCLE_RES, will return max current
 * 
 */
double lookup_exp_current(double dc);

#endif //MOTOR_LUT_H_