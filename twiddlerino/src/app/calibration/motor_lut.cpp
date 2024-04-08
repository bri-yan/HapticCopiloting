/**
 * @file motor_lut.cpp
 * @brief Motor calibration look up tables
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//header for this file
#include "app/calibration/motor_lut.h"

#include "app/twid32_config.h"

#include "drivers/motor.h"


/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define LUT_LENGTH 50U

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

//lookup values for dutycycle (0->1024)
static float lut_dc[LUT_LENGTH] = {
    0.0,
    20.0,
    41.0,
    62.0,
    83.0,
    104.0,
    125.0,
    146.0,
    167.0,
    188.0,
    208.0,
    229.0,
    250.0,
    271.0,
    292.0,
    313.0,
    334.0,
    355.0,
    376.0,
    397.0,
    417.0,
    438.0,
    459.0,
    480.0,
    501.0,
    522.0,
    543.0,
    564.0,
    585.0,
    606.0,
    626.0,
    647.0,
    668.0,
    689.0,
    710.0,
    731.0,
    752.0,
    773.0,
    794.0,
    815.0,
    835.0,
    856.0,
    877.0,
    898.0,
    919.0,
    940.0,
    961.0,
    982.0,
    1003.0,
    1024.0,
    };

//lookup values for current (in amps)
static float lut_current[LUT_LENGTH] = {
    -0.005513,
    0.07307,
    0.164064,
    0.125461,
    0.183365,
    0.190258,
    0.15717,
    0.181985,
    0.191636,
    0.206802,
    0.186123,
    0.190258,
    0.184743,
    0.186123,
    0.198531,
    0.201287,
    0.206802,
    0.208181,
    0.226103,
    0.221968,
    0.21921,
    0.239891,
    0.228861,
    0.23024,
    0.224725,
    0.253677,
    0.234376,
    0.231618,
    0.246784,
    0.246784,
    0.257813,
    0.259193,
    0.272979,
    0.275736,
    0.278494,
    0.275736,
    0.274357,
    0.281252,
    0.284007,
    0.277114,
    0.278494,
    0.279872,
    0.306066,
    0.288145,
    0.297795,
    0.300553,
    0.304688,
    0.301931,
    0.304688,
    0.310204,
};

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

double lookup_exp_current(double dc) {
    if (dc > MOTOR_DUTY_CYCLE_RES || dc < -MOTOR_DUTY_CYCLE_RES) {
        return lut_current[LUT_LENGTH-1];
    }

    //linear interpolation
    //https://stackoverflow.com/questions/7091294/how-to-build-a-lookup-table-in-c-sdcc-compiler-with-linear-interpolation
    for(uint16_t i = 0; i < LUT_LENGTH-1; i++ )
    {
        if ( lut_dc[i] <= dc && lut_dc[i+1] >= dc )
        {
            float diffx = dc - lut_dc[i+1];
            float diffn = lut_dc[i+1] - lut_dc[i];
            return lut_current[i] + ( lut_current[i+1] - lut_current[i] ) * diffx / diffn; 
        }
    }

    return lut_current[LUT_LENGTH-1];
}