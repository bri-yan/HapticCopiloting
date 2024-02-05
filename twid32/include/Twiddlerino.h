#ifndef Twiddlerino_h
#define Twiddlerino_h

#include <stdint.h>
#include "Encoder.h"

//initializes Twiddlerino
void TwiddlerinoInit();

//Sets the output to the motor (between -255 and 255)
void SetPWMOut(int32_t dc);

#endif

