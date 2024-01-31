#ifndef Twiddlerino_h
#define Twiddlerino_h

#include "Arduino.h"

//initializes Twiddlerino
void TwiddlerinoInit();

//Reads the encoder returning a signed integer value
int ReadEncoder();
// Reads the encoder without using loops
int ReadEncoderFast();

//Resets the encoder to zero
void ZeroEncoder();

//Sets the output to the motor (between -255 and 255)
void SetPWMOut(int);

#endif

