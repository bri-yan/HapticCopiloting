/**
 * @file Twiddlerino.cpp
 * @version 1.0a
 * by Oliver Schneider (oschneid@cs.ubc.ca)
 * modified by Bereket Guta (bguta@cs.ubc.ca)
 * modified for ESP32 port by Yousif El-Wishahy (ywishahy@student.ubc.ca) 02-2024
*/

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "Twiddlerino.h"

//arduino
#include "Arduino.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define ENCODER_DATA_LENGTH 8U

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

// Typedefs that are only used in this file, empty for now

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

static void SetMotorDirection(uint8_t md);
static void SetMotorSpeed(uint8_t spd);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

// Pin Definitions

static const gpio_num_t EncoderQuadAPin = ((gpio_num_t) 15);
static const gpio_num_t EncoderQuadBPin = ((gpio_num_t) 2);

static const gpio_num_t MotorPowerPin   = ((gpio_num_t) 13);
static const gpio_num_t MotorDir0Pin    = ((gpio_num_t) 12);
static const gpio_num_t MotorDir1Pin    = ((gpio_num_t) 14);


// static const uint8_t OutputEnablePin = 15;
// static const uint8_t SelectPin = 2;
// static const uint8_t ResetEncoderPin = 4;

// //Encoder read pins
// static const uint8_t EncoderDataPins[ENCODER_DATA_LENGTH] = {36,39,34,35,32,33,25,26};

//motor pwm timer variables
static const uint32_t mPwmFreq = 32000; //pwm frequency
static const uint8_t mPwmDutyCycleRes = 8; //pwm resolution (8bits=0-255)
static const uint8_t mPwmChan = 0;//harware timer channel for pwm signal

//motor control state variables
static const uint8_t MOTOR_LEFT = 0; 
static const uint8_t MOTOR_RIGHT = 1;
static const uint8_t MOTOR_LOW = 2;
static uint8_t motor_direction = MOTOR_LOW;
static const float EPSILON = 1;

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

// Global variable definitions matching the extern definitions in the header

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void TwiddlerinoInit() {
    
  // MOTOR
  pinMode(MotorDir0Pin, OUTPUT);
  pinMode(MotorDir1Pin, OUTPUT);

  //setup motor PWM output on MotorPowerPin
  //this uses the esp32 idf ledc driver
  //the ledc driver is writting to power leds 
  // but can also gen pwm signals for other purposes
  ledcSetup(mPwmChan, mPwmFreq, mPwmDutyCycleRes); //channel 0, 32khz, 8 bit (0-255) duty cycle resolution
  ledcAttachPin(MotorPowerPin, mPwmChan);//attack motor power pin to channel 0 timer
  ledcWrite(mPwmChan, 0);//start at duty cycle of 0 (analog voltage ~ 0)
  
  SetMotorDirection(MOTOR_RIGHT);
}

void SetPWMOut(int32_t dc)
{
  uint8_t pwmDuty = min(255, max(-255, dc));
  if (pwmDuty == 0)
  {
    SetMotorDirection(MOTOR_LOW);
  }
  else if (pwmDuty < 0)
  {
    SetMotorDirection(MOTOR_LEFT);
    SetMotorSpeed(abs(pwmDuty));
  } else
  {
    SetMotorDirection(MOTOR_RIGHT);
    SetMotorSpeed(abs(pwmDuty));  
  }
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void SetMotorDirection(uint8_t md)
{
   switch(md)
   {
     case MOTOR_LEFT:
       digitalWrite(MotorDir0Pin, HIGH);
       digitalWrite(MotorDir1Pin, LOW);
       break;
     case MOTOR_RIGHT:
        digitalWrite(MotorDir0Pin, LOW);
        digitalWrite(MotorDir1Pin, HIGH);
       break;
     case MOTOR_LOW:
        digitalWrite(MotorDir0Pin, LOW);
        digitalWrite(MotorDir1Pin, LOW);
        break;
     default:
       Serial.println("ERROR, incorrect motor direction!");
   }
}


static void SetMotorSpeed(uint8_t spd)
{
  ledcWrite(mPwmChan, spd);
}

