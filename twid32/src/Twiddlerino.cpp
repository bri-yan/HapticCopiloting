/**********************************************************************************************
 * Twiddlerino Library - Version 0.1a
 * by Oliver Schneider (oschneid@cs.ubc.ca)
 * modified by Bereket Guta (bguta@cs.ubc.ca)
 **********************************************************************************************/
#include "Twiddlerino.h"

//Encoder control pin numbers
static const uint8_t OutputEnablePin = 15;
static const uint8_t SelectPin = 2;
static const uint8_t ResetEncoderPin = 4;

//Encoder read pins
#define ENCODER_DATA_LENGTH 8U
static const uint8_t EncoderDataPins[ENCODER_DATA_LENGTH] = {36,39,34,35,32,33,25,26};

//Motor Control Pins
static const uint8_t MotorPowerPin = 27;
static const uint8_t MotorDir0Pin = 14;
static const uint8_t MotorDir1Pin = 12;

//motor control variables
static const uint32_t mPwmFreq = 32000; //pwm frequency
static const uint8_t mPwmDutyCycleRes = 8; //pwm resolution (8bits=0-255)
static const uint8_t mPwmChan = 0;//harware timer channel for pwm signal

//Motor states
const int MOTOR_LEFT = 0; 
const int MOTOR_RIGHT = 1;
const int MOTOR_LOW = 2;
int motor_direction = MOTOR_LOW;

//motor functions
void SetMotorDirection(int);
void SetMotorSpeed(uint8_t spd);




const float EPSILON = 1;


void TwiddlerinoInit() {
  
  // DATA BUS
  //used to read in a byte from the encoder
  for(int i =0; i < ENCODER_DATA_LENGTH; i++) {
    pinMode(EncoderDataPins[i], INPUT);
  }
    
  // MOTOR
  pinMode(MotorPowerPin, OUTPUT);
  pinMode(MotorDir0Pin, OUTPUT);
  pinMode(MotorDir1Pin, OUTPUT);

  //setup motor PWM output on MotorPowerPin
  //this uses the esp32 idf ledc driver
  //the ledc driver is writting to power leds 
  // but can also gen pwm signals for other purposes
  ledcSetup(mPwmChan, mPwmFreq, mPwmDutyCycleRes); //channel 0, 32khz, 8 bit (0-255) duty cycle resolution
  ledcAttachPin(MotorPowerPin, mPwmChan);//attack motor power pin to channel 0 timer
  ledcWrite(mPwmChan, 0);//start at duty cycle of 0 (analog voltage ~ 0)
  
  // ENCODER
  pinMode(OutputEnablePin, OUTPUT); 
  pinMode(SelectPin, OUTPUT); 
  pinMode(ResetEncoderPin, OUTPUT);
  

  //old code from arduino due timer register setup,keeping for reference in case i run into issues

  //   //Make pins 9 and 10 output PWM at 32 kHz
  //   TCCR1B &= 0b11111000; //Zero out the first three bits of the register
  //   int prescalerVal = 1; //disables prescaler, timer runs at 32kHz
  //   TCCR1B |= prescalerVal; //Sets the register*/
    
  //  // Don't use ANALOG OUT 11 or 3
  //   //Setup internal timers
  //   TCCR2B &= 0b11111000;
  //   //TCCR2B |= 3; //clkTS2/32 = 500 kHz.  Timer 2 now increments TCNT2 every 2 uS
  //   TCCR2B |= 4; //clkTS2/64 = 250 kHz.  Timer 2 now increments TCNT2 every 4 uS


  //disable encoder output, enabling the encoder value to be updated
  digitalWrite(OutputEnablePin, 1); 
  
  //initial reset of the encoder
  ZeroEncoder();
  SetMotorDirection(MOTOR_RIGHT);
}


// ReadEncoder()
//
// reads the encoder's value, returns it as an int
//
//The bus to the encoder is only 8 bits, but it has a 16 bit value
//therefore, to read the encoder we have to lock the encoder from changing,
//get each part of the 16-bit word, and then combine them.
//
//This function is written for readability, not efficiency
uint16_t ReadEncoder()
{
  //Enable data output
  //This prevents the encoder value from changing?
  digitalWrite(OutputEnablePin, 0);
  
  //tell the encoder we want the high byte
  digitalWrite(SelectPin, 0);

  //read the high byte value
  uint8_t high_byte = 0;
  for (int i = ENCODER_DATA_LENGTH - 1 ; i >= 1 ; i--) {
      high_byte += digitalRead(EncoderDataPins[i]);
      high_byte = high_byte << 1;
  }
  high_byte += digitalRead(EncoderDataPins[0]); //Need to avoid shifting the last time

  
  //tell the encoder we want the low byte
  digitalWrite(SelectPin, 1);

  //read the high byte value
  uint8_t low_byte = 0;
  for (int i = ENCODER_DATA_LENGTH - 1 ; i >= 1 ; i--) {
      low_byte += digitalRead(EncoderDataPins[i]);
      low_byte = low_byte << 1;
   }
  low_byte += digitalRead(2);

  
  //disable data output, allowing encoder value to be updated
  digitalWrite(OutputEnablePin, 1);
  
  //compute the final value of the encoder
  uint16_t encoder_value = (high_byte<<8) + low_byte;
  
  return encoder_value;
  
}

// uint16_t ReadEncoderFast(){
//   //Enable data output
//   //This prevents the encoder value from changing?
//   digitalWrite(OutputEnablePin, 0);

//   //tell the encoder we want the high byte
//   digitalWrite(SelectPin, 0);
//   int high_byte = ((PINB & 0x03) << 6) | // B1-B0 are at bits 0-1 of PORTB
//                   ((PIND & 0xFC) >> 2);  // D2-D7 are at bits 2-7 of PORTD

//   //tell the encoder we want the low byte
//   digitalWrite(SelectPin, 1);
//   int low_byte = ((PINB & 0x03) << 6) |
//                   ((PIND & 0xFC) >> 2);

//   //disable data output, allowing encoder value to be updated
//   digitalWrite(OutputEnablePin, 1);

//   //compute the final value of the encoder
//   return (high_byte<<8) + low_byte;
// }

//This resets the encoder's position value to 0
void ZeroEncoder()
{
  //Apparanetly I have to do both of these. 
  //The documentation implies that I would only need to do "LOW", but there ya go
  digitalWrite(ResetEncoderPin, LOW);
  digitalWrite(ResetEncoderPin, HIGH); 
}

void SetMotorDirection(int md)
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


void SetMotorSpeed(uint8_t spd)
{
  ledcWrite(mPwmChan, spd);
}

void SetPWMOut(int pwm_out)
{
  uint8_t pwmDuty = min(255, max(-255, pwm_out));
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

