/**********************************************************************************************
 * Twiddlerino Library - Version 0.1a
 * by Oliver Schneider (oschneid@cs.ubc.ca)
 * modified by Bereket Guta (bguta@cs.ubc.ca)
 **********************************************************************************************/
#include "Twiddlerino.h"

//Encoder control pin numbers
int OutputEnable = A0;
int Select = A1;
int ResetEncoder = A2;

//Motor control
const int MOTOR_LEFT = 0; 
const int MOTOR_RIGHT = 1;
const int MOTOR_LOW = 2;
int motor_direction = MOTOR_LOW;
void SetMotorDirection(int);
void SetMotorSpeed(int);


const float EPSILON = 1;


void TwiddlerinoInit() {
  
  // DATA BUS
  //used to read in a byte from the encoder
  for(int pin = 2; pin <= 9; pin++) {
    pinMode(pin, INPUT);
  }
    
  // MOTOR
  // 11 and 12 control motor direction
  for(int pin = 10; pin <= 12; pin++) {
    pinMode(pin, OUTPUT);
  }
  
  // ENCODER
  pinMode(OutputEnable, OUTPUT); 
  pinMode(Select, OUTPUT); 
  pinMode(ResetEncoder, OUTPUT); 
  

  //Make pins 9 and 10 output PWM at 32 kHz
  TCCR1B &= 0b11111000; //Zero out the first three bits of the register
  int prescalerVal = 1; //disables prescaler, timer runs at 32kHz
  TCCR1B |= prescalerVal; //Sets the register*/
  
 // Don't use ANALOG OUT 11 or 3
  //Setup internal timers
  TCCR2B &= 0b11111000;
  //TCCR2B |= 3; //clkTS2/32 = 500 kHz.  Timer 2 now increments TCNT2 every 2 uS
  TCCR2B |= 4; //clkTS2/64 = 250 kHz.  Timer 2 now increments TCNT2 every 4 uS


  //disable encoder output, enabling the encoder value to be updated
  digitalWrite(OutputEnable, 1); 
  
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
int ReadEncoder()
{
  //Enable data output
  //This prevents the encoder value from changing?
  digitalWrite(OutputEnable, 0);
  
  //tell the encoder we want the high byte
  digitalWrite(Select, 0);
  //read the high byte value
  int high_byte = 0;
  for (int i = 9; i >= 3; i--) {
      high_byte += digitalRead(i);
      high_byte = high_byte << 1;
  }
  high_byte += digitalRead(2); //Need to avoid shifting the last time

  
  //tell the encoder we want the low byte
  digitalWrite(Select, 1);
  //read the high byte value
  int low_byte = 0;
  for (int i = 9; i >= 3; i--) {
      low_byte += digitalRead(i);
      low_byte = low_byte << 1;
   }
  low_byte += digitalRead(2);

  
  //disable data output, allowing encoder value to be updated
  digitalWrite(OutputEnable, 1);
  
  //compute the final value of the encoder
  int encoder_value = (high_byte<<8) + low_byte;
  
  return encoder_value;
  
}

int ReadEncoderFast(){
  //Enable data output
  //This prevents the encoder value from changing?
  digitalWrite(OutputEnable, 0);

  //tell the encoder we want the high byte
  digitalWrite(Select, 0);
  int high_byte = ((PINB & 0x03) << 6) | // B1-B0 are at bits 0-1 of PORTB
                  ((PIND & 0xFC) >> 2);  // D2-D7 are at bits 2-7 of PORTD

  //tell the encoder we want the low byte
  digitalWrite(Select, 1);
  int low_byte = ((PINB & 0x03) << 6) |
                  ((PIND & 0xFC) >> 2);

  //disable data output, allowing encoder value to be updated
  digitalWrite(OutputEnable, 1);

  //compute the final value of the encoder
  return (high_byte<<8) + low_byte;
}

//This resets the encoder's position value to 0
void ZeroEncoder()
{
  //Apparanetly I have to do both of these. 
  //The documentation implies that I would only need to do "LOW", but there ya go
  digitalWrite(ResetEncoder, LOW);
  digitalWrite(ResetEncoder, HIGH); 
}

void SetMotorDirection(int md)
{
   switch(md)
   {
     case MOTOR_LEFT:
       digitalWrite(11, HIGH);
       digitalWrite(12, LOW);
       break;
     case MOTOR_RIGHT:
        digitalWrite(11, LOW);
        digitalWrite(12, HIGH);
       break;
     case MOTOR_LOW:
        digitalWrite(11, LOW);
        digitalWrite(12, LOW);
        break;
     default:
       Serial.println("ERROR, incorrect motor direction!");
   }

}


void SetMotorSpeed(int spd)
{
  analogWrite(10, spd);
}

void SetPWMOut(int pwm_out)
{
  pwm_out = min(255, max(-255, pwm_out));
  if (pwm_out == 0)
  {
    SetMotorDirection(MOTOR_LOW);
  }
  else if (pwm_out < 0)
  {
    SetMotorDirection(MOTOR_LEFT);
    SetMotorSpeed(abs(pwm_out));
  } else
  {
    SetMotorDirection(MOTOR_RIGHT);
    SetMotorSpeed(abs(pwm_out));  
  }
}

