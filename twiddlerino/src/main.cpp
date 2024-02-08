#include "Twiddlerino.h"
#include "PID_v1.h"

//local function definitions
static float ReadFloat();
static int ReadInt();

//Define Variables we'll be connecting to
double target = 0;
double pos = 0;
double last_pos = 0;
double vel = 0;
double vel_hat = 0;
double PWMOut = 0;
int tmp=0;

long updateInterval = 10; //send position value every 10 ms
long last_time = 0; //last time we sent a position value
long write_last_time = 5;

//define initial p, i, and d values
double p = 0.4;
double i = 0.0;
double d = 0.0;

// Setup the PID. It will automatically noticed changes to
// input (pos) and target, and modify output (PWMOut).
// This is why they are passed by reference.
// Reverse means to apply a force opposite of the error (to correct for it).
PID myPID(&pos, &PWMOut, &target, p, i, d, REVERSE);


//how many bytes Processing will write to the serial
//when passing p, i, d, and target updates
int SERIAL_WRITE_LENGTH = 4;

void setup()
{
  Serial.begin(115200);
  TwiddlerinoInit();
  target = 0;
  myPID.SetOutputLimits(-255, 255); //for PWM
  myPID.SetMode(AUTOMATIC); //just need to call myPID.compute() and things will work
  myPID.SetSampleTime(1); //every 1 ms
  last_time = millis();
}

void loop()
{
  if (Serial.available())
  {
    byte command = Serial.read();
    if (command == 'r') {
      Serial.write('a');
      pos = ReadEncoderFast();
      int position = (int)pos;
      bool isNegative = position < 0;
      if (isNegative) {
        position = abs(position); // If negative, make it positive for transmission
      }
      byte signBit = isNegative ? 1 : 0;
      Serial.write(signBit);
      Serial.write(position & 0xFF);    // Low byte
      Serial.write((position >> 8) & 0xFF); // High byte
    } else if (command == 'w') {
      target = ReadFloat();
      Serial.write('a');  
    }
  }

  myPID.Compute();
  
  SetPWMOut(PWMOut);
}

int ReadInt()
{
  char inData[SERIAL_WRITE_LENGTH];
  Serial.readBytes(inData, SERIAL_WRITE_LENGTH);
  return atoi(inData); //Inefficient, but it'll do for now.
}

float ReadFloat()
{
    char inData[SERIAL_WRITE_LENGTH];
    Serial.readBytes(inData, SERIAL_WRITE_LENGTH);
    return atof(inData);
}
