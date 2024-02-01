#include "Twiddlerino.h"
#include "PID_v1.h"

//local function definitions
static float ReadFloat();
static int ReadInt();
double getVelocity();
double getAcceleration();
double getCurrent();
double getUserTorque();

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
double p = 3.5;
double i = 0.0;
double d = 0.02;

// Setup the PID. It will automatically noticed changes to
// input (pos) and target, and modify output (PWMOut).
// This is why they are passed by reference.
// Reverse means to apply a force opposite of the error (to correct for it).
PID myPID(&pos, &PWMOut, &target, p, i, d, REVERSE);


//how many bytes Processing will write to the serial
//when passing p, i, d, and target updates
int SERIAL_WRITE_LENGTH = 32;

extern int motor_direction;

void setup()
{
  Serial.begin(115200);
  TwiddlerinoInit();
  target = 0;
  myPID.SetOutputLimits(-255, 255); //for PWM
  myPID.SetMode(AUTOMATIC); //just need to call myPID.compute() and things will work
  myPID.SetSampleTime(1); //every 1 ms
  last_time = millis();

  pinMode(A4, INPUT);
}

int itr = 0;
const int loopsPerMeasurement = 5;
const int numMeasurements = 2 * loopsPerMeasurement + 1;
int positionVals[numMeasurements] = {};
unsigned long timeVals[numMeasurements] = {};

int currentItr = 0;
const int numCurrentMeasurements = 3;
int currentVals[numCurrentMeasurements] = {};

void loop()
{
  target = 2000 * sin((double) millis() / 500); // Update trajectory

  if (++itr == numMeasurements) {
    itr = 0;

    double Ka = 5.0;
    double Kv = 0.1;
    //Serial.println("Expected: " + String((Ka * getAcceleration()) + (Kv * getVelocity())) + " Actual: " + String(getCurrent()));
  }
  pos = ReadEncoderFast();
  positionVals[itr] = pos;
  timeVals[itr] = micros();

  if (++currentItr == numCurrentMeasurements) {
    currentItr = 0;
    //Serial.println("Expected: " + String(0) + " Actual: " + String(getCurrent()));
    Serial.println(String(getCurrent()));
  }
  int instantaneousCurrent = analogRead(A4);
  if (motor_direction == 1) instantaneousCurrent *= -1; // 0 means CCW
  currentVals[currentItr] = instantaneousCurrent;

  //Write position every updateInterval ms
  long t = millis();
  long dt = t - last_time;
  if (dt >= updateInterval)
  {  
    // Serial.println((int)pos);
    // Send position immediately
    int position = (int)pos;
    bool isNegative = position < 0;
    if (isNegative) {
      position = abs(position); // If negative, make it positive for transmission
    }
    
    // Send sign bit, low byte, and high byte immediately
    byte signBit = isNegative ? 1 : 0;
    // Serial.write(signBit);
    // Serial.write(position & 0xFF);    // Low byte
    // Serial.write((position >> 8) & 0xFF); // High byte
    last_time = t;
  }
  
  //read any changes to PID
  boolean changed = false;
  while (Serial.available())
  {
    byte b = Serial.read();
    if (b == 'p') {
      p = ReadFloat();
      changed = true;
    } else if (b == 'i') {
      i = ReadFloat();
      changed = true;
    } else if (b == 'd') {
      d = ReadFloat();
      changed = true;
    } else if (b == 't') {
      target = ReadFloat();
      changed = true;
    }
  }
  if(changed)
  {
    myPID.SetTunings(p, i, d);
  }

  // long write_dt = t - write_last_time;
  // int receivedValue = target;
  // if (write_dt >= updateInterval) {
  //   if (Serial.available() > 0) {
  //     receivedValue = ReadInt();
  //   }
  //   Serial.println(receivedValue);
  // }
  
  //PID class handles its own update rate
  myPID.Compute();
  
  //Write to Twiddlerino
  SetPWMOut(PWMOut);
}

double getVelocity() {
  int numRollingAvg = numMeasurements / 5;

  double velocity = 0.0;
  for (int j = 0; j < numRollingAvg; ++j) {
    int rollingAvgIdx2 = itr - j;
    if (rollingAvgIdx2 < 0) {
      rollingAvgIdx2 = numMeasurements + rollingAvgIdx2;
    } 
    int rollingAvgIdx1 = rollingAvgIdx2 + 1;
    if (rollingAvgIdx1 == numMeasurements) {
      rollingAvgIdx1 = 0;
    }

    double dx = positionVals[rollingAvgIdx2] - positionVals[rollingAvgIdx1];
    double dt = ((double)(timeVals[rollingAvgIdx2] - timeVals[rollingAvgIdx1])) / 20000.0;
    velocity += dx / dt;
  }
  return velocity / ((double) numRollingAvg);
}

double getAcceleration() {
  int numRollingAvg = numMeasurements / 5;

  // Get most recent average velocity
  double velocity2 = 0.0;
  for (int j = 0; j < numRollingAvg; ++j) {
    int rollingAvgIdx2 = itr - j;
    if (rollingAvgIdx2 < 0) {
      rollingAvgIdx2 = numMeasurements + rollingAvgIdx2;
    } 
    int rollingAvgIdx1 = rollingAvgIdx2 - numRollingAvg;
    if (rollingAvgIdx1 < 0) {
      rollingAvgIdx1 = numMeasurements + rollingAvgIdx1;
    }

    double dx = positionVals[rollingAvgIdx2] - positionVals[rollingAvgIdx1];
    double dt = ((double)(timeVals[rollingAvgIdx2] - timeVals[rollingAvgIdx1])) / 20000.0;
    velocity2 += dx / dt;
  }

  // Get older average velocity
  double velocity1 = 0.0;
  for (int j = 0; j < numRollingAvg; ++j) {
    int rollingAvgIdx2 = itr - (numMeasurements / 2) - j;
    if (rollingAvgIdx2 < 0) {
      rollingAvgIdx2 = numMeasurements + rollingAvgIdx2;
    } 
    int rollingAvgIdx1 = rollingAvgIdx2 - numRollingAvg;
    if (rollingAvgIdx1 < 0) {
      rollingAvgIdx1 = numMeasurements + rollingAvgIdx1;
    }

    double dx = positionVals[rollingAvgIdx2] - positionVals[rollingAvgIdx1];
    double dt = ((double)(timeVals[rollingAvgIdx2] - timeVals[rollingAvgIdx1])) / 20000.0;
    velocity1 += dx / dt;
  }

  int t2Idx = itr;
  int t1Idx = (t2Idx == numMeasurements - 1) ? 0 : t2Idx + 1;
  
  return (velocity2 - velocity1) / (((double)(timeVals[t2Idx] - timeVals[t1Idx])) / 10000.0);
}

double getCurrent() {
  double averagedCurrent;
  for (int j = 0; j < numCurrentMeasurements; ++j) {
    averagedCurrent += ((double) currentVals[j]) / ((double) numCurrentMeasurements);
  }
  return averagedCurrent;
}

double getUserTorque() {
  return 0;
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
