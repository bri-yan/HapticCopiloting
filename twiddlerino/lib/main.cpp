#include <Arduino_FreeRTOS.h>
#include "Twiddlerino.h"
#include "PID_v1.h"

#include <queue.h>

// define tasks
void TaskEncoderRead( void *pvParameters );
void TaskPID( void *pvParameters );
void TaskSerialRead( void *pvParameters );
void TaskSerialWrite( void *pvParameters );

//Global variables
double target=0;
double pos = 0;
double last_pos = 0;
double vel = 0;
double vel_hat = 0;
double PWMOut = 0;
int tmp=0;
long updateInterval = 10; //send position value every 10 ms
long last_time = 0; //last time we sent a position value

//define initial p, i, and d values
double p = 0.4;
double i = 0.0;
double d = 0.0;

//how many bytes Processing will write to the serial
//when passing p, i, d, and target updates
int SERIAL_WRITE_LENGTH = 32;

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication
  Serial.begin(115200);

  Serial.println("USB Connected");

  //init twidlerino encoder sensor and motor
  TwiddlerinoInit();

  Serial.println("Twiddlerino Initiliazed");

  // Now set up freertos tasks
  xTaskCreate(
    TaskPID
    ,  "PID Control"
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  
  xTaskCreate(
    TaskEncoderRead
    ,  "Encoder Read"
    ,  128
    ,  NULL
    ,  2
    ,  NULL );

  xTaskCreate(
    TaskSerialRead
    ,  "Serial Read"
    ,  128  
    ,  NULL
    ,  1
    ,  NULL );

  xTaskCreate(
    TaskSerialWrite
    ,  "Serial Write"
    ,  128  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  NULL );

    Serial.println("Created FreeRTOS Tasks");

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Leave Empty
  //things are done in the freertos tasks
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskEncoderRead(void *pvParameters)
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    pos = ReadEncoderFast();
    vTaskDelay(1); // wait one tick delay (15 ms)
  }
}

void TaskPID(void *pvParameters)
{
  (void) pvParameters;

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000;
  BaseType_t xWasDelayed;

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount ();

  // Setup the PID. It will automatically noticed changes to
  // input (pos) and target, and modify output (PWMOut).
  // This is why they are passed by reference.
  // Reverse means to apply a force opposite of the error (to correct for it).
  PID myPID(&pos, &PWMOut, &target, p, i, d, REVERSE);

  //setpoint starts at 0
  target = 0;
  myPID.SetOutputLimits(-255, 255); //for PWM
  myPID.SetMode(AUTOMATIC); //just need to call myPID.compute() and things will work
  myPID.SetSampleTime(1); //sample time = 1 rtos tick = 15 ms

  for (;;)
  {
    // Wait for the next cycle.
    xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
    //PID class handles its own update rate with millis()
    if(myPID.Compute())
    {
      //only update PWM out if the controller output changes
      SetPWMOut(PWMOut);
      Serial.println(xWasDelayed);
    }
  }
}

void TaskSerialRead(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    if(Serial.availableForWrite())
    {
      Serial.print("Position: ");
      Serial.print((int)pos);
      Serial.println();
    }

    //print position 4 times a second
    vTaskDelay( 250 / portTICK_PERIOD_MS);
  }
}

void TaskSerialWrite(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    if(Serial.available())
    {
      byte b = Serial.read();
      Serial.print("read 1 byte: ");
      Serial.print(b);
      Serial.println();

      if(b == 'p')
      {
        Serial.println("processing floats");
        char inData[SERIAL_WRITE_LENGTH];
        Serial.readBytes(inData, SERIAL_WRITE_LENGTH);
        Serial.print("Read float ");
        Serial.print((int)atoi(inData));
        Serial.println();
      }
    }

    vTaskDelay(1);
  }
}