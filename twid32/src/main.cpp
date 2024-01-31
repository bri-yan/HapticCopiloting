#include <Twiddlerino.h>
#include "Arduino.h"
#include <PID_v1.h>

//freertos tasks
void TaskUart0RW(void *pvParameters);
void TaskTwiddlerinoControl(void *pvParameters);

//global variables
static uint64_t last_time;
static double Kp=0.4, Ki=0, Kd=0;
static double Setpoint, Position, DutyCycle;

SemaphoreHandle_t xSemaphore;

void setup() {

  last_time = micros();
  
  //mutex for position variable
  xSemaphore = xSemaphoreCreateMutex();

  //start these tasks on core 1
  xTaskCreatePinnedToCore(
    TaskUart0RW
    ,  "Uart0 RW"
    ,  8192
    ,  NULL
    , 2
    , NULL
    , 1
  );

  xTaskCreatePinnedToCore(
    TaskTwiddlerinoControl
    ,  "Twiddlerino Control"
    ,  8192
    ,  NULL
    ,  5
    ,  NULL
    ,  0
  );
}

void loop() {
  // put your main code here, to run repeatedly:
}

void TaskUart0RW(void *pvParameters) {
  Serial.begin( 115200 );//this is on rx0 tx0
  
  Serial.println( "Serial connected on uart0!" );

  for(;;) 
  {
    if( Serial.available() ) {
      byte b = Serial.read();
      if(b == 'w'){
        Serial.print('a');
        char bytes[4];

        while(!Serial.available()){
          vTaskDelay(1);
        }

        b = Serial.read();
        Serial.readBytes(bytes,4);


        if(b=='P'){
          Kp = atof(bytes);
        } else if (b=='I'){
          Ki = atof(bytes);
        } else if (b=='D'){
          Kd = atof(bytes);
        } else if (b=='S'){
          double val = atof(bytes);
          if(xSemaphoreTake(xSemaphore, 10) == pdTRUE){
            Setpoint = val;
            xSemaphoreGive(xSemaphore);
          }
        }

        Serial.printf("***********************\nUpdate for %c with int value: %d\n**************************\n",b,atoi(bytes));
      } else if(b=='r') {
        while(!Serial.availableForWrite()) {
          vTaskDelay(10);
        }

        if(xSemaphoreTake(xSemaphore, 10) == pdTRUE){
          Serial.printf("Encoder Position: %d\tDuty Cycle: %d\tSet Point:%d\n",Position,DutyCycle,Setpoint);
          xSemaphoreGive(xSemaphore);
        }
      }
    }
    
    vTaskDelay( 1 );
  }
}

void TaskTwiddlerinoControl(void *pvParameters){

  ledcSetup(0,32000,8); //channel 0, 32khz, 8 bit (0-255) duty cycle resolution
  ledcAttachPin(27,0);//attack motor power pin to channel 0 timer
  ledcWrite(0,120);//start at duty cycle of 0 (analog voltage ~ 0)

  TwiddlerinoInit();

  Setpoint = 100;
  DutyCycle = 0;

  //position is being read from other cores (maybe)
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  Position = 0;
  PID myPID(&Position, &DutyCycle, &Setpoint, Kp, Ki, Kd, AUTOMATIC);
  xSemaphoreGive(xSemaphore);

  myPID.SetSampleTime(1);

  for(;;) 
  {
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    Position = ReadEncoder();
    myPID.Compute();
    SetPWMOut(DutyCycle);
    xSemaphoreGive(xSemaphore);

    vPortYield();
  }
}