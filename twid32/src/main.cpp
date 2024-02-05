#include <Twiddlerino.h>
#include <PID_v1.h>
#include "Arduino.h"

//freertos tasks
void TaskUart0R(void *pvParameters);
void TaskUart0W(void *pvParameters);
void TaskTwiddlerinoControl(void *pvParameters);
// Encoder enc(36,39, encoder_mode_t::SINGLE);

//global variables
static uint64_t last_time;
static double Kp=0.4, Ki=0, Kd=0;
static double Setpoint, Position, DutyCycle;
PID myPID(&Position, &DutyCycle, &Setpoint, Kp, Ki, Kd, REVERSE);

SemaphoreHandle_t xSemaphore;
QueueHandle_t xQueueTelemetry;

typedef struct {
  uint32_t timestamp_millis;
  uint32_t dt;
  uint32_t read_dt;
  uint32_t control_dt;
  double pos;
  double duty_cycle;
  double set_point;
  bool pid_flag;
  int64_t encoder_count;
} telemetry_t;


void setup() {

  last_time = micros();
  
  //mutex for position variable
  xSemaphore = xSemaphoreCreateMutex();
  xQueueTelemetry = xQueueCreate(3, sizeof(telemetry_t));

  //start these tasks on core 1
  xTaskCreatePinnedToCore(
    TaskUart0W
    ,  "Uart0 Write"
    ,  8192
    ,  NULL
    , configMAX_PRIORITIES - 1
    , NULL
    , 0
  );

  // xTaskCreatePinnedToCore(
  //   TaskUart0R
  //   ,  "Uart0 Read"
  //   ,  8192
  //   ,  NULL
  //   , configMAX_PRIORITIES
  //   , NULL
  //   , 0
  // );

  xTaskCreatePinnedToCore(
    TaskTwiddlerinoControl
    ,  "Twiddlerino Control"
    ,  8192
    ,  NULL
    ,  configMAX_PRIORITIES
    ,  NULL
    ,  1
  );
}

void loop() {
  // put your main code here, to run repeatedly:
}

void TaskUart0R(void *pvParameters) {
  if(!Serial) {
    Serial.begin( 115200 );//this is on rx0 tx0
    Serial.println( "Serial connected on uart0!" );
  }

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

        if (b=='S'){
          double val = atof(bytes);
          if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE){
            Setpoint = val;
            xSemaphoreGive(xSemaphore);
          }
        } else {
          if(b=='P'){
            Kp = atof(bytes);
          } else if (b=='I'){
            Ki = atof(bytes);
          } else if (b=='D'){
            Kd = atof(bytes);
          }

          if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE){
            myPID.SetTunings(Kp,Ki,Kd);
            xSemaphoreGive(xSemaphore);
          }
        }

        Serial.printf("***********************\nUpdate for %c with int value: %d\n**************************\n",b,atoi(bytes));
      }
    }
    
    vTaskDelay( 1 );
  }
}

void TaskUart0W(void *pvParameters) {
  if(!Serial) {
    Serial.begin( 115200 );//this is on rx0 tx0
    Serial.println( "Serial connected on uart0!" );
  }

  telemetry_t telem;

  for(;;) 
  {
    // if( Serial.available() ) {
    //   byte b = Serial.read();
    //   if(b == 'r' && Serial.availableForWrite() && xQueueReceive(xQueueTelemetry, &telem, 5) == pdTRUE){
    //     Serial.printf("Time: %lu ms\t\tLoop dt: %lu us\t\tControl dt: %lu us\t\tRead dt: %lu us\t\tEncoder Cnt: %lli\t\tPos: %lf\t\tDutyCycle: %lf\t\tSetpoint: %lf\t\tpid flag: %i\n",
    //     telem.timestamp_millis, telem.dt, telem.control_dt, telem.read_dt, telem.pos, telem.duty_cycle, telem.set_point, telem.pid_flag);
    //   }
    // }

    if(Serial.availableForWrite() && xQueueReceive(xQueueTelemetry, &telem, 5) == pdTRUE){
      Serial.printf("Time: %lu ms\t\tLoop dt: %lu us\t\tControl dt: %lu us\t\tRead dt: %lu us\t\tEncoder Cnt: %lli\t\tPos: %lf\t\tDutyCycle: %lf\t\tSetpoint: %lf\t\tpid flag: %i\n",
      telem.timestamp_millis, telem.dt, telem.control_dt, telem.read_dt, telem.encoder_count, telem.pos, telem.duty_cycle, telem.set_point, telem.pid_flag);
    }
    vTaskDelay( 1 );
  }
}

void TaskTwiddlerinoControl(void *pvParameters){

  TwiddlerinoInit();
  
  //position is being read from other cores (maybe)
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  Position = 0;
  Setpoint = 10;
  DutyCycle = 0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetControllerDirection(REVERSE);

  xSemaphoreGive(xSemaphore);

  last_time = micros();
  uint32_t dt = 0;
  uint32_t read_dt = 0;
  uint32_t control_dt = 0;
  telemetry_t telem;

  //encoder quaderature signal counter
  //the encoder library uses cpu interrupt count peripheral to count the signal steps
  encoder_init(PCNT_UNIT_0,(gpio_num_t) 36,(gpio_num_t) 39, 250);

  encoder_clear_count();
  telem.encoder_count = encoder_get_count();

  for(;;) 
  {
    xSemaphoreTake(xSemaphore, portMAX_DELAY);

    read_dt = micros();
    telem.encoder_count = encoder_get_count();
    Position = encoder_get_angle();
    read_dt = micros() - read_dt;

    control_dt = micros();
    telem.pid_flag = myPID.Compute();
    telem.pid_flag = 0;
    SetPWMOut(DutyCycle);
    control_dt = micros() - control_dt;

    telem.duty_cycle = DutyCycle;
    telem.pos = Position;
    telem.set_point = Setpoint;

    xSemaphoreGive(xSemaphore);
    telem.timestamp_millis = millis();
    dt = micros() - last_time;
    last_time+=dt;
    telem.dt = dt;
    telem.control_dt = control_dt;
    telem.read_dt = read_dt;
    xQueueSend(xQueueTelemetry, &telem, 0);

    vTaskDelay(1);
  }
}