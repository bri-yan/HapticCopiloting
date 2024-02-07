/**
 * @file twiddlerino.h
 * @brief Twiddlerino main file
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "app/twiddlerino.h"

//twiddler pin definitions
#include "app/twid32_pin_defs.h"

//drivers
#include "drivers/encoder.h"
#include "drivers/motor.h"

//arduino
#include "Arduino.h"

#include "PID_v1.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/


/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

// Typedefs that are only used in this file, empty for now

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

//freertos tasks
void TaskReadSerial(void *pvParameters);
void TaskPublishTelemetry(void *pvParameters);
void TaskTwiddlerinoControl(void *pvParameters);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

//global variables
static uint64_t last_time;
static double Kp=0.4, Ki=0, Kd=0;
static double Setpoint, Position, DutyCycle;
PID myPID(&Position, &DutyCycle, &Setpoint, Kp, Ki, Kd, REVERSE);

static SemaphoreHandle_t xControlVariableSemaphore;
static QueueHandle_t xQueueTelemetry;

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

// Global variable definitions matching the extern definitions in the header

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void twiddlerino_setup() {
  if(!Serial) {
    Serial.begin( 115200 );//this is on rx0 tx0
    Serial.println( "Serial connected on uart0!" );
  }

  encoder_init(pcnt_unit_t::PCNT_UNIT_0, PIN_ENCODER_QUAD_A, PIN_ENCODER_QUAD_B, ENCODER_DEFAULT_FILTER);
  motor_init(PIN_MOTOR_POWER, PIN_MOTOR_DIR_0, PIN_MOTOR_DIR_1);

  last_time = micros();
  
  //mutex for position variable
  xControlVariableSemaphore = xSemaphoreCreateMutex();
  xQueueTelemetry = xQueueCreate(3, sizeof(control_telemetry_t));

  //start these tasks on core 1
  xTaskCreatePinnedToCore(
    TaskPublishTelemetry
    ,  "Controller Telemetry"
    ,  8192
    ,  NULL
    , configMAX_PRIORITIES - 1
    , NULL
    , 0
  );

  // xTaskCreatePinnedToCore(
  //   TaskReadSerial
  //   ,  "Read Serial"
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

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

char* encode_telemetry(control_telemetry_t *t){
  static char buffer[200];
  snprintf(buffer, sizeof(buffer), "time_ms:%lu,loop_dt:%lu,control_dt:%lu,read_dt:%lu,pid_success_flag:
    %i,position:%lf,pwm_duty_cycle:%lf,set_point:%f,velocity:%lf,current:%lf,torque_external:%lf\n", 
    t->timestamp_ms, t->loop_dt, t->control_dt, t->read_dt, t->pid_success_flag, t->position, 
    t->pwm_duty_cycle, t->set_point, t->velocity, t->current, t->torque_external);
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
          if(xSemaphoreTake(xControlVariableSemaphore, portMAX_DELAY) == pdTRUE){
            Setpoint = val;
            xSemaphoreGive(xControlVariableSemaphore);
          }
        } else {
          if(b=='P'){
            Kp = atof(bytes);
          } else if (b=='I'){
            Ki = atof(bytes);
          } else if (b=='D'){
            Kd = atof(bytes);
          }

          if(xSemaphoreTake(xControlVariableSemaphore, portMAX_DELAY) == pdTRUE){
            myPID.SetTunings(Kp,Ki,Kd);
            xSemaphoreGive(xControlVariableSemaphore);
          }
        }

        Serial.printf("***********************\nUpdate for %c with int value: %d\n**************************\n",b,atoi(bytes));
      }
    }
    
    vTaskDelay( 1 );
  }
}

void TaskPublishTelemetry(void *pvParameters) {
  if(!Serial) {
    Serial.begin( 115200 );
    Serial.println( "Serial connected on uart0!" );
  }

  control_telemetry_t telem;

  for(;;) 
  {
    if(Serial.availableForWrite() && xQueueReceive(xQueueTelemetry, &telem, 5) == pdTRUE){
      Serial.printf("Time: %lu ms\t\tLoop dt: %lu us\t\tControl dt: %lu us\t\tRead dt: %lu us\t\tEncoder Cnt: %lli\t\tPos: %lf\t\tDutyCycle: %lf\t\tSetpoint: %lf\t\tpid flag: %i\n",
      telem.timestamp_ms,telem.control_dt,telem.);
    }
    vTaskDelay( 1 );
  }
}

void TaskTwiddlerinoControl(void *pvParameters){
  //position is being read from other cores (maybe)
  xSemaphoreTake(xControlVariableSemaphore, portMAX_DELAY);
  Position = 0;
  Setpoint = 10;
  DutyCycle = 0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetControllerDirection(REVERSE);

  xSemaphoreGive(xControlVariableSemaphore);

  last_time = micros();
  uint32_t dt = 0;
  uint32_t read_dt = 0;
  uint32_t control_dt = 0;
  control_telemetry_t telem;

  //encoder quaderature signal counter
  //the encoder library uses cpu interrupt count peripheral to count the signal steps
  encoder_init(PCNT_UNIT_0,(gpio_num_t) 36,(gpio_num_t) 39, 250);

  encoder_clear_count();
  telem.encoder_count = encoder_get_count();

  for(;;) 
  {
    xSemaphoreTake(xControlVariableSemaphore, portMAX_DELAY);

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

    xSemaphoreGive(xControlVariableSemaphore);
    telem.timestamp_ms = millis();
    dt = micros() - last_time;
    last_time+=dt;
    telem.dt = dt;
    telem.control_dt = control_dt;
    telem.read_dt = read_dt;
    xQueueSend(xQueueTelemetry, &telem, 0);

    vTaskDelay(1);
  }
}

