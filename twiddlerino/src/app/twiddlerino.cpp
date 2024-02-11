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

//telemetry
#include "comms.h"
//control
#include "PID_v1.h"

//arduino
#include "Arduino.h"


/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define TELEMETRY_QUEUE_SIZE 1000U
#define COMMAND_QUEUE_SIZE 10U
#define UART_BAUD_RATE 250000U

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

// Typedefs that are only used in this file, empty for now

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

//freertos tasks
void TaskReadCommands(void *pvParameters);
void TaskPublishTelemetry(void *pvParameters);
void TaskTwiddlerinoControl(void *pvParameters);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

static QueueHandle_t xQueueTelemetry;
static QueueHandle_t xQueueCommand;
static TaskHandle_t xTelemTask;
static TaskHandle_t xCommandTask;

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

// Global variable definitions matching the extern definitions in the header

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void twiddlerino_setup(startup_type_t startup_type) {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );//this is on rx0 tx0
    Serial.printf( "Serial connected on uart0! with baud rate %lu\n",UART_BAUD_RATE);
  }

  //queues for communicating with the control core
  xQueueTelemetry = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(telemetry_t));
  xQueueCommand = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(test_config_t));

  //after this , if we want to idle dont run any tasks
  if (startup_type == startup_type_t::IDLE) {
    Serial.println("Entering idle mode");
    return;
  }

  //start telemetry on core 0
  xTaskCreatePinnedToCore(
    TaskPublishTelemetry
    ,  "Controller Publish Telemetry"
    ,  8192
    ,  NULL
    , configMAX_PRIORITIES - 1
    , &xTelemTask
    , 0
  );
  Serial.printf("Telemetry Task Initialized. Task Status: %i\n",eTaskGetState(xTelemTask));
  

  //start the command reader task if that is our perferred startup mode
  //this task awaits commands to trigger test/controller
  //this task can trigger other tasks to run the controller
  if (startup_type == startup_type_t::RUN_AWAIT_COMMANDS) {
    xTaskCreatePinnedToCore(
      TaskReadCommands
      ,  "Read Commands"
      ,  8192
      ,  NULL
      , configMAX_PRIORITIES
      , &xCommandTask
      , 0
    );
    Serial.printf("Command Task Initialized. Task Status: %i\n",eTaskGetState(xCommandTask));
  }

  //run default controller
  if (startup_type == startup_type_t::RUN_CONTROLLER_DEFAULT) {

    //default config
    test_config_t test_config;
    test_config.Kd = 0;
    test_config.Kp = 1;
    test_config.Ki = 0;
    test_config.sample_rate_us = 1000;
    test_config.set_point = 0;
    test_config.test_duration_ms = -1;

    //start task
    TaskHandle_t xDefaultControlTask;
    xTaskCreatePinnedToCore(
      TaskTwiddlerinoControl
      ,  "Twiddlerino Control"
      ,  8192
      ,  &test_config
      ,  configMAX_PRIORITIES
      ,  &xDefaultControlTask
      ,  1
    );
    Serial.printf("Control Task Initialized.\nTask Status: %i\n",eTaskGetState(xDefaultControlTask));
  }
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

void TaskReadCommands(void *pvParameters) {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );//this is on rx0 tx0
    Serial.println( "Serial connected on uart0!" );
  }

  String read_string;
  test_config_t test_config;
  cmd_type_t cmd_type;
  TaskHandle_t task_handle = NULL;
  eTaskState task_state = eTaskState::eInvalid;
  uint32_t ctask_duration_ms = 0;
  uint32_t ctask_start_time_ms = 0;

  for(;;) 
  {
    if( Serial.available() ) {
      read_string = read_string_until('\n');
      cmd_type = decode_cmd(&read_string, &test_config);

      Serial.printf("read line \"%s\" and decoded command %i\n",read_string,cmd_type);

      // Serial.printf("Starting control task with params:\n\t\tKp:%lf\tKi:%lf\tKd:%lf\tsample_rate_us:%lu\ttest_duration_ms:%lu\n",
      //   test_config.Kp,test_config.Ki,test_config.Kd,test_config.sample_rate_us,test_config.test_duration_ms);

      if(cmd_type==cmd_type_t::CONFIG_TEST) {
        xQueueSend(xQueueCommand, &test_config, portMAX_DELAY);
        Serial.printf("ack\n");
      }

      if(cmd_type==cmd_type_t::START_TEST) {
        if(task_handle != NULL){
          task_state = eTaskGetState(task_handle);
        } else {
          task_state = eTaskState::eInvalid;
        }

        test_config_t t;

        if((task_state == eTaskState::eDeleted || task_state == eTaskState::eSuspended || task_state == eTaskState::eInvalid ) || millis() > ctask_start_time_ms + ctask_duration_ms ) {

          if (xQueueReceive(xQueueCommand, &t, 5) == pdTRUE) {
            ctask_duration_ms = t.test_duration_ms;
            //start control test task on core 1
            xTaskCreatePinnedToCore(
              TaskTwiddlerinoControl
              ,  "Twiddlerino Control"
              ,  8192
              ,  &t
              ,  configMAX_PRIORITIES
              ,  &task_handle
              ,  1
            );
            ctask_start_time_ms = millis();
            Serial.printf("ack\n");
          } else {
            printf("could not aqquire test_config from queue\n");
          }
  
        } else {
          Serial.printf("Another test is currently running, test_config waiting in queue\n");
          Serial.printf("Task State: %i",task_state);
        }
      }

      if(cmd_type==cmd_type_t::ABORT_TEST) {
        if (task_handle != NULL && eTaskGetState(task_handle) == eTaskState::eRunning) {
          vTaskDelete(task_handle);
        }
        motor_set_pwm(0);
        motor_set_state(motor_state_t::MOTOR_LOW);
        Serial.printf("ack\n");
        Serial.printf("Test Aborted\n");
        task_handle = NULL;
        task_state = eTaskState::eInvalid;
      }

      if(millis() > ctask_start_time_ms + ctask_duration_ms) {
        task_handle = NULL;
        task_state = eTaskState::eInvalid;
      }
    }
    vTaskDelay( 1 );
  }
}

void TaskPublishTelemetry(void *pvParameters) {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );
    Serial.println( "Serial connected on uart0!" );
  }

  telemetry_t t;

  for(;;) 
  {
    if(Serial.availableForWrite() && xQueueReceive(xQueueTelemetry, &t, 20) == pdTRUE){
      publish_telemetry(&t);
    }
    vTaskDelay( 1 );
  }
}

//this task is triggered whenever we want to run the basic controller
//we can pass params to the controller
void TaskTwiddlerinoControl(void *pvParameters){
  test_config_t config;
  if (pvParameters != NULL) {
    config = *((test_config_t *) pvParameters);
  } else {
    //default config
    test_config_t config;
    config.Kd = 0;
    config.Kp = 1;
    config.Ki = 0;
    config.sample_rate_us = 1000;
    config.set_point = 0;
    config.test_duration_ms = -1;
  }

  Serial.printf("Starting control task with params:\n\t\tKp:%lf\tKi:%lf\tKd:%lf\tsample_rate_us:%lu\ttest_duration_ms:%lu\n",
    config.Kp,config.Ki,config.Kd,config.sample_rate_us,config.test_duration_ms);

  //hardwaire setup
  encoder_init(pcnt_unit_t::PCNT_UNIT_0, PIN_ENCODER_QUAD_A, PIN_ENCODER_QUAD_B, ENCODER_DEFAULT_FILTER);
  motor_init(PIN_MOTOR_POWER, PIN_MOTOR_DIR_0, PIN_MOTOR_DIR_1);
  encoder_clear_count();
  motor_set_state(motor_state_t::MOTOR_LOW);

  //pid config
  double Setpoint=config.set_point, Position=0, DutyCycle=0;
  PID myPID(&Position, &DutyCycle, &Setpoint, config.Kp, config.Ki, config.Kd, REVERSE);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(config.sample_rate_us/1000);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetTunings(config.Kp, config.Ki, config.Kd);
  myPID.SetControllerDirection(REVERSE);

  //time and telemetry
  uint32_t dt = 0;
  uint32_t read_dt = 0;
  uint32_t control_dt = 0;
  uint64_t last_time = micros();
  uint64_t start_time = micros();
  telemetry_t telem;

  //initial hardware state
  motor_set_pwm(0);
  motor_set_state(motor_state_t::MOTOR_LOW);
  encoder_clear_count();

  while(micros() - start_time < config.test_duration_ms*1000.0)
  {
    if(micros() - last_time >= config.sample_rate_us) {
      dt = micros() - last_time;
      read_dt = micros();
      telem.position = encoder_get_angle();
      Position = telem.position;
      read_dt = micros() - read_dt;

      control_dt = micros();
      telem.pid_success_flag = myPID.Compute();
      telem.pid_success_flag = 0;
      motor_set_pwm(DutyCycle);
      control_dt = micros() - control_dt;

      telem.pwm_duty_cycle = DutyCycle;
      telem.set_point = Setpoint;

      telem.timestamp_ms = (micros() - start_time)/1000.0;
      last_time+=dt;
      telem.loop_dt = dt;
      telem.control_dt = control_dt;
      telem.read_dt = read_dt;

      telem.velocity = -1;
      telem.current = -1;
      telem.torque_external = -1;

      xQueueSend(xQueueTelemetry, &telem, 0);
    }

    vTaskDelay(0);
  }

  motor_set_pwm(0);
  motor_set_state(motor_state_t::MOTOR_LOW);
  encoder_clear_count();
  Serial.printf("test_complete\n");

  //delete the task
  vTaskDelete(NULL);
}

