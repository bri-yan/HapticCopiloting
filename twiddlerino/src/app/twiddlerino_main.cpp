/**
 * @file twiddlerino.h
 * @brief Twiddlerino main file
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//header
#include "app/twiddlerino_main.h"

//twiddler pin definitions
#include "app/twid32_pin_defs.h"

//default configuration
#include "app/twid32_config.h"

//hardware drivers
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/current_sensor.h"

//telemetry and control
#include "comms.h"
#include "app/control/twid_control.h"

//virtual environment interface
#include "app/virtual_environment/virtual_environment.h"

//arduino
#include "Arduino.h"


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
void TaskPublishTelemetry(void *pvParameters);
void TaskRunTControl(void *pvParameters);
void TaskReadTCommands(void *pvParameters);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

static QueueHandle_t xQueueTelemetry;
static TaskHandle_t xTelemTask;
static TaskHandle_t xCommandTask;
static TaskHandle_t xTControlTask;

static bool enable_debug_telemetry = ENABLE_DEBUG_TELEMETRY_ON_INIT;

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

  //after this , if we want to idle dont run any tasks
  if (startup_type == startup_type_t::IDLE) {
    Serial.println("Entering idle mode");
    return;
  }

  //hardware setup
  Serial.printf("Hardware setup started.\n");
  current_sensor_init();
  encoder_init(pcnt_unit_t::PCNT_UNIT_0, PIN_ENCODER_QUAD_A, PIN_ENCODER_QUAD_B, ENCODER_DEFAULT_FILTER);
  motor_init(PIN_MOTOR_POWER, PIN_MOTOR_DIR_0, PIN_MOTOR_DIR_1);
  encoder_clear_count();
  motor_set_state(motor_state_t::MOTOR_LOW);

  //start communication tasks on core 0
  xTaskCreatePinnedToCore(
    TaskReadTCommands
    ,  "TReadCommands"
    ,  8192
    ,  NULL
    , TASK_PRIORITY_COMMAND
    , &xCommandTask
    , CORE_SERIAL_READ_TASK
  );
  Serial.printf("Parameter Update Task Initialized. Waiting for param update commands. Task Status: %i\n",eTaskGetState(xCommandTask));

  //initialize game serial interface
  game_interface_init();

  //start telemetry on core 0
  xTaskCreatePinnedToCore(
    TaskPublishTelemetry
    ,  "Controller Publish Telemetry"
    ,  8192
    ,  NULL
    , TASK_PRIORITY_TELEMETRY
    , &xTelemTask
    , CORE_SERIAL_WRITE_TASK
  );
  Serial.printf("Telemetry Task Initialized. Task Status: %i\n",eTaskGetState(xTelemTask));
  
  if (startup_type == startup_type_t::RUN_CONTROLLER_DEFAULT) {
    //start task
    xTaskCreatePinnedToCore(
      TaskRunTControl
      ,  "Twiddlerino T Control"
      ,  8192
      ,  NULL
      ,  TASK_PRIORITY_CONTROL
      ,  &xTControlTask
      ,  CORE_CONTROL_TASK
    );
  }
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

void TaskPublishTelemetry(void *pvParameters) {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );
    Serial.println( "Serial connected on uart0!" );
  }

  telemetry_t t;

  for(;;) 
  {
    if(enable_debug_telemetry && Serial.availableForWrite() && (xQueueReceive(xQueueTelemetry, &t, 20) == pdTRUE)){
      // publish_telemetry(&t);
      publish_telemetry_serial_studio(&t);
    }
    vTaskDelay( 1 );
  }
}

void TaskRunTControl(void *pvParameters){
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );//this is on rx0 tx0
    Serial.println( "Serial connected on uart0!" );
  }
  INIT_CONTROLLER_CONFIG(control_config);
  control_config.telem_queue_handle = &xQueueTelemetry;
  Serial.printf("T Control Task Initialized.\n");

  //initial hardware state
  encoder_clear_count();
  motor_set_state(motor_state_t::MOTOR_LOW);
  tcontrol_cfg(&control_config);
  tcontrol_start();
  Serial.printf("Controller runnning: %i\n",tcontrol_is_running());

  for(;;)
  {
    vTaskDelay(1);
  }
}

void TaskReadTCommands(void *pvParameters) {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );//this is on rx0 tx0
    Serial.println( "Serial connected on uart0!" );
  }

  String read_string;
  Serial.printf("Update Control Params Task Started\n");

  for(;;) 
  {
    //check if any new bytes received in uart buffer
    if( Serial.available()) {
      read_string = read_string_until('\n');
      //for logging purposes
      Serial.printf("Received data: %s\n", read_string);

      if(read_string.substring(0,4) == "game") {
        game_process_command(&read_string);
      } else if(read_string == "STOP" || read_string == "stop"){
        tcontrol_stop();

      } else if(read_string == "RESET" || read_string == "reset"){
        tcontrol_reset();

      } else if(read_string == "telemetry_enable") {
        enable_debug_telemetry = true;
      } else if(read_string == "telemetry_disable") {
        enable_debug_telemetry = false;
      } else if(read_string.substring(0,12) == "set_setpoint"){
        double vals[4] = {0.0, 0.0, 0.0, 0.0};
        extract_doubles(&read_string, vals, 4);
  
        setpoint_t sp;
        sp.pos = vals[0];
        sp.vel = vals[1];
        sp.accel = vals[2];
        sp.torque = vals[3];

        tcontrol_update_setpoint(&sp);
        Serial.printf("Setpoint updated.\n");

      } else if(read_string.substring(0,13) == "set_dutycycle") {
          int16_t i0 = read_string.indexOf(',',0);
          i0+=1;
          int16_t i = read_string.indexOf(',',i0);
          int32_t dc = read_string.substring(i0,i).toInt();
          motor_set_pwm(dc);
          Serial.printf("Set pwm duty cycle to %li with frequency %lu.\nMotor State: %i.\n",
          motor_get_duty_cycle(), motor_get_frequency(), motor_get_state());

      } else {
        controller_config_t cfg;
        tcontrol_get_cfg(&cfg);

        if(decode_config_cmd(&read_string, &cfg)){
          tcontrol_update_cfg(&cfg);
          Serial.printf("Controller config updated.\n");
        }

        print_controller_cfg();
      }
    }

    vTaskDelay( 10 );
  }
}