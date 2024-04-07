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
static const char* TAG = "twiddlerino_main";

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

void twiddlerino_setup() {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );//this is on rx0 tx0
    ESP_LOGI(TAG, "Serial connected on uart0! with baud rate %lu\n",UART_BAUD_RATE);
  }

  //queues for communicating with the control core
  xQueueTelemetry = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(telemetry_t));

  //hardware setup
  ESP_LOGI(TAG, "Hardware setup started");

  //setup hardware handles
  motor1_handle = {
    .power_pin = PIN_MOTOR1_POWER,
    .dir_pin = PIN_MOTOR1_DIR,
    .pwm_channel = MOTOR1_PWM_CHAN_DEFAULT,
    .state = motor_state_t::MOTOR_NOT_INITIALIZED,
    .handle_name = "motor1"
    };

  motor2_handle = {
    .power_pin = PIN_MOTOR2_POWER,
    .dir_pin = PIN_MOTOR2_DIR,
    .pwm_channel = MOTOR2_PWM_CHAN_DEFAULT,
    .state = motor_state_t::MOTOR_NOT_INITIALIZED,
    .handle_name = "motor2"
    };

  encoder1_handle = {
    .quad_pin_a = PIN_ENCODER1_QUAD_A,
    .quad_pin_b = PIN_ENCODER1_QUAD_B,
    .pcnt_unit = pcnt_unit_t::PCNT_UNIT_0,
    .filter = ENCODER_DEFAULT_FILTER};

  encoder2_handle = {
    .quad_pin_a = PIN_ENCODER2_QUAD_A,
    .quad_pin_b = PIN_ENCODER2_QUAD_B,
    .pcnt_unit = pcnt_unit_t::PCNT_UNIT_1,
    .filter = ENCODER_DEFAULT_FILTER};

  //error checks handled inside init functions
  current_sensor_init();

  encoder_init(&encoder1_handle);
  encoder_init(&encoder2_handle);

  motor_init(&motor1_handle);
  motor_init(&motor2_handle);

  motor_set_state(&motor1_handle, motor_state_t::MOTOR_LOW);
  motor_set_state(&motor2_handle, motor_state_t::MOTOR_LOW);

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
  ESP_LOGI(TAG, "Command Task Initialized. Task Status: %i\n",eTaskGetState(xCommandTask));

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
  ESP_LOGI(TAG, "Telemetry Task Initialized. Task Status: %i\n",eTaskGetState(xTelemTask));
  
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
  ESP_LOGI(TAG, "Control Task Initialized. Task Status: %i\n",eTaskGetState(xTControlTask));
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

void TaskPublishTelemetry(void *pvParameters) {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );
    ESP_LOGI(TAG, "Serial connected on uart0!" );
  }

  telemetry_t t;
  uint64_t last_time = millis();

  for(;;) 
  {
    if(enable_debug_telemetry && Serial.availableForWrite() && (xQueueReceive(xQueueTelemetry, &t, 20) == pdTRUE)){
      publish_telemetry_serial_studio(&t);
    }

    if (millis() > last_time + FREE_RTOS_TELEM_SAMPLE_RATE) {
      ESP_LOGI(TAG, "Number of freertos tasks running: %i", uxTaskGetNumberOfTasks());
      TaskHandle_t handle_arr[3] = {xCommandTask, xTelemTask, xTControlTask};
      for(int i=0;i<3;i++){
        auto name = pcTaskGetName(handle_arr[i]);
        auto mem = uxTaskGetStackHighWaterMark(handle_arr[i]);
        auto state = eTaskStateGet(handle_arr[i]);
        ESP_LOGI(TAG, "Task Status:\n\t*name:%s\tStatus:%i\tFree Stack Space:%u",
        name, state, mem);
      }

      last_time = millis();
    }

    vTaskDelay( 1 );
  }
}

void TaskRunTControl(void *pvParameters){
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );//this is on rx0 tx0
    ESP_LOGI(TAG, "Serial connected on uart0!" );
  }

  ESP_LOGI(TAG, "T Control Task Initialized.\n");

  //there's probably a better way to write the following section
  // and name the variables
  //this is prone to typos :)
  INIT_CONTROLLER_CONFIG_PARTIAL(default_config);
  controller1_handle->config = default_config;
  controller1_handle->ctrl_id = TWID1_ID;
  controller1_handle->telem_queue_handle = xQueueTelemetry;
  controller1_handle->motor_handle = &motor1_handle;
  controller1_handle->encoder_handle = &encoder1_handle;
  controller1_handle->current_sens_chan = curr_sens_adc_channel_t::CURRENT_SENSOR_1;
  tcontrol_init(controller1_handle, &(controller1_handle->config));
  tcontrol_start(controller1_handle);
  ESP_LOGI(TAG, "Controller %s loop status: %i\n", controller1_handle->ctrl_id,  tcontrol_is_running(controller1_handle));

  if (ENABLE_SECOND_CONTROLLER) {
    controller2_handle->config = default_config;
    controller2_handle->ctrl_id = TWID2_ID;
    controller2_handle->telem_queue_handle = xQueueTelemetry;
    controller2_handle->motor_handle = &motor2_handle;
    controller2_handle->encoder_handle = &encoder2_handle;
    controller2_handle->current_sens_chan = curr_sens_adc_channel_t::CURRENT_SENSOR_2;

    tcontrol_init(controller2_handle, &(controller2_handle->config));
    tcontrol_start(controller2_handle);

    ESP_LOGI(TAG, "Controller %s loop status: %i\n", controller2_handle->ctrl_id,  tcontrol_is_running(controller2_handle));
  }

  for(;;)
  {
    vTaskDelay(10);
  }
}

void TaskReadTCommands(void *pvParameters) {
  if(!Serial) {
    Serial.begin( UART_BAUD_RATE );//this is on rx0 tx0
    ESP_LOGI(TAG, "Serial connected on uart0!" );
  }

  String read_string;
  ESP_LOGI(TAG, "Update Control Params Task Started\n");
  cmd_type_t last_cmd = NA_CMD;

  for(;;) 
  {
    if (!Serial) {
      Serial.end();
      Serial.begin( UART_BAUD_RATE );
    }
    //check if any new bytes received in uart buffer
    if( Serial.available()) {
      read_string = read_string_until('\n');
      last_cmd = cmd_type_t::NA_CMD;
      ESP_LOGD(TAG, "Received data: %s\n", read_string);
      last_cmd = handle_command(&read_string);
      ack_cmd(last_cmd);
    }

    vTaskDelay( 10 );
  }
}

void enable_telemetry_publisher() {
  enable_debug_telemetry = true;
}

void disable_telemetry_publisher() {
    enable_debug_telemetry = false;
}
