/**
 * @file twid_control.c
 * @brief Control class for implementing the various controllers for the twiddlerino
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 *         Based on PID_v1 library but modified to use timer interrupts
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//header for this file
#include "app/control/twid_control.h"

//hardware drivers
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/current_sensor.h"

//control
#include "PID_v1.h"

//filters
#include "app/filter/ewma_filter.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/


/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

static void pid_callback(void *args);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

//controller information to be passed to callback function
INIT_CONTROLLER_CONFIG(controller_config);
static double setpoint_signal = 0.0;
static double meas_signal = 0.0;
static double control_signal = 0.0;
static double last_meas_signal = 0.0;
static uint32_t last_time = 0.0;
static uint32_t start_time = 0.0;
static telemetry_t telem;

// static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static EwmaFilter velocity_filt_ewa(controller_config.velocity_filter_const, 0.0);

//pid timer handle
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
esp_timer_handle_t pid_timer;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void tcontrol_configure(controller_config_t* config) {
    if(pid_timer != NULL){
        tcontrol_stop();
        ESP_ERROR_CHECK(esp_timer_delete(pid_timer));
    }

    controller_config = *config;
    setpoint_signal = 0.0;
    meas_signal = 0.0;
    control_signal = 0.0;
    last_meas_signal = 0.0;
    last_time = 0.0;
    start_time = 0.0;

    const esp_timer_create_args_t timer_args = {
        .callback = pid_callback,
        .arg = NULL,
        .name = "pid_loop"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pid_timer));

    if(Serial.availableForWrite()){
        Serial.printf("Controller config complete\n");
    }
}

void tcontrol_start(){
    if(!esp_timer_is_active(pid_timer)) {
        last_time = micros();
        start_time = micros();
        ESP_ERROR_CHECK(esp_timer_start_periodic(pid_timer, controller_config.sample_time_us));
        if(Serial.availableForWrite()){
            Serial.printf("Controller timer started\n");
        }
    }
}

void tcontrol_stop(){
    ESP_ERROR_CHECK(esp_timer_stop(pid_timer));
    if(Serial.availableForWrite()){
        Serial.printf("Controller timer stopped\n");
    }
}

void tcontrol_reset(){
    INIT_CONTROLLER_CONFIG(def);
    controller_config = def;
    tcontrol_configure(&def);
    tcontrol_start();
}

bool tcontrol_is_running(){
    return esp_timer_is_active(pid_timer);
}

// void tcontrol_get_telem(telemetry_t* telem){
//     //copy
//     *telem = *controller_config.telemetry;
// }

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void pid_callback(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    //sensor read segment 
    telem.loop_dt = micros() - last_time;
    telem.read_dt = micros();
    telem.position = encoder_get_angle();
    telem.set_point = setpoint_signal;
    meas_signal = telem.position;

    telem.current = -1;//current_sensor_read();
    telem.velocity = encoder_get_velocity();

    //calculate torque
    telem.torque_external = -1;
    //TODO

    telem.filtered_velocity = velocity_filt_ewa(telem.velocity);
    telem.current_sps = current_sensor_sps();

    telem.read_dt = micros() - telem.read_dt;

    //control segment
    telem.control_dt = micros();
    telem.pid_success_flag = 1;

    //pid calculation
    //TODO
    
    telem.pwm_duty_cycle = motor_set_pwm(control_signal);
    telem.pwm_frequency = motor_get_frequency();
    telem.control_dt = micros() - telem.control_dt;

    //fill and return telemetry structure
    telem.set_point = setpoint_signal;
    telem.timestamp_ms = (micros() - start_time)/1000.0;
    last_time += telem.loop_dt;

    xQueueSendFromISR(*controller_config.telem_queue_handle, &telem, &xHigherPriorityTaskWoken);
}