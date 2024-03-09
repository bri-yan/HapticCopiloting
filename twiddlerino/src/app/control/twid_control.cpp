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
static double output_signal = 0.0;
static double last_meas_signal = 0.0;

static double integral_sum = 0.0;
static double derivative_prev = 0.0;
static double Kp = 0.0, Ki = 0.0, Kd = 0.0;
static uint32_t last_time = 0.0;
static uint32_t start_time = 0.0;
static telemetry_t telem;

static EwmaFilter velocity_filt_ewa(controller_config.velocity_filter_const, 0.0);
static EwmaFilter current_filt_ewa(controller_config.current_filter_const, 0.0);

//pid timer handle
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
esp_timer_handle_t pid_timer;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void tcontrol_configure(controller_config_t* config) {
    if(tcontrol_is_running()){
        tcontrol_stop();
    }

    controller_config = *config;
    setpoint_signal = 0.0;
    meas_signal = 0.0;
    output_signal = 0.0;
    last_meas_signal = 0.0;
    last_time = 0.0;
    start_time = 0.0;
    integral_sum = 0.0;
    derivative_prev = 0.0;
    velocity_filt_ewa = EwmaFilter(controller_config.velocity_filter_const, 0.0);
    current_filt_ewa = EwmaFilter(controller_config.current_filter_const, 0.0);

    tcontrol_update_tunings(&controller_config);

    const esp_timer_create_args_t timer_args = {
        .callback = pid_callback,
        .arg = NULL,
        .name = "pid_loop"
    };

    if(pid_timer == NULL){
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pid_timer));
    }

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
        Serial.printf("Controller timer stopped.\n");
    }
}

void tcontrol_reset(){
    tcontrol_stop();
    INIT_CONTROLLER_CONFIG(def);
    def.telem_queue_handle = controller_config.telem_queue_handle;
    controller_config = def;
    tcontrol_configure(&controller_config);
    tcontrol_start();
}

bool tcontrol_is_running(){
    return esp_timer_is_active(pid_timer);
}

// void tcontrol_get_telem(telemetry_t* telem){
//     //copy
//     *telem = *controller_config.telemetry;
// }

void tcontrol_update_setpoint(double sp) {
    setpoint_signal = sp;
}

void tcontrol_update_tunings(double kp, double ki, double kd, controller_direction_t direction) { 
    int8_t dir = (direction == controller_direction_t::NEGATIVE_FEEDBACK) ? 1 : -1;
    Kp = dir * kp;
    Ki = dir * ki;
    Kd = dir * kd;
}

void tcontrol_update_tunings(controller_config_t* config) {
    int8_t dir = (config->controller_direction == controller_direction_t::NEGATIVE_FEEDBACK) ? 1 : -1;
    Kp = dir*config->Kp;
    Ki = dir*config->Ki;
    Kd = dir*config->Kd;
}

double tcontrol_get_kp() {
    return Kp;
}

double tcontrol_get_ki() {
    return Ki;
}

double tcontrol_get_kd() {
    return Kd;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void pid_callback(void *args)
{
    telem.set_point = setpoint_signal;

    //sensor read segment 

    telem.loop_dt = micros() - last_time;
    last_time += telem.loop_dt;
    telem.read_dt = micros();
    telem.position = encoder_get_angle();

    //read and filter current
    telem.current = -1;//current_sensor_read();
    telem.filtered_current = current_filt_ewa(telem.current);
    telem.current_sps = -1;//current_sensor_sps();

    //read and filter velocity
    telem.velocity = encoder_get_velocity();
    telem.filtered_velocity = velocity_filt_ewa(telem.velocity);

    //calculate torque
    telem.torque_net = controller_config.torque_Ke * telem.filtered_current;
    telem.torque_control = controller_config.torque_Kv * telem.filtered_velocity;
    telem.torque_external = telem.torque_net - telem.torque_control;

    telem.read_dt = micros() - telem.read_dt;

    //control segment
    telem.control_dt = micros();
    telem.pid_success_flag = 1;
    //controller mode, select signal
    switch(controller_config.control_type) {
        case control_type_t::POSITION_CTRL:
            meas_signal = telem.position;
            break;
        case control_type_t::VELOCITY_CTRL:
            meas_signal = telem.filtered_velocity;
            break;
        case control_type_t::TORQUE_CTRL:
            //we are controlling the net torque
            meas_signal = telem.torque_net;
            break;
        case control_type_t::IMPEDANCE_CTRL:
            //TODO apply impedance law
            meas_signal = 0;
            break;
        case control_type_t::ADMITTANCE_CTRL:
            meas_signal = 0;
            break;
        default:
            meas_signal = telem.position;
            break;
    }

    //compute control signal with discrete pid equation
    //https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
    output_signal = 0.0;
    //standard negative feedback control error
    double error = setpoint_signal - meas_signal;
    telem.error = error;
    //changed in measured signal between the last two iterations
    double delta_meas_signal = (meas_signal - last_meas_signal);
    double h = (controller_config.sample_time_us * 1.0e-6);
    //derivative term
    double derivative =  (Kd / (Kd + controller_config.N * h)) * (derivative_prev - controller_config.N * (delta_meas_signal));
    derivative_prev = derivative;
    output_signal += derivative;
    //integral term
    output_signal += integral_sum;
    //compute next integral term
    integral_sum += (Ki * h * error);
    //proportional term
    output_signal += (Kp * error);

    //apply control signal
    telem.pwm_duty_cycle = motor_set_pwm(output_signal);
    telem.pwm_frequency = motor_get_frequency();
    telem.control_dt = micros() - telem.control_dt;

    //fill and return telemetry structure
    telem.set_point = setpoint_signal;
    telem.timestamp_ms = (micros() - start_time)/1000.0;

    if(controller_config.telem_queue_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdTRUE;
        xQueueSendFromISR(*controller_config.telem_queue_handle, &telem, &xHigherPriorityTaskWoken);
    }
}