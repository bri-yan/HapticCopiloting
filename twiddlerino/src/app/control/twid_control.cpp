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

#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

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

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

//controller information to be passed to callback function
INIT_CONTROLLER_CONFIG(controller_config);
static setpoint_t trajectory_point = {.pos = 0.0, .vel =0.0, .accel = 0.0, .torque = 0.0};
static uint32_t last_time = 0.0;
static uint32_t start_time = 0.0;
static telemetry_t telem;
static EwmaFilter velocity_filt_ewa(controller_config.velocity_filter_const, 0.0);
static EwmaFilter current_filt_ewa(controller_config.current_filter_const, 0.0);
static DiscretePID pid_controller(&controller_config);

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
    trajectory_point = {.pos = 0.0, .vel =0.0, .accel = 0.0};
    last_time = 0.0;
    start_time = 0.0;
    velocity_filt_ewa = EwmaFilter(controller_config.velocity_filter_const, 0.0);
    current_filt_ewa = EwmaFilter(controller_config.current_filter_const, 0.0);
    pid_controller = DiscretePID(&controller_config);

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

void tcontrol_update_trajectory(setpoint_t* tp) {
    _ENTER_CRITICAL();
    trajectory_point = *tp;
    _EXIT_CRITICAL();
}

void tcontrol_update_tunings(double kp, double ki, double kd, controller_direction_t direction) {
    _ENTER_CRITICAL(); 
    pid_controller.set_gains(kp, ki, kd);
    pid_controller.set_direction(direction);
    _EXIT_CRITICAL();
}

void tcontrol_update_tunings(controller_config_t* config) {
    _ENTER_CRITICAL(); 
    pid_controller.set_all_params(config);
    _EXIT_CRITICAL();
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void pid_callback(void *args)
{
    telem.target = trajectory_point;

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
    auto feedback_signal = 0.0;
    auto output_signal = 0.0;
    auto setpoint_signal = 0.0;

    //controller mode, select signal
    switch(controller_config.control_type) {
        case control_type_t::POSITION_CTRL:
            feedback_signal = telem.position;
            setpoint_signal = trajectory_point.pos;
            break;
        case control_type_t::VELOCITY_CTRL:
            feedback_signal = telem.filtered_velocity;
            setpoint_signal = trajectory_point.vel;
            break;
        case control_type_t::TORQUE_CTRL:
            //we are controlling the net torque through current control
            feedback_signal = telem.filtered_current;
            setpoint_signal = trajectory_point.torque / controller_config.torque_Ke;;
            break;
        case control_type_t::IMPEDANCE_CTRL:
            //TODO apply impedance law
            setpoint_signal = 0;//torque
            feedback_signal = 0;
            break;
        case control_type_t::ADMITTANCE_CTRL:
            feedback_signal = 0;
            setpoint_signal = 0;
            break;
        default:
            feedback_signal = telem.position;
            setpoint_signal = trajectory_point.pos;
            break;
    }

    output_signal = pid_controller.compute(feedback_signal, setpoint_signal);
    telem.error = pid_controller.get_error();

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


/******************************************************************************/
/*                                 C L A S S                                  */
/******************************************************************************/


DiscretePID::DiscretePID(double kp, double ki, double kd, double h) {
    this->N = 1.0;
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
    this->h = h;
    reinit();
}

DiscretePID::DiscretePID::DiscretePID(double kp, double ki, double kd, double h, double N) {
    this->N = N;
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
    this->h = h;
    reinit();
}

DiscretePID::DiscretePID(controller_config_t* cfg) {
    this->N = cfg->N;
    this->Kp = cfg->Kp;
    this->Ki = cfg->Ki;
    this->Kd = cfg->Kd;
    this->h = cfg->sample_time_us * 1.0e-6;
    this->OUTPUT_MAX = cfg->output_hlim;
    this->OUTPUT_MIN = cfg->output_llim;
    reinit();
}

void DiscretePID::reinit(){
    last_measured = 0.0;
    last_setpoint = 0.0;
    last_output = 0.0;
    last_derivative = 0.0;
    integral_sum = 0.0;
}

void DiscretePID::set_all_params(controller_config_t* cfg) {
    this->direction = cfg->controller_direction;
    this->N = cfg->N;
    this->Kp = cfg->Kp;
    this->Ki = cfg->Ki;
    this->Kd = cfg->Kd;
    this->h = cfg->sample_time_us * 1.0e-6;
    this->OUTPUT_MAX = cfg->output_hlim;
    this->OUTPUT_MIN = cfg->output_llim;
}

void DiscretePID::set_gains(double kp, double ki, double kd) {
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
}

void DiscretePID::set_direction(controller_direction_t dir) {
    this->direction = dir;
}

void DiscretePID::set_limits(int32_t high, int32_t low){
    OUTPUT_MAX = high;
    OUTPUT_MIN = low;
}

double DiscretePID::compute(double measured, double setpoint) {
    auto output = 0.0;
    auto error = (setpoint - measured) * ((direction == controller_direction_t::NEGATIVE_FEEDBACK) ? 1 : -1);
    auto delta_measured = (measured - last_measured);

    auto derivative =  (Kd / (Kd + N * h)) * (last_derivative - N * (delta_measured));
    last_derivative = derivative;
    output += derivative;
    output += (Kp * error);
    output += integral_sum;
    integral_sum += (Ki * h * error);

    //clamp
    if(output > OUTPUT_MAX) {
        output = OUTPUT_MAX;
    } else if (output < OUTPUT_MIN) {
        output = OUTPUT_MIN;
    }

    last_derivative = measured;
    last_setpoint = setpoint;
    last_output = output;
    return output;
}

double DiscretePID::get_error() {
    return last_setpoint - last_measured;
}

double DiscretePID::get_kp() {
    return Kp;
}

double DiscretePID::get_ki() {
    return Ki;
}

double DiscretePID::get_kd() {
    return Kd;
}