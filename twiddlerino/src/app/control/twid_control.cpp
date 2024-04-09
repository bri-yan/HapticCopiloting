/**
 * @file twid_control.c
 * @brief Control class for implementing the various controllers for the twiddlerino
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//header for this file
#include "app/control/twid_control.h"

//config
#include "app/twid32_config.h"

//hardware drivers
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/current_sensor.h"

//filters
#include "app/filter/ewma_filter.h"

//calibration
#include "app/calibration/motor_lut.h"

//comms
#include "app/comms.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define DEGS_TO_RPM 0.1666667
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/


/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

static void pid_callback(void *arg);

static void debug_print_cfg(twid_controller_t*);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/
//logtag
static const char* TAG = "twid_control";

static twid_controller_t controller_1;
static twid_controller_t controller_2;
twid_controller_t *controller2_handle = &controller_1;
twid_controller_t *controller1_handle = &controller_2;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void tcontrol_init(twid_controller_t* ctrl_handle, controller_config_t* config) {
    // if(tcontrol_is_running(ctrl_handle)){
    //     tcontrol_stop(ctrl_handle);
    // }
    motor_set_state(ctrl_handle->motor_handle, motor_state_t::MOTOR_LOW);
    encoder_clear_count(ctrl_handle->encoder_handle);

    ctrl_handle->mutex = xSemaphoreCreateMutex();
    ctrl_handle->config = *config;
    ctrl_handle->setpoint = {.pos = 0.0, .vel =0.0, .accel = 0.0};
    ctrl_handle->last_time = 0;
    ctrl_handle->start_time = 0;
    ctrl_handle->telem_dt = 0;
    ctrl_handle->last_pos = 0;
    ctrl_handle->itr = 0;
    ctrl_handle->velocity_filt_ewa = EwmaFilter(ctrl_handle->config.velocity_filter_const, 0.0);
    ctrl_handle->current_filt_ewa = EwmaFilter(ctrl_handle->config.current_filter_const, 0.0);
    ctrl_handle->pid_controller = DiscretePID(&ctrl_handle->config);
    ctrl_handle->telem.current_sps = current_sensor_sps();
    ctrl_handle->telem.nframes_sent_queue = 0;

    const esp_timer_create_args_t timer_args = {
        .callback = pid_callback,
        .arg = ctrl_handle,
        .name = "pid_loop"
    };

    if(ctrl_handle->pid_timer == NULL){
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &ctrl_handle->pid_timer));
    }

    ESP_LOGI(TAG, "controller config complete");
    debug_print_cfg(ctrl_handle);
}

void tcontrol_start(twid_controller_t* ctrl_handle){
    _ENTER_CRITICAL();
    ESP_LOGD(TAG, "%s starting", ctrl_handle->ctrl_id);
    if(!esp_timer_is_active(ctrl_handle->pid_timer)) {
        ctrl_handle->last_time = micros();
        ctrl_handle->start_time = micros();
        ESP_ERROR_CHECK(esp_timer_start_periodic(ctrl_handle->pid_timer, ctrl_handle->config.sample_time_us));
        ESP_LOGD(TAG, "Started pid callback for controller %s with timer with period %lu", ctrl_handle->ctrl_id, ctrl_handle->config.sample_time_us);
    }
    _EXIT_CRITICAL();
}

void tcontrol_stop(twid_controller_t* ctrl_handle){
    _ENTER_CRITICAL();
    if(esp_timer_is_active(ctrl_handle->pid_timer)) {
        ESP_ERROR_CHECK(esp_timer_stop(ctrl_handle->pid_timer));
        ESP_LOGW(TAG, "stopped control loop with id: %s", ctrl_handle->ctrl_id);
    }
    _EXIT_CRITICAL();
}

void tcontrol_reset(twid_controller_t* ctrl_handle){
    _ENTER_CRITICAL();
    ESP_LOGD(TAG, "tcontrol_reset called for id: %s", ctrl_handle->ctrl_id);
    tcontrol_stop(ctrl_handle);
    INIT_CONTROLLER_CONFIG_PARTIAL(def);
    ctrl_handle->config = def;
    tcontrol_init(ctrl_handle, &ctrl_handle->config);
    tcontrol_start(ctrl_handle);
    _EXIT_CRITICAL();
}

bool tcontrol_is_running(twid_controller_t* ctrl_handle){
    return esp_timer_is_active(ctrl_handle->pid_timer);
}

void tcontrol_update_setpoint(twid_controller_t* ctrl_handle, setpoint_t* tp) {
    _ENTER_CRITICAL();
    ctrl_handle->setpoint = *tp;
    ESP_LOGD(TAG, "Updated control setpoint to %lf, %lf, %lf, %lf",
    ctrl_handle->setpoint.pos, ctrl_handle->setpoint.vel, ctrl_handle->setpoint.accel, ctrl_handle->setpoint.torque);
    debug_print_cfg(ctrl_handle);
    _EXIT_CRITICAL();
}

void tcontrol_update_cfg(twid_controller_t* ctrl_handle, controller_config_t* cfg) {
    _ENTER_CRITICAL();
    ctrl_handle->config.Kp = cfg->Kp;
    ctrl_handle->config.Ki = cfg->Ki;
    ctrl_handle->config.Kd = cfg->Kd;
    ctrl_handle->config.impedance.K = cfg->impedance.K;
    ctrl_handle->config.impedance.J = cfg->impedance.J;
    ctrl_handle->config.impedance.B = cfg->impedance.B;
    ctrl_handle->config.control_type = cfg->control_type;
    ctrl_handle->config.telemetry_sample_rate = cfg->telemetry_sample_rate;
    ctrl_handle->pid_controller.set_all_params(&ctrl_handle->config);
    debug_print_cfg(ctrl_handle);
    _EXIT_CRITICAL();
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void pid_callback(void *args)
{
    twid_controller_t* ctrl = (twid_controller_t*) args;

    ESP_LOGV(TAG, "entered pid callback for controller id: %s", ctrl->ctrl_id);

    ctrl->telem.ctrl_id = ctrl->ctrl_id;

    //sensor read segment 
    ctrl->telem.timestamp_ms = (micros() - ctrl->start_time)/1000.0;
    ctrl->telem.loop_dt = micros() - ctrl->last_time;
    ctrl->last_time += ctrl->telem.loop_dt;

    ctrl->telem.read_dt = micros();
    ctrl->telem.position = encoder_get_angle(ctrl->encoder_handle);

    //read and filter current
    ctrl->telem.current_sens_adc_volts = current_sensor_get_volts(ctrl->current_sens_chan);
    ctrl->telem.current = current_sensor_get_current(ctrl->current_sens_chan);
    ctrl->telem.filtered_current = ctrl->current_filt_ewa(ctrl->telem.current);

    //read and filter velocity
    //velocity is taken as the rate of change of the positional readings in 1 sample time
    ctrl->telem.velocity = ((ctrl->telem.position - ctrl->last_pos)/(ctrl->config.sample_time_us*1e-6)) * DEGS_TO_RPM;
    ctrl->last_pos = ctrl->telem.position;

    motor_safety_check(ctrl->motor_handle, ctrl->telem.filtered_velocity);

    ctrl->telem.filtered_velocity = ctrl->velocity_filt_ewa(ctrl->telem.velocity);

    //calculate torque
    ctrl->telem.torque_net = ctrl->config.motor_Ke * ctrl->telem.filtered_current;
    ctrl->telem.torque_control = ctrl->config.motor_Ke * lookup_exp_current((double)motor_get_duty_cycle(ctrl->motor_handle)); //controller_config.motor_Kv * telem.filtered_velocity;
    if (motor_get_state(ctrl->motor_handle) == motor_state_t::MOTOR_DRIVE_CCW) {
        ctrl->telem.torque_control *= -1;
    }
    ctrl->telem.torque_external = ctrl->telem.torque_net - ctrl->telem.torque_control;

    ctrl->telem.read_dt = micros() - ctrl->telem.read_dt;

    //control segment
    ctrl->telem.control_dt = micros();
    ctrl->telem.pid_success_flag = 1;
    auto feedback_signal = 0.0;
    auto output_signal = 0.0;
    auto setpoint_signal = 0.0;

    //ratio between real (empircal) and desired inertia
    auto jjv = ctrl->config.motor_J / ctrl->config.impedance.J;

    //controller mode, select signal
    switch(ctrl->config.control_type) {
        case control_type_t::POSITION_CTRL:
            feedback_signal = ctrl->telem.position;
            setpoint_signal = ctrl->setpoint.pos;
            break;
        case control_type_t::VELOCITY_CTRL:
            feedback_signal = ctrl->telem.filtered_velocity;
            setpoint_signal = ctrl->setpoint.vel;
            break;
        case control_type_t::TORQUE_CTRL:
            //we are controlling the net torque through current control
            feedback_signal = ctrl->telem.filtered_current;
            setpoint_signal = ctrl->setpoint.torque / ctrl->config.motor_Ke;;
            break;
        case control_type_t::IMPEDANCE_CTRL_SPRING:
            //apply spring impedance law
            setpoint_signal = 
            ((ctrl->config.motor_Kv * ctrl->telem.filtered_velocity) 
            + ((ctrl->config.impedance.K) * (ctrl->setpoint.pos - ctrl->telem.position))) 
            / ctrl->config.motor_Ke;

            //feeback signal is current
            feedback_signal = ctrl->telem.current;
            break;
        case control_type_t::IMPEDANCE_CTRL_DAMPING:
            //apply damping impedance law
            setpoint_signal = 
            ((ctrl->config.motor_Kv * ctrl->telem.filtered_velocity) 
            + (ctrl->config.impedance.B) * (ctrl->setpoint.vel - ctrl->telem.filtered_velocity)) 
            / ctrl->config.motor_Ke;
            feedback_signal = ctrl->telem.current;
            break;
        case control_type_t::IMPEDANCE_CTRL_SPRING_DAMPING:
            //apply impedance law, the setpoint signal is a torque
            setpoint_signal = (ctrl->config.motor_Kv * ctrl->telem.filtered_velocity) + 
                ((ctrl->config.impedance.K) * (ctrl->setpoint.pos - ctrl->telem.position)) +
                ((ctrl->config.impedance.B) * (ctrl->setpoint.vel - ctrl->telem.filtered_velocity));
            setpoint_signal/=ctrl->config.motor_Ke;
            feedback_signal = ctrl->telem.current;
            break;
        case control_type_t::IMPEDANCE_CTRL_IGNORE_T_EXT:
            //apply impedance law, the setpoint signal is a torque
            setpoint_signal = (ctrl->config.motor_J * ctrl->setpoint.accel) + 
                (ctrl->config.motor_Kv * ctrl->telem.filtered_velocity) + 
                ((jjv * ctrl->config.impedance.K) * (ctrl->setpoint.pos - ctrl->telem.position)) +
                ((jjv * ctrl->config.impedance.B) * (ctrl->setpoint.vel - ctrl->telem.filtered_velocity));
            //convert torque to current for current control
            setpoint_signal/=ctrl->config.motor_Ke;
            feedback_signal = ctrl->telem.current;
            break;
        case control_type_t::IMPEDANCE_CTRL:
            //apply impedance law, the setpoint signal is a torque
            setpoint_signal = (ctrl->config.motor_J * ctrl->setpoint.accel) + 
                (ctrl->telem.torque_external * (1 - jjv)) + 
                (ctrl->config.motor_Kv * ctrl->telem.filtered_velocity) + 
                ((jjv * ctrl->config.impedance.K) * (ctrl->setpoint.pos - ctrl->telem.position)) +
                ((jjv * ctrl->config.impedance.B) * (ctrl->setpoint.vel - ctrl->telem.filtered_velocity));
            setpoint_signal/= ctrl->config.motor_Ke;
            feedback_signal = ctrl->telem.current;
            break;
        case control_type_t::ADMITTANCE_CTRL:
            feedback_signal = ctrl->telem.torque_external/ctrl->config.impedance.K + ctrl->setpoint.pos;
            feedback_signal = ctrl->telem.current;
            break;
        default:
            feedback_signal = 0;
            setpoint_signal = 0;
            break;
    }

    output_signal = ctrl->pid_controller.compute(feedback_signal, setpoint_signal);
    ctrl->telem.error = ctrl->pid_controller.get_error();

    //apply control signal
    if(ctrl->config.control_type != control_type_t::NO_CTRL) {
        motor_set_pwm(ctrl->motor_handle, output_signal);
    }
    ctrl->telem.pwm_duty_cycle = (double)motor_get_duty_cycle(ctrl->motor_handle);

    ctrl->telem.pwm_frequency = motor_get_frequency(ctrl->motor_handle);
    ctrl->telem.control_dt = micros() - ctrl->telem.control_dt;

    //fill and return telemetry structure
    ctrl->telem.telemetry_dt = ctrl->telem_dt; //get last telemetry delta time
    ctrl->telem_dt = micros();
    ctrl->telem.setpoint = ctrl->setpoint;
    ctrl->telem.Kp = ctrl->config.Kp;
    ctrl->telem.Ki = ctrl->config.Ki;
    ctrl->telem.Kd = ctrl->config.Kd;
    ctrl->telem.impedance = ctrl->config.impedance;
    ctrl->telem.control_type = ctrl->config.control_type;

    if((ctrl->itr % ctrl->config.telemetry_sample_rate) == 0 && ctrl->telem_queue_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xHigherPriorityTaskWoken = pdFALSE;
        ctrl->telem.nframes_sent_queue+=1;
        xQueueSendFromISR(ctrl->telem_queue_handle, &ctrl->telem, &xHigherPriorityTaskWoken);
    }
    ctrl->itr++;
    ctrl->telem_dt = micros() - ctrl->telem_dt;
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

void DiscretePID::reinit() {
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

static void debug_print_cfg(twid_controller_t* ctrl_handle) {
    ESP_LOGD(TAG, "%s CONTROLLER CONFIGURATION:\n*control_type:%i\t*sample_time:%lu us\n*Kp:%lf\t*Ki:%lf\t*Kd:%lf\n*stiffness:%lf\t*damping:%lf\t*intertia:%lf",
        ctrl_handle->ctrl_id, ctrl_handle->config.control_type, 
        ctrl_handle->config.sample_time_us, ctrl_handle->config.Kp, 
        ctrl_handle->config.Ki, ctrl_handle->config.Kd, ctrl_handle->config.impedance.K, 
        ctrl_handle->config.impedance.B, ctrl_handle->config.impedance.J);
}