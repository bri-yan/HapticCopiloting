/**
 * @file twid_control.h
 * @brief Control class for implementing the various controllers for the twiddlerino
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 */

#ifndef TWID_CONTROL_H_
#define TWID_CONTROL_H_

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include "app/control/control_types.h"

#include "app/filter/ewma_filter.h"

#include "app/twid32_config.h"

#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/current_sensor.h"

#include "Arduino.h"

/******************************************************************************/
/*                                 C L A S S                                  */
/******************************************************************************/

//discrete pid class to carry out calculation
//https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
//should be called only at discrete intervals h seconds
class DiscretePID {
  public:
    DiscretePID(double kp, double ki, double kd, double h);
    DiscretePID(double kp, double ki, double kd, double h, double N);
    DiscretePID(controller_config_t* cfg);
    void reinit();
    void set_all_params(controller_config_t* cfg);
    void set_gains(double kp, double ki, double kd);
    void set_direction(controller_direction_t dir);
    void set_limits(int32_t high, int32_t low);
    double compute(double measured, double setpoint);
    double get_error();
    double get_kp();
    double get_ki();
    double get_kd();

  private:
    controller_direction_t direction = controller_direction_t::NEGATIVE_FEEDBACK;
    int32_t OUTPUT_MAX = 1024;
    int32_t OUTPUT_MIN = -1024;
    double Kp = 0.0, Ki = 0.0, Kd = 0.0; //pid params
    double h; //controller discrete step time in seconds
    double N; //derivative filter constant
    double last_measured;
    double last_setpoint;
    double last_output;
    double last_derivative;
    double integral_sum;
};

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

struct twid_controller_t {
    const char* ctrl_id;

    controller_config_t config; //configuration
    SemaphoreHandle_t mutex;  //controller lock mutex to handle sharing with isr
    esp_timer_handle_t pid_timer; //pid timer handle
    QueueHandle_t telem_queue_handle; //telemetry queue handle

    //hardware handles
    motor_driver_context_t* motor_handle;
    encoder_context_t* encoder_handle;
    curr_sens_adc_channel_t current_sens_chan;

    //filters
    //default is no filtering
    EwmaFilter velocity_filt_ewa = EwmaFilter(0, 0);
    EwmaFilter current_filt_ewa = EwmaFilter(0, 0);

    //underlying discrete pid implementation
    //default value is 0 gain
    DiscretePID pid_controller = DiscretePID(0, 0, 0, 1e-3);

    //telemetry struct
    telemetry_t telem;

    //data updated per loop
    setpoint_t setpoint; //current setpoint
    uint32_t last_time; //last loop timestamp us
    uint32_t start_time; //start timestamp us

    uint32_t start_admittance_switch;
    uint32_t start_impedance_switch; 

    uint32_t start_admittance_tf;
    uint32_t start_impedance_tf;

    uint32_t telem_dt; //telemetry code delta time in us
    uint32_t itr; //loop number
    double last_pos; //last encoder position measurement
};

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//configure controller and timer isr
//this function must be run before tcontrol_start or an error will be thrown
void tcontrol_init(twid_controller_t*, controller_config_t*);

//starts control timer isr
void tcontrol_start(twid_controller_t*);

//stops control timer isr
void tcontrol_stop(twid_controller_t*);

//resets to default config and restarts control timer isr
void tcontrol_reset(twid_controller_t*);

//returns true if the pid timer isr is active
bool tcontrol_is_running(twid_controller_t*);

//copies config to destination
void tcontrol_get_cfg(twid_controller_t*, controller_config_t*);

//set a new setpoint target
void tcontrol_update_setpoint(twid_controller_t*, setpoint_t*);

//update existing configuration
//NOTE!!! this will only update gains for pid, impedance, and control mode for now
void tcontrol_update_cfg(twid_controller_t*, controller_config_t*);


extern twid_controller_t *controller1_handle;
extern twid_controller_t *controller2_handle;

#endif //TWID_CONTROL_H_