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

#include "app/twid32_config.h"

#include "Arduino.h"

/******************************************************************************/
/*                             F U N C T I O N S                              */
/******************************************************************************/

//configure controller and timer isr
//this function must be run before tcontrol_start or an error will be thrown
void tcontrol_cfg(controller_config_t*);

//starts control timer isr
void tcontrol_start();

//stops control timer isr
void tcontrol_stop();

//resets to default config and restarts control timer isr
void tcontrol_reset();

//returns true if the pid timer isr is active
bool tcontrol_is_running();

//copies config to destination
void tcontrol_get_cfg(controller_config_t*);

//set a new setpoint target
void tcontrol_update_setpoint(setpoint_t*);

//update existing configuration
//NOTE!!! this will only update gains for pid, impedance, and control mode for now
void tcontrol_update_cfg(controller_config_t*);

// get the latest controller state (telemetry struct)
// copies over telemetry to destination
void tcontrol_get_state(telemetry_t*);

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

#endif //TWID_CONTROL_H_