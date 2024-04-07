/**
 * @file comms.cpp
 * @brief Communications for command and telemetry
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

//library header
#include "app/comms.h"

//comms types for telem and command
#include "app/control/twid_control.h"

#include "Arduino.h"

#include "twiddlerino_main.h"

//hardware drivers
#include "drivers/encoder.h"
#include "drivers/motor.h"
#include "drivers/current_sensor.h"




/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define READ_TIMEOUT_US 200U

#define MAX_ELEMENTS 100U

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

// Typedefs that are only used in this file, empty for now

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

static char timed_read();

static void insert_setpoint_buffer(cmd_type_t cmd, String * str, setpoint_t* buffer, SemaphoreHandle_t* mutex, uint16_t num_values);

uint16_t extract_cs_strings(String * str, String *out);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

static const char* TAG = "comms";
static uint32_t nframes_sent_serial = 0;

//array of char arrays
static String elements[MAX_ELEMENTS];

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

cmd_type_t handle_command(String *str) {
    ESP_LOGD(TAG, "Recived cmd string %s", *str);

    auto num_elements = extract_cs_strings(str, elements);
    ESP_LOGD(TAG, "Extracted %i elements from command string", num_elements);
    twid_controller_t *ctrl;
    uint16_t i0 = 0;

    if (elements[0] == TWID1_ID) {
        ESP_LOGD(TAG, "Interpreting command for Twid 1");
        ctrl = controller1_handle;
        i0++; //index 0 is an id, data starts at index 1
    } else if (elements[0] == TWID2_ID && ENABLE_SECOND_CONTROLLER) {
        ESP_LOGD(TAG, "Interpreting command for Twid 2");
        ctrl = controller2_handle;
        i0++;
    } else {
        ESP_LOGD(TAG, "Defaulting to command interpreting for twid 1");
        //default to controller 1 handle
        ctrl = controller1_handle;
    }

    auto cfg = ctrl->config;

    if(*str == "hello" || *str == "ping"){
        Serial.printf("esp_alive_signal\n");
    } else if(*str == "STOP" || *str == "stop"){
        motor_fast_stop(ctrl->motor_handle);
        tcontrol_stop(ctrl);
        return cmd_type_t::STOP;
    } else if(*str == "RESET" || *str == "reset"){
        motor_fast_stop(ctrl->motor_handle);
        reset_sent_count();
        tcontrol_reset(ctrl);
        return cmd_type_t::RESET;
    } else if(*str == "REBOOT" || *str == "reboot"){
        auto last_cmd = cmd_type_t::REBOOT;
        //we need to send ack before the cpu resets
        ack_cmd(last_cmd);
        //reset cpu, does not return
        esp_restart();
    } else if(*str == "telemetry_enable") {
        enable_telemetry_publisher();
        return cmd_type_t::TELEM_ENABLE;
    } else if(*str == "telemetry_disable") {
        disable_telemetry_publisher();
        return cmd_type_t::TELEM_DISABLE;
    } else if(str->substring(0,7) == "set_pid" && num_elements >= 3){
        cfg.Kp = elements[i0].toDouble();
        cfg.Ki = elements[i0+1].toDouble();
        cfg.Kd = elements[i0+2].toDouble();
        tcontrol_update_cfg(ctrl, &cfg);
        return cmd_type_t::SET_PID;
    } else if(str->substring(0,13) == "set_impedance" && num_elements >= 3) {
        cfg.impedance.K = elements[i0].toDouble();
        cfg.impedance.B = elements[i0+1].toDouble();
        cfg.impedance.J = elements[i0+2].toDouble();
        tcontrol_update_cfg(ctrl, &cfg);
        return cmd_type_t::SET_IMPEDANCE;
    } else if(str->substring(0,8) == "set_mode" && num_elements >= 1) {
        auto name = elements[i0];

        control_type_t mode = cfg.control_type;
        if(name == "position"){
            mode = control_type_t::POSITION_CTRL;
        } else if(name=="velocity") {
            mode = control_type_t::VELOCITY_CTRL;
        } else if(name=="torque") {
            mode = control_type_t::TORQUE_CTRL;
        } else if(name=="impedance") {
            mode = control_type_t::IMPEDANCE_CTRL;
        } else if(name=="admittance") {
            mode = control_type_t::ADMITTANCE_CTRL;
        } else if(name=="no_control") {
            mode = control_type_t::NO_CTRL;
        } else if(name=="impedance_spring") {
            mode = control_type_t::IMPEDANCE_CTRL_SPRING;
        } else if(name=="impedance_damping" || name=="impedance_damper") {
            mode = control_type_t::IMPEDANCE_CTRL_DAMPING;
        } else if(name=="impedance_ignore_t_ext") {
            mode = control_type_t::IMPEDANCE_CTRL_IGNORE_T_EXT;
        } else if(name=="impedance_spring_damping") {
            mode = control_type_t::IMPEDANCE_CTRL_SPRING_DAMPING;
        } else {
            return cmd_type_t::NA_CMD;
        }

        cfg.control_type = mode;
        tcontrol_update_cfg(ctrl, &cfg);
        return cmd_type_t::SET_MODE;
    } else if(str->substring(0,19) == "set_telemsamplerate" && num_elements >= 1) {
        ESP_LOGD(TAG, "%s", *str);
        auto val = elements[i0].toInt();
        if (val >= 1) {
            cfg.telemetry_sample_rate = (uint32_t)val;
            tcontrol_update_cfg(ctrl, &cfg);
            return cmd_type_t::SET_TELEMSAMPLERATE;
        }
    } else if(str->substring(0,12) == "set_setpoint" && num_elements >= 4){
        setpoint_t new_sp;
        new_sp.pos = elements[i0].toDouble();
        new_sp.vel = elements[i0+1].toDouble();
        new_sp.accel = elements[i0+2].toDouble();
        new_sp.torque = elements[i0+3].toDouble();
        tcontrol_update_setpoint(ctrl, &new_sp);
        return cmd_type_t::SET_SETPOINT;
    } else if(str->substring(0,13) == "set_dutycycle" && num_elements >= 1) {
        int32_t new_dc = elements[i0].toInt();
        motor_set_pwm(ctrl->motor_handle, new_dc);
        ESP_LOGD(TAG, "Set pwm duty cycle to %li with frequency %lu.\nMotor State: %i.\n",
        motor_get_duty_cycle(ctrl->motor_handle), motor_get_frequency(ctrl->motor_handle), motor_get_state(ctrl->motor_handle));
        return cmd_type_t::SET_DUTYCYCLE;
    } else if(str->substring(0,17) == "set_multisetpoint") {
        return cmd_type_t::NA_CMD;
        // int16_t i0 = str->indexOf(',',0);
        // i0+=1;
        // int16_t i = str->indexOf(',',i0);
        // auto name = str->substring(i0,i);

        // auto cmd = cmd_type_t::NA_CMD;
        // Serial.printf("%s",name);
        // if (name=="position") {
        //     cmd = cmd_type_t::SET_MULTISETPOINT_POSITION;
        // } else if (name=="velocity") {
        //     cmd = cmd_type_t::SET_MULTISETPOINT_VELOCITY;
        // } else if (name=="acceleration") {
        //     cmd = cmd_type_t::SET_MULTISETPOINT_ACCELERATION;
        // } else if (name=="torque") {
        //     cmd = cmd_type_t::SET_MULTISETPOINT_TORQUE;
        // }

        // if (cmd >= cmd_type_t::SET_MULTISETPOINT_POSITION && cmd <= cmd_type_t::SET_MULTISETPOINT_TORQUE ) {
        //     i0 = i+1;
        //     i = str->indexOf(',',i0);
        //     auto num_vals = (uint32_t)str->substring(i0,i).toInt();
        //     auto vals = str->substring(i);
        //     insert_setpoint_buffer(cmd, &vals, (setpoint_t*)cfg->setpoint_buffer, cfg->buffer_mutex, num_vals);
        // }

        // return cmd;
    }

    return cmd_type_t::NA_CMD;
}

uint32_t publish_telemetry(telemetry_t *telem) {
    uint32_t size = 0;
    telemetry_t t = *telem;
    if(Serial) {
        size = Serial.printf("telem,time_ms:%lu,loop_dt:%lu,control_dt:%lu,read_dt:%lu,pid_success_flag:%i,position:%lf,pwm_duty_cycle:%lf,set_point:%f,velocity:%lf,current:%lf,torque_external:%lf,\n", 
            t.timestamp_ms, t.loop_dt, t.control_dt, t.read_dt, t.pid_success_flag, t.position, t.pwm_duty_cycle, t.setpoint.pos, t.velocity, t.current, t.torque_external);
    }
    return size;
}

void ack_cmd(cmd_type_t cmd) {
    ESP_LOGD(TAG, "acknowledging command %i", cmd);
    Serial.printf("/*TWIDDLERINO_ACK,%i*/\n",(int16_t)cmd);
}

//Serial studio frames are read as "/*TITLE,%s,%s,%s,...,%s*/"
//start of frame: /*
//end of frame: */
//docs https://github.com/Serial-Studio/Serial-Studio/wiki/Communication-Protocol
uint32_t publish_telemetry_serial_studio(telemetry_t *telem) {
    uint32_t size = 0;
    telemetry_t t = *telem;
    if(Serial) {
        nframes_sent_serial+=1;
        size = Serial.printf("/**TWIDDLERINO_TELEMETRY,%lu,%lu,%lu,%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lu,%lu,%i,%lf,%s**/\n", 
            t.timestamp_ms, t.loop_dt, t.control_dt, t.read_dt, 
            t.pwm_duty_cycle, t.pwm_frequency, 
            t.position, t.velocity, t.filtered_velocity, t.current, t.filtered_current, t.torque_external, t.torque_control, t.torque_net,
            t.setpoint.pos, t.setpoint.vel, t.setpoint.accel, t.setpoint.torque,
            t.Kp, t.Ki, t.Kd, t.impedance.K, t.impedance.B, t.impedance.J, t.nframes_sent_queue, 
            nframes_sent_serial, (int16_t)t.control_type, t.current_sens_adc_volts, t.ctrl_id);
    }
    return size;
}

uint32_t print_controller_cfg(twid_controller_t* ctrl_handle) {
    uint32_t size = 0;

    if(Serial) {
        controller_config_t cfg = ctrl_handle->config;
        size = Serial.printf("################\n%s CONTROLLER CONFIGURATION:\n*control_type:%i\t*sample_time:%lu us\n*Kp:%lf\t*Ki:%lf\t*Kd:%lf\n*stiffness:%lf\t*damping:%lf\t*intertia:%lf\n################\n",
        ctrl_handle->ctrl_id, cfg.control_type, cfg.sample_time_us, cfg.Kp, cfg.Ki, cfg.Kd, cfg.impedance.K, cfg.impedance.B, cfg.impedance.J);
    }

    return size;
}

String read_string_until(char terminator) {
    String ret;
    int c = timed_read();
    while(c >= 0 && c != terminator) {
        ret += (char) c;
        c = timed_read();
    }
    return ret;
}

void extract_doubles(String * str, double* out, uint16_t num_values) {
    int32_t i = 0, i0 = 0;
    for(int j = 0; j < num_values; j++) {
        i0 = str->indexOf(",",i0);
        i0++;
        i = str->indexOf(",",i0);
        if (i < 0) {
            i = str->length() - 1;
        }
        out[j] = str->substring(i0,i).toDouble();
        i++;
    }
}

static void insert_setpoint_buffer(cmd_type_t cmd, String * str, 
setpoint_t* buffer, SemaphoreHandle_t* mutex, uint16_t num_values) {
    ESP_LOGD(TAG, "Trying to insert %i setpoints into buffer", num_values);
    int32_t i = 0, i0 = 0;
    for(int j = 0; j < num_values; j++) {
        i0 = str->indexOf(",",i0);
        i0++;
        i = str->indexOf(",",i0);
        if (i < 0) {
            i = str->length() - 1;
        }

        auto val = str->substring(i0,i).toDouble();
        setpoint_t sp = {.pos = 0, .vel = 0, .accel = 0, .torque = 0};

        switch(cmd) {
            case cmd_type_t::SET_MULTISETPOINT_POSITION :
                sp.pos = val;
                break;
            case cmd_type_t::SET_MULTISETPOINT_VELOCITY:
                sp.vel = val;
                break;
            case cmd_type_t::SET_MULTISETPOINT_ACCELERATION:
                sp.accel = val;
                break;
            case cmd_type_t::SET_MULTISETPOINT_TORQUE:
                sp.torque = val;
                break;
            default:
                return;
        }
        xSemaphoreTake(*mutex, portMAX_DELAY);
        buffer[i] = sp;
        xSemaphoreGive(*mutex);
        i++;
    }
}

void reset_sent_count() {
    nframes_sent_serial = 0;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

uint16_t extract_cs_strings(String * str, String *out) {
    int32_t i = 0, i0 = 0;
    uint16_t num = 0;
    char buffer[100];
    for(int j = 0; j < MAX_ELEMENTS; j++) {
        i0 = str->indexOf(",",i0);
        i0++;
        i = str->indexOf(",",i0);
        if (i < 0) {
            i = str->length() - 1;
            break;
        }
        out[j] = str->substring(i0,i);
        out[j].toCharArray(buffer, 100);

        //verbose log, only compiles if build flag is set
        ESP_LOGD(TAG, "Extracted substring %s", buffer);

        num++;
        vTaskDelay(1);
    }

    return num;
}

static char timed_read(){
    char c;
    uint32_t start = micros();
    do {
        c = Serial.read();
        if(c >= 0) {
            return c;
        }
    } while(micros() - start < READ_TIMEOUT_US);
    return -1;     // -1 indicates timeout
}