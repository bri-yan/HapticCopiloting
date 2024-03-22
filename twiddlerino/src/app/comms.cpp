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


/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define READ_TIMEOUT_US 200U

/******************************************************************************/
/*                              T Y P E D E F S                               */
/******************************************************************************/

// Typedefs that are only used in this file, empty for now

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

static char timed_read();

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

static uint32_t nframes_sent_serial = 0;


/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

bool decode_config_cmd(String *str, controller_config_t *cfg) {
    if(str->substring(0,7).compareTo("set_pid") == 0){
        double vals[3] = {0.0 ,0.0, 0.0};
        extract_doubles(str, vals, 3);
        cfg->Kp = vals[0];
        cfg->Ki = vals[1];
        cfg->Kd = vals[2];
        return true;
    } else if(str->substring(0,13).compareTo("set_impedance") == 0) {
        double vals[3] = {0.0,0.0,0.0};
        extract_doubles(str, vals, 3);
        cfg->impedance.K = vals[0];
        cfg->impedance.B = vals[1];
        cfg->impedance.J = vals[2];
        return true;
    } else if(str->substring(0,8).compareTo("set_mode") == 0) {
        int16_t i0 = str->indexOf(',',0);
        i0+=1;
        int16_t i = str->indexOf(',',i0);
        auto name = str->substring(i0,i);

        control_type_t mode = cfg->control_type;
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
        } else {
            return false;
        }

        cfg->control_type = mode;
        return true;
    } else if(str->substring(0,19) == "set_telemsamplerate") {
        double vals[1] = {0.0};
        extract_doubles(str, vals, 1);
        if (vals[0] >= 1) {
            cfg->telemetry_sample_rate = (uint32_t)vals[0];
            return true;
        }
    }

    return false;

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

//Serial studio frames are read as "/*TITLE,%s,%s,%s,...,%s*/"
//start of frame: /*
//end of frame: */
//docs https://github.com/Serial-Studio/Serial-Studio/wiki/Communication-Protocol
uint32_t publish_telemetry_serial_studio(telemetry_t *telem) {
    uint32_t size = 0;
    telemetry_t t = *telem;
    if(Serial) {
        nframes_sent_serial+=1;
        size = Serial.printf("/*TWIDDLERINO_TELEMETRY,%lu,%lu,%lu,%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lu,%lu*/\n", 
            t.timestamp_ms, t.loop_dt, t.control_dt, t.read_dt, 
            t.pwm_duty_cycle, t.pwm_frequency, 
            t.position, t.velocity, t.filtered_velocity, t.current, t.filtered_current, t.torque_external, t.torque_control, t.torque_net,
            t.setpoint.pos, t.setpoint.vel, t.setpoint.accel, t.setpoint.torque,
            t.Kp, t.Ki, t.Kd, t.impedance.K, t.impedance.B, t.impedance.J, t.nframes_sent_queue, nframes_sent_serial);
    }
    return size;
}

uint32_t print_controller_cfg() {
    uint32_t size = 0;

    if(Serial) {
        controller_config_t cfg;
        tcontrol_get_cfg(&cfg);
        size = Serial.printf("################\nCONTROLLER CONFIGURATION:\n*control_type:%i\t*sample_time:%lu us\n*Kp:%lf\t*Ki:%lf\t*Kd:%lf\n*stiffness:%lf\t*damping:%lf\t*intertia:%lf\n################\n",
        cfg.control_type, cfg.sample_time_us, cfg.Kp, cfg.Ki, cfg.Kd, cfg.impedance.K, cfg.impedance.B, cfg.impedance.J);
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

//initializes Twiddlerino
cmd_type_t decode_test_cmd(String * read_string, test_config_t *t) {
    int32_t i = 0;
    int32_t i0 = 0;
    t->cmd_type = cmd_type_t::NA_CMD;

    //'config_test,P:{params.P},I:{params.I},D:{params.D},set_point:{params.SetPoint}'
    if(read_string->substring(0,11).compareTo("config_test") == 0){
        t->cmd_type = cmd_type_t::CONFIG_TEST;

        i0 = read_string->indexOf(':',0);
        i0+=1;
        i = read_string->indexOf(',',i0);
        t->Kp = read_string->substring(i0,i).toDouble();
        Serial.printf("substring decoded from %i to %i : %s\n",i0,i,read_string->substring(i0,i));

        i0 = read_string->indexOf(':',i0+1);
        i0+=1;
        i = read_string->indexOf(',',i0);
        t->Ki = read_string->substring(i0,i).toDouble();
        Serial.printf("substring decoded from %i to %i : %s\n",i0,i,read_string->substring(i0,i));

        i0 = read_string->indexOf(':',i0+1);
        i0+=1;
        i = read_string->indexOf(',',i0);
        t->Kd = read_string->substring(i0,i).toDouble();
        Serial.printf("substring decoded from %i to %i : %s\n",i0,i,read_string->substring(i0,i));

        i0 = read_string->indexOf(':',i0+1);
        i0+=1;
        i = read_string->indexOf(',',i0);
        t->set_point = read_string->substring(i0,i).toDouble();
        Serial.printf("substring decoded from %i to %i : %s\n",i0,i,read_string->substring(i0,i));

        i0 = read_string->indexOf(':',i0+1);
        i0+=1;
        i = read_string->indexOf(',',i0);
        t->sample_rate_us = read_string->substring(i0,i).toInt();
        Serial.printf("substring decoded from %i to %i : %s\n",i0,i,read_string->substring(i0,i));

        i0 = read_string->indexOf(':',i0+1);
        i0+=1;
        i = read_string->indexOf(',',i0);
        if (i < 0 || i > read_string->length()) {
            i = read_string->length();
        }
        t->test_duration_ms = read_string->substring(i0,i).toInt();
        Serial.printf("substring decoded from %i to %i : %s\n",i0,i,read_string->substring(i0,i));

        // Serial.printf("Starting control task with params:\n\t\tKp:%lf\tKi:%lf\tKd:%lf\tsample_rate_us:%lu\ttest_duration_ms:%lu\n",
        //   t.Kp,t.Ki,t.Kd,t.sample_rate_us,t.test_duration_ms);
    } else if (read_string->substring(0,10).compareTo("start_test") == 0) {
        t->cmd_type = cmd_type_t::START_TEST;
    } else if (read_string->substring(0,10).compareTo("abort_test") == 0) {
        t->cmd_type = cmd_type_t::ABORT_TEST;
    }

    return t->cmd_type;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

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