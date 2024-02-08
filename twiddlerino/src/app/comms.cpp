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

char timed_read();

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/



/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

//initializes Twiddlerino
cmd_type_t decode_cmd(String * string, test_config_t *test_config) {
    uint16_t i = 0;
    uint16_t i0 = 0;
    String read_string = *string;
    test_config_t t = *test_config;
    t.cmd_type = cmd_type_t::NA_CMD;

    //'config_test,P:{params.P},I:{params.I},D:{params.D},set_point:{params.SetPoint}'
    if(read_string.substring(0,10).compareTo("config_test") == 0){
        t.cmd_type = cmd_type_t::CONFIG_TEST;
        i0 = read_string.indexOf(',',15);
        t.Kp = read_string.substring(15,i0-1).toDouble();
        i = read_string.indexOf(',', i0+3);
        t.Ki = read_string.substring(i0+3,i-1).toDouble();
        i0 = i+3;
        i = read_string.indexOf(',', i0);
        t.Kd = read_string.substring(i0,i-1).toDouble();
        i0 = i+3;
        i = read_string.indexOf(',', i0);
        t.set_point = read_string.substring(i0,i-1).toDouble();
    } else if (read_string.substring(0,9).compareTo("start_test") == 0) {
        t.cmd_type = cmd_type_t::START_TEST;
    } else if (read_string.substring(0,9).compareTo("abort_test") == 0) {
        t.cmd_type = cmd_type_t::ABORT_TEST;
    }

    return t.cmd_type;

}

uint32_t publish_telemetry(control_telemetry_t *telem) {
    uint32_t size = -1;
    control_telemetry_t t = *telem;
    if(Serial.availableForWrite()){
      size = Serial.printf("time_ms:%lu,loop_dt:%lu,control_dt:%lu,read_dt:%lu,pid_success_flag:%i,position:%lf,pwm_duty_cycle:%lf,set_point:%f,velocity:%lf,current:%lf,torque_external:%lf\n", 
        t.timestamp_ms, t.loop_dt, t.control_dt, t.read_dt, t.pid_success_flag, t.position, t.pwm_duty_cycle, t.set_point, t.velocity, t.current, t.torque_external);
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

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

char timed_read(){
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