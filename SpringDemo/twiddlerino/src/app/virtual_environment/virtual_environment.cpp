/**
 * @file virtual_environment.h
 * @brief Code related to haptic virtual environment interface
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/

#include "app/virtual_environment/virtual_environment.h"

#include "app/control/twid_control.h"

#include "app/twid32_config.h"

#include "app/comms.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/

void TaskSendGameData(void *pvParameters);

/**
 * @brief Send data to game interface
 * 
 */
uint32_t game_send_data();

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/

static uint32_t data_send_delay_ms = GAME_TELEMETRY_WAIT_MS;
static TaskHandle_t xGameSendDataTaskHandle;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void game_interface_init() {

    xTaskCreatePinnedToCore(
        TaskSendGameData
        ,  "Send Game Data Task"
        ,  8192
        ,  NULL
        ,  TASK_PRIORITY_GAME_SEND
        ,  &xGameSendDataTaskHandle
        ,  CORE_SERIAL_WRITE_TASK
    );

}

bool game_process_command(String* str) {
    if(str->substring(0,4) != "game") {
        return false;
    }

    if(str->substring(0,17) == "game_set_setpoint") {
        double vals[4] = {0.0, 0.0, 0.0, 0.0};
        extract_doubles(str, vals, 4);

        setpoint_t sp;
        sp.pos = vals[0];
        sp.vel = vals[1];
        sp.accel = vals[2];
        sp.torque = vals[3];

        tcontrol_update_setpoint(&sp);
        if(Serial.availableForWrite()) {
            Serial.printf("Setpoint updated.\n");
        }

        return true;
    } else if(str->substring(0,18) == "game_set_setpoint_multiple") {
        if(Serial.availableForWrite()) {
            Serial.printf("Feature not implemented yet.\n");
        }
        return false;;
    } else if(str->substring(0,18) == "game_set_datadelay") {
        if(Serial.availableForWrite()) {
            Serial.printf("Feature not implemented yet.\n");
        }
        return false;;
    }

    return false;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

uint32_t game_send_data() {
    uint32_t size = 0;
    telemetry_t t;
    tcontrol_get_state(&t);
    if(Serial) {
        // size = Serial.printf("/*TWIDDLERINO_TELEMETRY,timestamp_ms:%lu,position:%lf,velocity:%lf,position_setpoint:%lf,velocity_setpoint:%lf,*/\n", 
        //     t.timestamp_ms, t.position, t.filtered_velocity, t.setpoint.pos, t.setpoint.vel);

        size = Serial.printf("/*TWIDDLERINO_TELEMETRY,timestamp_ms:%lu,position:%lf,velocity:%lf,current:%lf,torque_external:%lf,torque_control:%lf,torque_net:%lf,position_setpoint:%lf,velocity_setpoint:%lf,accel_setpoint:%lf,torque_setpoint:%lf,Kp:%lf,Ki:%lf,Kd:%lf,impedance_K:%lf,impedance_B:%lf,impedance_J:%lf,*/\n", 
            t.timestamp_ms,
            t.position, t.filtered_velocity, t.filtered_current, t.torque_external, t.torque_control, t.torque_net,
            t.setpoint.pos, t.setpoint.vel, t.setpoint.accel, t.setpoint.torque,
            t.Kp, t.Ki, t.Kd, t.impedance.K, t.impedance.B, t.impedance.J);
    }
    return size;
}

void TaskSendGameData(void *pvParameters) {
    if(!Serial) {
        Serial.begin( UART_BAUD_RATE );
        Serial.println( "Serial connected on uart0!" );
    }

    Serial.printf("Started Send Game Data Task.\n");

    for(;;){
        game_send_data();
        vTaskDelay(pdMS_TO_TICKS(data_send_delay_ms));
    }
}