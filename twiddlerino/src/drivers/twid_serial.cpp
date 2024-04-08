/**
 * @file twid_serial.cpp
 * @brief esp32 uart driver with event pattern detection
 * @author Yousif El-Wishahy (ywishahy@student.ubc.ca)
 * 
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html#
 * 
 */

/******************************************************************************/
/*                              I N C L U D E S                               */
/******************************************************************************/
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

//library header
#include "drivers/twid_serial.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#include "app/twid32_config.h"

#include "app/comms.h"

/******************************************************************************/
/*                               D E F I N E S                                */
/******************************************************************************/

#define UART_RX_BUFFER_SIZE (2048)
#define UART_TX_BUFFER_SIZE (1024)
#define UART_EVENT_QUEUE_SIZE 100
#define UART_TASKS_CORE 0

/******************************************************************************/
/*                P U B L I C  G L O B A L  V A R I A B L E S                 */
/******************************************************************************/

/******************************************************************************/
/*            P R I V A T E  F U N C T I O N  P R O T O T Y P E S             */
/******************************************************************************/\

static void uart_event_task(void *pvParameters);

/******************************************************************************/
/*               P R I V A T E  G L O B A L  V A R I A B L E S                */
/******************************************************************************/
static const char* TAG = "twid_serial";

static QueueHandle_t uart0_queue;
static TaskHandle_t uart0_event_task_handle;

/******************************************************************************/
/*                       P U B L I C  F U N C T I O N S                       */
/******************************************************************************/

void uart_init() {
    //params for uart driver
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t local_err = ESP_OK;
    local_err = uart_driver_install(UART_NUM_0, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE , UART_EVENT_QUEUE_SIZE, &uart0_queue, 0);
    ESP_LOGI(TAG, "uart_driver_install complete");
    local_err = uart_param_config(UART_NUM_0, &uart_config);
    ESP_LOGI(TAG, "uart_param_config complete");

    //Set UART pins (using UART0 default pins ie no changes.)
    local_err = uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "uart_set_pin UART_NUM_0");

    //Set uart pattern detect function.
    local_err = uart_enable_pattern_det_baud_intr(UART_NUM_0, '\n', 1, 10, 0, 0);
    ESP_LOGI(TAG, "uart_enable_pattern_det_baud_intr on new line termination");

    //Reset the pattern queue length to record at most 20 pattern positions.
    local_err = uart_pattern_queue_reset(UART_NUM_0, UART_EVENT_QUEUE_SIZE);

    ESP_ERROR_CHECK(local_err);

    //Create a task to handler UART event from ISR
    xTaskCreatePinnedToCore(uart_event_task, "twid_uart_event_task", 4096, NULL, configMAX_PRIORITIES-1, &uart0_event_task_handle, UART_TASKS_CORE);
    ESP_LOGI(TAG, "Initialized twid_uart_event_task with stack depth 4096");
}

int twid_uart_write(const char *format, ...) {
    /*
    implementation copied from Print.cpp
    Print.cpp - Base class that provides print() and println()
    Copyright (c) 2008 David A. Mellis.  All right reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
    */
    char loc_buf[64];
    char * temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
    va_end(copy);
    if(len < 0) {
        va_end(arg);
        return 0;
    }
    if(len >= (int)sizeof(loc_buf)){  // comparation of same sign type for the compiler
        temp = (char*) malloc(len+1);
        if(temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len+1, format, arg);
    }
    va_end(arg);
    len = uart_write_bytes(UART_NUM_0, (uint8_t*)temp, len);
    if(temp != loc_buf){
        free(temp);
    }
    return len;
}

/******************************************************************************/
/*                      P R I V A T E  F U N C T I O N S                      */
/******************************************************************************/

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(UART_RX_BUFFER_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, UART_RX_BUFFER_SIZE);
            ESP_LOGD(TAG, "uart[%d] event:", UART_NUM_0);
            switch (event.type) {
            //Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGV(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
                ESP_LOGV(TAG, "[DATA EVT]: %s",(const char*) dtmp);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(uart0_queue);
                ESP_LOGI(TAG, "flushed uart and reset event queue");
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(uart0_queue);
                ESP_LOGI(TAG, "flushed uart and reset event queue");
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGW(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGW(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGW(TAG, "uart frame error");
                break;
            case UART_DATA_BREAK: {
                ESP_LOGI(TAG, "UART TX data and break event");
                break;
            }
            //UART_PATTERN_DET
            case UART_PATTERN_DET: {
                uart_get_buffered_data_len(UART_NUM_0, &buffered_size);
                int pos = uart_pattern_pop_pos(UART_NUM_0);
                ESP_LOGD(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(UART_NUM_0);
                } else {
                    uart_read_bytes(UART_NUM_0, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[UART_NUM_0 + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(UART_NUM_0, pat, 1, 100 / portTICK_PERIOD_MS);
                    ESP_LOGD(TAG, "read data: %s", dtmp);
                    ESP_LOGD(TAG, "read pat : %s", pat);
                    auto cmd_type = handle_command((const char*) dtmp, pos);
                    ack_cmd(cmd_type);
                }
                break;
            }
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}