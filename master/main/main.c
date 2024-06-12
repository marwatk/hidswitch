/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

#include "led_strip.h"



#include "sdkconfig.h"


#include "messages.h"
#include "master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "main.h"

#define LED_GPIO 48

static led_strip_handle_t led_strip;
static QueueHandle_t led_queue;
static QueueHandle_t send_queue;

static char *TAG = "MAIN";

static void configure_led(void) {
    ESP_LOGI(TAG, "Initializing LED");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

static void led_task(void *arg) {
    LedMessage msg;
    while(1) {
        if (xQueueReceive(led_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "LED set: %d,%d,%d", msg.red, msg.green, msg.blue);
            led_strip_set_pixel(led_strip, 0, msg.red, msg.green, msg.blue);
            led_strip_refresh(led_strip);
        }
    }    
}

void led_test(void *arg) {
    LedMessage localMsg;
    localMsg.green = 0;
    localMsg.blue = 0;

    uint8_t color = 0;
    Message remoteMsg;
    remoteMsg.len = sizeof(LedMessage);
    remoteMsg.slave = 0;
    remoteMsg.type = MESSAGE_TYPE_LED;
    while(1) {
        color = color == 0 ? 16 : 0;
        localMsg.red = color;
        xQueueSendToBack(led_queue, &localMsg, 0);
        remoteMsg.slave = 0;
        memset(remoteMsg.data, 0, sizeof(LedMessage));
        ((LedMessage*)remoteMsg.data)->green = color;
        xQueueSendToBack(send_queue, &remoteMsg, 0);
        remoteMsg.slave = 1;
        memset(remoteMsg.data, 0, sizeof(LedMessage));
        ((LedMessage*)remoteMsg.data)->blue = color;
        xQueueSendToBack(send_queue, &remoteMsg, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    configure_led();

    send_queue = xQueueCreate( 10, sizeof( Message ) );
    led_queue = xQueueCreate(10, sizeof(LedMessage));

    xTaskCreate(led_task, "led_task", 1024 * 2, (void *)0, 10, NULL);
    master_main(send_queue);
}
