#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "hid_host.h"
#include "messages.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "main.h"
#include <esp32/rom/crc.h>
#include <time.h>

#define I2C_MASTER_SCL_IO 9               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define CRC_ERROR 0x8a

static char *TAG = "MSTR";

static uint8_t active_slave = 0;

static HidQueues hid_queues;
static QueueHandle_t send_queue;

static uint8_t meta_modifiers = 0x11; // Left ctrl + Right ctrl
static uint8_t left_ctrl_only_modifier = 0x1; 
static uint8_t left_ctrl_left_shift_modifier = 0x3;

#define INACTIVE_MOUSE_BOUNDING_BOX 20

static int8_t inactive_mouse_x = INACTIVE_MOUSE_BOUNDING_BOX / 2;
static int8_t inactive_mouse_y = INACTIVE_MOUSE_BOUNDING_BOX / 2;

static hid_mouse_input_report_boot_extended_t empty_mouse_report = { 0 };
static hid_keyboard_input_report_boot_t empty_kb_report = { 0 };

static int switch_taps = 0;
static clock_t switch_tap_time;

hid_keyboard_output_report_boot_t states[NUM_SLAVES] = { 0 };

// Adapted from: https://github.com/espressif/esp-idf/blob/cbce221e88d52665523093b2b6dd0ebe3f1243f1/examples/peripherals/i2c/i2c_self_test/main/i2c_example_main.c#L101
static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_wr, size_t size)
{
    uint8_t crc = crc8_le(0, data_wr, size);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, crc, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// From: https://github.com/espressif/esp-idf/blob/cbce221e88d52665523093b2b6dd0ebe3f1243f1/examples/peripherals/i2c/i2c_self_test/main/i2c_example_main.c#L71
static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 0);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void update_leds() {
    Message msg;
    for( int i = 0; i < NUM_SLAVES; i++ ) {
        msg.slave = i;
        msg.type = MESSAGE_TYPE_LED;
        msg.len = sizeof(LedMessage);
        if ( !states[i].led_state.tud_mounted ) {
            ((LedMessage*)msg.data)->green = 0;
            ((LedMessage*)msg.data)->red = 12;
            ((LedMessage*)msg.data)->blue = 0;
        }
        else if ( i == active_slave ) {
            ((LedMessage*)msg.data)->green = 12;
            ((LedMessage*)msg.data)->red = 0;
            ((LedMessage*)msg.data)->blue = 0;
        }
        else {
            ((LedMessage*)msg.data)->green = 0;
            ((LedMessage*)msg.data)->red = 0;
            ((LedMessage*)msg.data)->blue = 12;
        }
        xQueueSendToBack(send_queue, &msg, 0);
    }
}

static void master_sender_task(QueueHandle_t send_queue) {
    Message msg;
    esp_err_t err;
    uint8_t *buf = malloc(512);
    while(1) {
        if ( xQueueReceive( send_queue, &msg, portMAX_DELAY ) == pdTRUE) {
            esp_err_t ret;
            uint16_t slave_addr = ESP_SLAVE_BASE_ADDR + msg.slave;
            
            if (msg.type == MESSAGE_TYPE_MOUSE) {
                hid_mouse_input_report_boot_extended_t *mouse_report = msg.data;
                ESP_LOGD(TAG, "Mous%d: %d,%d, wheel: %d buttons: %d %d %d %d %d %d %d %d",
                    msg.slave,
                    mouse_report->x_displacement,
                    mouse_report->y_displacement,
                    mouse_report->scroll_displacement,
                    mouse_report->buttons.button1,
                    mouse_report->buttons.button2,
                    mouse_report->buttons.button3,
                    mouse_report->buttons.button4,
                    mouse_report->buttons.button5,
                    mouse_report->buttons.button6,
                    mouse_report->buttons.button7,
                    mouse_report->buttons.button8);
            }
            
            buf[0] = msg.type;
            buf[1] = msg.len;
            memcpy(&buf[2], msg.data, msg.len);
            err = i2c_master_write_slave(I2C_MASTER_NUM, slave_addr, buf, msg.len + 2);
            if ( err != ESP_OK) {
                ESP_LOGD(TAG, "Err i2c send to %d: %d", msg.slave, err);
            }
            else {
                ESP_LOGD(TAG, "i2c send to %d: %d (%d) [0]: %x", msg.slave, msg.type, msg.len, ((uint8_t*)msg.data)[0]);
            }
            
            hid_keyboard_output_report_boot_t new_state = { 0 };
            err = i2c_master_read_slave(I2C_MASTER_NUM, slave_addr, &new_state, sizeof(hid_keyboard_output_report_boot_t));
            if ( err != ESP_OK) {
                ESP_LOGD(TAG, "Err i2c read from to %d: %d", msg.slave, err);
            }
            else {
                uint8_t crc = crc8_le(0, &new_state.led_state.val, sizeof(uint8_t));
                if ( crc != new_state.checksum ) {
                    ESP_LOGD(TAG, "SLV%d: bad checksum (%x, %x, %x)", msg.slave, new_state.led_state.val, new_state.checksum, crc);
                }
                else {
                    ESP_LOGD(TAG, "LS%d: %x", msg.slave, new_state.led_state.val);
                    if (new_state.led_state.val != states[msg.slave].led_state.val) {
                        states[msg.slave].led_state.val = new_state.led_state.val;
                        update_leds();
                    }
                }
            }
        }
    }
}

int8_t clamp_mouse(int8_t displacement, int8_t current_pos) {
    int8_t new_pos = displacement + current_pos;
    if ( new_pos < 0 ) {
        return displacement - new_pos;
    }
    if ( new_pos > INACTIVE_MOUSE_BOUNDING_BOX ) {
        return displacement - (new_pos - INACTIVE_MOUSE_BOUNDING_BOX);
    }
    return displacement;
}

static void set_slave(uint8_t slave_idx) {
    ESP_LOGI(TAG, "Switching to slave %d", slave_idx);
    Message msg;
    for( int i = 0; i < NUM_SLAVES; i++ ) {
        msg.type = MESSAGE_TYPE_KB;
        memcpy(msg.data, &empty_kb_report, sizeof(hid_keyboard_input_report_boot_t));
        msg.slave = i;
        msg.len = sizeof(hid_keyboard_input_report_boot_t);
        xQueueSendToBack(send_queue, &msg, 0);
        msg.type = MESSAGE_TYPE_MOUSE;
        memcpy(msg.data, &empty_mouse_report, sizeof(hid_mouse_input_report_boot_extended_t));
        msg.len = sizeof(hid_mouse_input_report_boot_extended_t);
        xQueueSendToBack(send_queue, &msg, 0);
    }
    active_slave = slave_idx;
    update_leds();
}

void mouse_listener_task() {
    hid_mouse_input_report_boot_extended_t report;
    esp_err_t err;
    hid_mouse_input_report_boot_extended_t *to_send;
    Message msg;
    int8_t x;
    int8_t y;
    while(1) {
        if ( xQueueReceive( hid_queues.mouse_queue, &report, portMAX_DELAY ) == pdTRUE) {
            switch_taps = 0;
            for( int i = 0; i < NUM_SLAVES; i++ ) {
                msg.len = sizeof(hid_mouse_input_report_boot_extended_t);
                msg.slave = i;
                msg.type = MESSAGE_TYPE_MOUSE;
                memcpy(msg.data, &report, sizeof(hid_mouse_input_report_boot_extended_t));
                if( i != active_slave ) {

                    x = clamp_mouse(report.x_displacement, inactive_mouse_x);
                    y = clamp_mouse(report.y_displacement, inactive_mouse_y);
                    inactive_mouse_x += x;
                    inactive_mouse_y += y;
                    if ( x == 0 && y == 0 ) {
                        // No sense in sending an empty report to the inactive slaves
                        continue;
                    }
                    ((hid_mouse_input_report_boot_extended_t*)msg.data)->buttons.val = 0;
                    ((hid_mouse_input_report_boot_extended_t*)msg.data)->scroll_displacement = 0;
                    ((hid_mouse_input_report_boot_extended_t*)msg.data)->x_displacement = x;
                    ((hid_mouse_input_report_boot_extended_t*)msg.data)->y_displacement = y;
                }
                xQueueSendToBack(send_queue, &msg, 0);
            }
        }
    }
}

void next_slave() {
    ESP_LOGD(TAG, "Next slave");
    int next_slave = active_slave;
    for (int i = 0; i < NUM_SLAVES; i++ ) {
        next_slave++;
        if ( next_slave >= NUM_SLAVES ) {
            next_slave = 0;
        }
        
        if (states[next_slave].led_state.tud_mounted) {
            set_slave(next_slave);
            return;
        }
        else {
            ESP_LOGD(TAG, "Slave %d is not connected", next_slave);
        }
    }
}

void switch_checker(uint8_t modifiers) {
    // More modifiers than left ctrl+shift pressed
    if ( (modifiers | left_ctrl_left_shift_modifier) != left_ctrl_left_shift_modifier) {
        switch_taps = 0;
        return;
    }
    if ( modifiers == 0 ) {
        if ( switch_taps % 2 == 1 ) {
            // Nothing pressed but we've started the cycle, increment us
            switch_taps++;
        }
        if ( switch_taps >= 4 ) {
            switch_taps = 0;
            next_slave();
        }
        return;
    }
    if ( (modifiers & left_ctrl_left_shift_modifier) == left_ctrl_left_shift_modifier ) {
        if ( switch_taps % 2 == 0 ) {
            switch_taps++;
        }
    }
}

void kb_listener_task() {
    hid_keyboard_input_report_boot_t report;
    esp_err_t err;
    hid_keyboard_input_report_boot_t *to_send;
    Message msg;
    while(1) {
        if ( xQueueReceive( hid_queues.kb_queue, &report, portMAX_DELAY ) == pdTRUE) {
            if ( report.modifier.val == meta_modifiers && report.key[0] >= 0x1e && report.key[0] <= 0x27 && report.key[1] == 0 ) {
                set_slave( report.key[0] - 0x1e );
            }
            else {
                msg.len = sizeof(hid_keyboard_input_report_boot_t);
                msg.slave = active_slave;
                msg.type = MESSAGE_TYPE_KB;
                memcpy(msg.data, &report, sizeof(hid_keyboard_input_report_boot_t));
                xQueueSendToBack(send_queue, &msg, 0);
            }
            if ( report.key[0] == 0 ) {
                switch_checker(report.modifier.val);
            }
            else {
                switch_taps = 0;
            }
        }
    }
}

void master_main(QueueHandle_t send_q) {
    ESP_LOGI(TAG, "Running as master");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    send_queue = send_q;
    hid_queues.mouse_queue = xQueueCreate( 10, sizeof(hid_mouse_input_report_boot_extended_t) );
    hid_queues.kb_queue = xQueueCreate( 10, sizeof(hid_keyboard_input_report_boot_t) );

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, ESP_INTR_FLAG_SHARED));

    xTaskCreate(master_sender_task, "master_sender", 1024 * 2, (QueueHandle_t)send_queue, 10, NULL);
    xTaskCreate(hid_host_main_task, "hid_host_main", 1024 * 2, (void *)&hid_queues, 10, NULL);
    xTaskCreate(kb_listener_task, "kb_listener", 1024 * 2, (void *)&hid_queues, 10, NULL);
    xTaskCreate(mouse_listener_task, "mouse_listener", 1024 * 2, (void *)&hid_queues, 10, NULL);
    set_slave( 0 );
}
