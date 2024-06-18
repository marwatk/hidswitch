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
#include "usb/hid_usage_keyboard.h"

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

static char *TAG = "MSTR";

static uint8_t active_slave = 0;

static HidQueues hid_queues;
static QueueHandle_t send_queue;

static uint8_t meta_modifiers = 0x11; // Left ctrl + Right ctrl

#define INACTIVE_MOUSE_BOUNDING_BOX 20

static int8_t inactive_mouse_x = INACTIVE_MOUSE_BOUNDING_BOX / 2;
static int8_t inactive_mouse_y = INACTIVE_MOUSE_BOUNDING_BOX / 2;

static hid_mouse_input_report_boot_extended_t empty_mouse_report = { 0 };
static hid_keyboard_input_report_boot_t empty_kb_report = { 0 };

typedef struct {
    union {
        struct {
            uint8_t num_lock:    1;
            uint8_t caps_lock:   1;
            uint8_t scroll_lock: 1;
            uint8_t compose:     1;
            uint8_t kana:        1;
            uint8_t reserved1:    1;
            uint8_t reserved2:    1;
            uint8_t tud_mounted: 1;
        };
        uint8_t val;
    } led_state;
} __attribute__((packed)) hid_keyboard_output_report_boot_t;

hid_keyboard_output_report_boot_t states[NUM_SLAVES] = { 0 };

static void master_sender_task(QueueHandle_t send_queue) {
    // if( xQueueSendToBack( xQueue1, ( void * ) &ulVar, ( TickType_t ) 10 ) != pdPASS )
    // {
         // Failed to post the message, even after 10 ticks.
    // }
    Message msg;
    esp_err_t err;
    uint8_t buf[512];
    while(1) {
        if ( xQueueReceive( send_queue, &msg, portMAX_DELAY ) == pdTRUE) {
            esp_err_t ret;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();    
            i2c_master_start(cmd);
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
            
            hid_keyboard_output_report_boot_t new_state = { 0 };

            i2c_master_write_byte(cmd, slave_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, msg.type, 0);
            i2c_master_write_byte(cmd, msg.len, 0);
            i2c_master_write(cmd, msg.data, msg.len, ACK_CHECK_EN);
            //i2c_master_read_byte(cmd, &new_state, NACK_VAL);
            i2c_master_stop(cmd);
            err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

            if ( err != ESP_OK) {
                ESP_LOGI(TAG, "Err i2c send to %d: %d", msg.slave, err);
            }
            else {
                ESP_LOGD(TAG, "i2c send to %d: %d (%d) [0]: %x", msg.slave, msg.type, msg.len, ((uint8_t*)msg.data)[0]);
            }
            i2c_cmd_link_delete(cmd);            
            if (new_state.led_state.val != states[msg.slave].led_state.val) {
                ESP_LOGI(TAG, "New state[%d]: num: %d, caps: %d, scroll: %d, connected: %d",
                    msg.slave,
                    new_state.led_state.num_lock,
                    new_state.led_state.caps_lock,
                    new_state.led_state.scroll_lock,
                    new_state.led_state.tud_mounted );
                states[msg.slave] = new_state;
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
        msg.type = MESSAGE_TYPE_LED;
        msg.len = sizeof(LedMessage);
        if ( i == slave_idx ) {
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
    active_slave = slave_idx;
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

void kb_listener_task() {
    hid_keyboard_input_report_boot_t report;
    esp_err_t err;
    hid_keyboard_input_report_boot_t *to_send;
    Message msg;
    while(1) {
        if ( xQueueReceive( hid_queues.kb_queue, &report, portMAX_DELAY ) == pdTRUE) {
            if ( report.modifier.val == meta_modifiers ) {
                if ( report.key[0] >= 0x1e && report.key[0] <= 0x27 ) { // number keys 1-0
                    set_slave( report.key[0] - 0x1e );
                }
            }
            else {
                msg.len = sizeof(hid_keyboard_input_report_boot_t);
                msg.slave = active_slave;
                msg.type = MESSAGE_TYPE_KB;
                memcpy(msg.data, &report, sizeof(hid_keyboard_input_report_boot_t));
                xQueueSendToBack(send_queue, &msg, 0);
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
