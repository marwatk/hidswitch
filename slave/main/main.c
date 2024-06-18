/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/i2c.h"
#include "led_strip.h"
#include "messages.h"

#define LED_GPIO 48
#define ESP_SLAVE_BASE_ADDR 0x28             /*!< ESP32 slave address, you can set any 7bit value */

#define DATA_LENGTH 512              

static led_strip_handle_t led_strip;

static char *TAG = "SLV0";
static int i2c_slave_port = 0;
static uint16_t device_address;
uint8_t *receive_buffer;

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

#define I2C_SLAVE_SCL_IO 9               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 8               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM 0 /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

typedef struct {
    union {
        struct {
            uint8_t button1:    1;
            uint8_t button2:    1;
            uint8_t button3:    1;
            uint8_t button4:    1;
            uint8_t button5:    1;
            uint8_t button6:    1;
            uint8_t button7:    1;
            uint8_t button8:    1;
        };
        uint8_t val;
    } buttons;
    int8_t x_displacement;
    int8_t y_displacement;
    int8_t scroll_displacement;
} __attribute__((packed)) hid_mouse_input_report_boot_extended_t;

typedef struct {
    union {
        struct {
            uint8_t left_ctr:    1;
            uint8_t left_shift:  1;
            uint8_t left_alt:    1;
            uint8_t left_gui:    1;
            uint8_t rigth_ctr:   1;
            uint8_t right_shift: 1;
            uint8_t right_alt:   1;
            uint8_t right_gui:   1;
        };
        uint8_t val;
    } modifier;
    uint8_t reserved;
    uint8_t key[6];
} __attribute__((packed)) hid_keyboard_input_report_boot_t;

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

hid_keyboard_output_report_boot_t led_state = { 0 };

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "Generic",             // 1: Manufacturer
    "Generic Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Generic HID interface",  // 4: HID
};

#define SLAVE_IDX 0

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

void send_led_state() {
    // i2c_slave_write_buffer(i2c_slave_port, &led_state, sizeof(hid_keyboard_output_report_boot_t), 0);
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    ESP_LOGI(TAG, "SET_REPORT: %d, %d, %d", report_id, report_type, bufsize);
    if ( bufsize == 1 ) {
        hid_keyboard_output_report_boot_t *out = buffer;
        ESP_LOGI(TAG, "SET_REPORT data: num: %d, caps: %d, scroll: %d",
            out->led_state.num_lock,
            out->led_state.caps_lock,
            out->led_state.scroll_lock );
        led_state.led_state.num_lock = out->led_state.num_lock;
        led_state.led_state.caps_lock = out->led_state.caps_lock;
        led_state.led_state.scroll_lock = out->led_state.scroll_lock;
        send_led_state();
    }
}

void dispatch(uint8_t message_type, uint8_t *data, uint8_t len) {
    ESP_LOGD(TAG, "Recv: %d[%d]", message_type, len);
    if(!tud_mounted()) {
        if ( led_state.led_state.tud_mounted ) {
            led_state.led_state.tud_mounted = 0;
            send_led_state();
        }
        led_strip_set_pixel(led_strip, 0, 16, 0, 0);
        led_strip_refresh(led_strip);
        return;
    }
    if (!led_state.led_state.tud_mounted) {
        led_state.led_state.tud_mounted = 1;
        send_led_state();
    }
    switch (message_type) {
        case MESSAGE_TYPE_LED:
            LedMessage *ledMsg = (LedMessage*)data;
            led_strip_set_pixel(led_strip, 0, ledMsg->red, ledMsg->green, ledMsg->blue);
            led_strip_refresh(led_strip);
            break;
        case MESSAGE_TYPE_KB:
            hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t*)data;
            ESP_LOGI(TAG, "Keyboard: %d%d%d%d%d%d%d%d %2x %2x %2x %2x %2x %2x",     
                kb_report->modifier.left_ctr,
                kb_report->modifier.left_shift,
                kb_report->modifier.left_alt,
                kb_report->modifier.left_gui,
                kb_report->modifier.rigth_ctr,
                kb_report->modifier.right_shift,
                kb_report->modifier.right_alt,
                kb_report->modifier.right_gui,
                kb_report->key[0],
                kb_report->key[1],
                kb_report->key[2],
                kb_report->key[3],
                kb_report->key[4],
                kb_report->key[5]
            );
            while (!tud_hid_n_report(0, HID_ITF_PROTOCOL_KEYBOARD, data, len) ) {
                // No op, retrying
                // ESP_LOGE(TAG, "Error sending hid kb report");
            }
            break;
        case MESSAGE_TYPE_MOUSE:
            hid_mouse_input_report_boot_extended_t *mouse_report = (hid_mouse_input_report_boot_extended_t*)data;
            //tud_hid_n_report(0, HID_ITF_PROTOCOL_MOUSE, data, len);
            while (!tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 
                    mouse_report->buttons.val, 
                    mouse_report->x_displacement, 
                    mouse_report->y_displacement, 
                    mouse_report->scroll_displacement, 0)) {
                // No op, retrying
                // ESP_LOGE(TAG, "Error sending hid mouse report");
            }

            ESP_LOGD(TAG, "Mouse: %d,%d, wheel: %d buttons: %d %d %d %d %d %d %d %d",
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
            
            break;
    }
}

void device_app_main(void)
{
    led_strip_set_pixel(led_strip, 0, 16, 0, 16);
    led_strip_refresh(led_strip);

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    led_strip_set_pixel(led_strip, 0, 16, 16, 0);
    led_strip_refresh(led_strip);

    TAG[3] = 48 + SLAVE_IDX; // 48 == Ascii zero
    device_address = ESP_SLAVE_BASE_ADDR + SLAVE_IDX;
    ESP_LOGI(TAG, "Running as slave %d (0x%x)", SLAVE_IDX, device_address);
    
    //receive_queue = xQueueCreate( 10, sizeof(i2c_slave_rx_done_event_data_t) );

    // xTaskCreate(hid_device_main, "hid_device_main", 1024 * 2, NULL, 10, NULL);

    receive_buffer = (uint8_t*)malloc(I2C_SLAVE_RX_BUF_LEN);
    memset(receive_buffer, 0, I2C_SLAVE_RX_BUF_LEN);

    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,          // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,          // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = device_address,      // address of your project
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_slave_port, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, sizeof(hid_keyboard_output_report_boot_t), 0));
    send_led_state();

    ESP_LOGI(TAG, "Slave init successful");

    led_strip_set_pixel(led_strip, 0, 0, 16, 16);
    led_strip_refresh(led_strip);

    int read;
    uint8_t message_type;
    uint8_t message_length;
    while(1) {
        read = i2c_slave_read_buffer(i2c_slave_port, receive_buffer, 2, 10000 / portTICK_PERIOD_MS);
        if ( read == 0 ) {
            continue;
        }
        message_type = receive_buffer[0];
        message_length = receive_buffer[1];
        if (message_length > 0) {
            read = i2c_slave_read_buffer(i2c_slave_port, receive_buffer, message_length, 50 / portTICK_PERIOD_MS);
            send_led_state();
        }
        dispatch(message_type, receive_buffer, message_length);
        // ESP_LOGI(TAG, "Received %d, %d", message_type, message_length);
        memset(receive_buffer, 0, read);
    }
}


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

void app_main(void) {
    configure_led();
    device_app_main();
}