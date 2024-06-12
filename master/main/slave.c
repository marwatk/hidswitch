#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usb/hid_usage_keyboard.h"
#include "sdkconfig.h"
#include "messages.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "main.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "slave.h"

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */

QueueHandle_t receive_queue;

uint8_t *receive_buffer;

static char *TAG = "SLV0";
static uint16_t device_address;

QueueHandle_t led_queue;



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

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

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

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

void hid_device_init(void)
{
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

    /*
    while (1) {
        if (tud_mounted()) {
            static bool send_hid_data = true;
            if (send_hid_data) {
                app_send_hid_demo();
            }
            send_hid_data = !gpio_get_level(APP_BUTTON);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    */
}


void dispatch(uint8_t message_type, uint8_t *data, uint8_t len) {
    switch (message_type) {
        case MESSAGE_TYPE_LED:
            if (xQueueSendToBack(led_queue, data, 0) != pdTRUE) {
                ESP_LOGE(TAG, "led queue full");
            }
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
            tud_hid_n_report(0, HID_ITF_PROTOCOL_KEYBOARD, data, len);
            break;
        case MESSAGE_TYPE_MOUSE:
            hid_mouse_input_report_boot_extended_t *mouse_report = (hid_mouse_input_report_boot_extended_t*)data;
            ESP_LOGI(TAG, "Mouse: %d,%d, wheel: %d buttons: %d %d %d %d %d %d %d %d",
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
            tud_hid_n_report(0, HID_ITF_PROTOCOL_MOUSE, data, len);
            break;
    }
}

void slave_main(uint8_t slave_idx, QueueHandle_t led_q) {
    TAG[3] = 48 + slave_idx; // 48 == Ascii zero
    device_address = ESP_SLAVE_BASE_ADDR + slave_idx;
    ESP_LOGI(TAG, "Running as slave %d (0x%x)", slave_idx, device_address);
    
    led_queue = led_q;
    //receive_queue = xQueueCreate( 10, sizeof(i2c_slave_rx_done_event_data_t) );

    hid_device_init();
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
    int i2c_slave_port = 0;

    ESP_ERROR_CHECK(i2c_param_config(i2c_slave_port, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
    
    ESP_LOGI(TAG, "Slave init successful");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

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
        }
        dispatch(message_type, receive_buffer, message_length);
        ESP_LOGI(TAG, "Received %d, %d", message_type, message_length);
        memset(receive_buffer, 0, read);
    }

    // xTaskCreate(slave_receiver_task, "slave_receiver", 1024 * 2, (void *)0, 10, NULL);
}