#define ESP_SLAVE_BASE_ADDR 0x28             /*!< ESP32 slave address, you can set any 7bit value */

#define NUM_SLAVES 3

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
    QueueHandle_t mouse_queue;
    QueueHandle_t kb_queue;
} HidQueues;