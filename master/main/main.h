#define ESP_SLAVE_BASE_ADDR 0x28             /*!< ESP32 slave address, you can set any 7bit value */

#define NUM_SLAVES 3

typedef struct {
    QueueHandle_t mouse_queue;
    QueueHandle_t kb_queue;
} HidQueues;