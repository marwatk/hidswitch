
#define MESSAGE_TYPE_LED 1
#define MESSAGE_TYPE_KB 2
#define MESSAGE_TYPE_MOUSE 3
#define MESSAGE_TYPE_KB_LEDS 4

#define MAX_DATA_LEN 10

typedef struct LedMessage LedMessage;
typedef struct Message Message;

struct LedMessage {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct Message {
    uint8_t type;
    uint8_t slave;
    uint8_t len;
    uint8_t data[MAX_DATA_LEN];
};

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
    uint8_t checksum;
} __attribute__((packed)) hid_keyboard_output_report_boot_t;

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
