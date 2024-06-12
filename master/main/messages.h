
#define MESSAGE_TYPE_LED 1
#define MESSAGE_TYPE_KB 2
#define MESSAGE_TYPE_MOUSE 3

#define MAX_DATA_LEN 10

typedef struct LedMessage LedMessage;
typedef struct Message Message;
typedef struct KeyboardReport KeyboardReport;

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