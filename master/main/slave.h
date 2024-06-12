#define I2C_SLAVE_SCL_IO 9               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 8               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM 0 /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

void slave_main(uint8_t slave_idx, QueueHandle_t led_q);