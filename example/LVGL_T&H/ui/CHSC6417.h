#include <Wire.h>

#define TOUCH_I2C_ADD 0x2E
#define TOUCH_I2C_W 0x5C
#define TOUCH_I2C_R 0x5D
// IIC
#define IIC_SDA 3
#define IIC_SCL 4

// TOUCH
#define TP_INT 1
#define TP_RST 2

void touch_init(void);
int i2c_write(uint8_t addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length);
int i2c_read(uint16_t addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length);



