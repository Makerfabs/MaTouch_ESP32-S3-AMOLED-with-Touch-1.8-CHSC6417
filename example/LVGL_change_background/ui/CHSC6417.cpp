#include "CHSC6417.h"
#include <Wire.h>

void touch_init(void)
{
    Wire.begin(IIC_SDA,IIC_SCL);
    pinMode(TP_RST, OUTPUT);
    pinMode(TP_INT, INPUT);
    
    digitalWrite(TP_RST, HIGH);
    delay(30);
    digitalWrite(TP_RST, LOW);
    delay(30);
    digitalWrite(TP_RST, HIGH);
    delay(30);
}

int i2c_read(uint16_t addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(true))
        return -1;
    Wire.requestFrom(addr, length, true);
    for (int i = 0; i < length; i++)
    {
        *reg_data++ = Wire.read();
    }
    return 0;
}

int i2c_write(uint8_t addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length)
{
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    for (int i = 0; i < length; i++)
    {
        Wire.write(*reg_data++);
    }
    if (Wire.endTransmission(true))
        return -1;
    return 0;
}


