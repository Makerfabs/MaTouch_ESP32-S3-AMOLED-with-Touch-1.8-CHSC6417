#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "CHSC6417.h"
#include <Wire.h>
#include "pin_config.h"

int16_t touch=0;
int16_t x=0,y=0,point_num=0;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
    LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RST /* RST */,
                                      0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT,
                                      16 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

void touch_interrupt_handler();

void setup()
{
    USBSerial.begin(115200);
    USBSerial.println("Drawing board demo");

    pinMode(LCD_EN, OUTPUT);
    digitalWrite(LCD_EN, HIGH);

    touch_init();
    attachInterrupt(digitalPinToInterrupt(TP_INT), touch_interrupt_handler, CHANGE);

    gfx->begin();
    gfx->fillScreen(WHITE);

    for (int i = 0; i <= 100; i++)  //0-255
    {
        gfx->Display_Brightness(i);
        gfx->setCursor(30, 150);
        gfx->setTextColor(BLUE);
        gfx->setTextSize(4);
        gfx->println("Drawing board");
        delay(3);
    }
    delay(500);
    gfx->fillScreen(WHITE);
    USBSerial.println("Setup done");
}

void loop()
{
  if(touch==1)
  {
    touch=0;
    read_touch();
    USBSerial.printf("Touch X:%d Y:%d\n",x,y);
    gfx->fillCircle(x,y,8,BLUE);
  }
}

void read_touch()
{
  uint8_t data1[1]={0xE0};
  uint8_t data2[3];

  i2c_write(TOUCH_I2C_ADD,TOUCH_I2C_W,data1,1);
  delay(1);
  i2c_read(TOUCH_I2C_ADD,TOUCH_I2C_R,data2,3);

  point_num = data2[0] & 0x03;
  
  x = (uint16_t)((((data2[0] & 0x40) >> 6) << 8) | data2[1]);
  y = (uint16_t)(((data2[0] & 0x80) >> 7) << 8) | data2[2];
}

void touch_interrupt_handler() 
{
  touch=1;
}

