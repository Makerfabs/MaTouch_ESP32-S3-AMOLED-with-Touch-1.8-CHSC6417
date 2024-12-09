#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "CHSC6417.h"
#include <Wire.h>
#include "pin_config.h"
#include "Material_16Bit_368x448px.h"

static uint8_t Image_Flag = 0;
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
    USBSerial.println("Touch demo");

    pinMode(LCD_EN, OUTPUT);
    digitalWrite(LCD_EN, HIGH);

    touch_init();
    attachInterrupt(digitalPinToInterrupt(TP_INT), touch_interrupt_handler, CHANGE);

    gfx->begin();
    gfx->fillScreen(WHITE);

    for (int i = 0; i <= 200; i++)
    {
        gfx->Display_Brightness(i);
        delay(5);
    }

    gfx->fillScreen(RED);
    delay(1000);
    gfx->fillScreen(GREEN);
    delay(1000);
    gfx->fillScreen(BLUE);
    delay(1000);

    gfx->fillScreen(WHITE);
    gfx->setCursor(30, 100);
    gfx->setTextSize(6);
    gfx->setTextColor(BLACK);
    gfx->printf("Makerfabs");
    gfx->setCursor(50, 200);
    gfx->setTextSize(6);
    gfx->setTextColor(BLACK);
    gfx->printf("Touch me");

    USBSerial.println("Setup OK");
    
    delay(1000);

}

void loop()
{
    if (touch == 1)
    {
        touch = 0;
        read_touch();

        if (point_num > 0)
        {
            switch (Image_Flag)
            {
            case 0:
                gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_1, LCD_WIDTH, LCD_HEIGHT); // RGB
                break;
            case 1:
                gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_2, LCD_WIDTH, LCD_HEIGHT); // RGB
                break;
            case 2:
                gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_3, LCD_WIDTH, LCD_HEIGHT); // RGB
                break;
            case 3:
                gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_4, LCD_WIDTH, LCD_HEIGHT); // RGB
                break;
            case 4:
                gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_5, LCD_WIDTH, LCD_HEIGHT); // RGB
                break;
            default:
                break;
            }

            Image_Flag++;

            if (Image_Flag > 4)
            {
                Image_Flag = 0;
            }
        }
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
