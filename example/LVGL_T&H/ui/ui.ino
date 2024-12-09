#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "CHSC6417.h"
#include "pin_config.h"
#include <ui.h>
#include "DHT.h"

#define DHTPIN 17
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

float h,t,f;
static const uint16_t screenWidth  = 368;
static const uint16_t screenHeight = 448;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

int16_t press=0;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
    LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RST /* RST */,
                                      0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT,
                                      16 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

void touch_interrupt_handler();

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    #if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    #else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    #endif

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
  uint8_t data1[1]={0xE0};
  uint8_t data2[3];
  int16_t x=0,y=0,point_num=0;
  
  i2c_write(TOUCH_I2C_ADD,TOUCH_I2C_W,data1,1);
  delay(1);
  i2c_read(TOUCH_I2C_ADD,TOUCH_I2C_R,data2,3);

  point_num = data2[0] & 0x03;
  
  x = (uint16_t)((((data2[0] & 0x40) >> 6) << 8) | data2[1]);
  y = (uint16_t)(((data2[0] & 0x80) >> 7) << 8) | data2[2];

  if(point_num==1)
  {
      data->state = LV_INDEV_STATE_PR;
      
      /*Set the coordinates*/
      data->point.x = x;
      data->point.y = y;

      /*USBSerial.print( "Data x " );
      USBSerial.print( x );

      USBSerial.print( "Data y " );
      USBSerial.println( y );*/
  }
  else
  {   
    data->state = LV_INDEV_STATE_REL;
  }
}

void setup()
{
    USBSerial.begin( 115200 ); /* prepare for possible serial debug */

    pinMode(LCD_EN, OUTPUT);
    digitalWrite(LCD_EN, HIGH);

    touch_init();
    attachInterrupt(digitalPinToInterrupt(TP_INT), touch_interrupt_handler, CHANGE);

    gfx->begin();
    gfx->Display_Brightness(200);

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    USBSerial.println( LVGL_Arduino );
    USBSerial.println( "I am LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    //disp_drv.sw_rotate = 1;   // add for rotation
    //disp_drv.rotated = LV_DISP_ROT_90;   // add for rotation
    lv_indev_drv_register( &indev_drv );

    ui_init();
    dht.begin();    
    USBSerial.println( "Setup done" );
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    h = dht.readHumidity();
    t = dht.readTemperature();
    if (isnan(t))
    {
     USBSerial.println(F("Sensor not found"));
    }
    else
    {
      lv_arc_set_value(ui_Arc1, h);
      lv_obj_invalidate(ui_Arc1); // 强制刷新arc，确保界面更新
      _ui_arc_set_text_value(ui_Label3, ui_Arc1, "", "%");
      lv_arc_set_value(ui_Arc2, t);
      lv_obj_invalidate(ui_Arc2); // 强制刷新arc，确保界面更新
      _ui_arc_set_text_value(ui_Label4, ui_Arc2, "", "°");
    
      USBSerial.print(F("Humidity: "));
      USBSerial.print(h);
      USBSerial.print(F("%  Temperature: "));
      USBSerial.print(t);
      USBSerial.println(F("°C "));
    }
    delay(5);
}

void touch_interrupt_handler() 
{
  press=1;
}

