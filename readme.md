MaTouch_ESP32-S3 AMOLED with Touch 1.8 CHSC6417
```c++
/*
Version:	    V1.0
Author:		    Yuki
Create Date:	2024/12/10
Note:
*/
```

# Makerfabs

[Makerfabs home page](https://www.makerfabs.com/)

[Makerfabs Wiki](https://wiki.makerfabs.com/)

## Intruduce

Product Link:[MaTouch_ESP32-S3 AMOLED with Touch 1.8 CHSC6417]()

Wiki Link : [MaTouch_ESP32-S3 AMOLED with Touch 1.8 CHSC6417](https://wiki.makerfabs.com/MaTouch%20ESP32-S3%20AMOLED%20with%20Touch%201.8%27%27%20CHSC6417.html)


## Features

Specs:
- Controller: ESP32-S3(R8)
- Wireless: WiFi& Bluetooth 5.0 
- Resolution: 1.8”, 368*448 
- Display Driver IC : CO5300
- Display Interface: QSPI
- Color Mode: Full Color (16.7M color)
- Touch Driver IC : CHSC6417
- Touch Interface: IIC(0x2E)
- Input: USB Type-C 5V(native USB)
- Input Voltage: 3.7V lion battery
- Output: 2X12 1.27mm Header
- Buttons: Reset, Boot and User button
- LED: WS2812
- Arduino support: Yes
- LVGL support: Yes
- High definition display


## Example

### GFX_CHSC6417_Image

This demo detects the display effect and touch response of the screen, demonstrating the high-definition display effect of the MAOLED screen.

### LVGL_change_background

This demo is based on the functionality of the LVGL buttons, which allows you to change the background color by pressing the buttons.

### LVGL_Animation

This demo is based on the LVGL animation feature. When you touch the ball, it will start bouncing and at the same time the touch coordinates will be displayed on the screen. There is also an adjustable slider for you to use.

### LVGL_T&H

This demo is a temperature and humidity detection system designed by combining Squareline Studio and Arduino. First, we use Squareline to design the UI interface, then output the code to Arduino for programming, then add the DHT11 sensor to display the detected temperature and humidity to the screen.

### Drawing_board 

This demo is a simple drawing board that allows you to draw your patterns in it.

