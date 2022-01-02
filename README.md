# Arduino_Nano_SSD1283A_CAN_e_Paramotor_Display

This Arduino (atmega328p) code drives  an SSD1283A LCD screen to displays data from CAN_Bus

![Paramotor_Display](https://chrisdrake.com/img/Paramotor_Display.jpg)


# Supported hardware

1. Only these 1.6" "SSD1283" screens:  https://www.aliexpress.com/wholesale?SearchText=SSD1283
   ![SSD2183 LCD Front](https://chrisdrake.com/img/lcd1.jpg) ![SSD2183 LCD Back](https://chrisdrake.com/img/lcd2.jpg)
2. A 5v Arduino Nano: https://en.wikipedia.org/wiki/Arduino_Nano (e.g. atmega328p chips at 5v and 16mhz)
   ![Arduino Nano_5v](https://chrisdrake.com/img/Arduino_Nano.jpg)
3. Cheap CAN_Bus sheild
   ![Chinese CAN-BUS Adapter](https://chrisdrake.com/img/chinese_CAN.jpg)

*. Here is a 3D-printed case to put it in: https://a360.co/3JJTYie
   ![Instrument_Box_CAD](https://chrisdrake.com/img/Instrument_Box_CAD.png)

# DEPENDENCIES

  Uses this CAN Bus library: https://github.com/coryjfowler/MCP_CAN_lib
    => to install: cd Arduino/libraries; git clone https://github.com/coryjfowler/MCP_CAN_lib

  With this LCD driver: https://github.com/ZinggJM/SSD1283A
    => to install: cd Arduino/libraries; git clone https://github.com/ZinggJM/SSD1283A

  And the SerialID library:
   => to install: cd Arduino/libraries; git clone https://github.com/gitcnd/SerialID.git 

  Plus the included and modified custom LCDWIKI_GUI library (has custom font chars for the screens)

# NOTICE

We are not affiliated in any way whatsoever with any hardware suppliers.  This is no commercial relationship or benefit involved at all with this code.

