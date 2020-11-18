# decal-bath
Arduino based Heated Decal Bath control software

## Background
Application of water-slide decals on scalemodels goes a little bit easier if the water is warm.

This project is based on a FDM 3D printed frame and some electronics that controls PCB heatbed of approx 8 Ohm. An external 12VDC/2A power-plug is needed as well. The UI consist of a 0.96" OLED display with 128x64 pixels using the SSD1306 controller, two LEDs and three buttons.

## Design and other resources
The microcontroller used is a Arduino Pro Mini 5V 16MHz version.

I used the Atom editor with PlatformIO plugin to manage the source-code and it uses the following libraries:
EEPROM
OneWire
DS18B20
U8x8lib
PID_v1

The complete design with .stl-files for 3D printing, electronics design and PCB Gerber files can be downloaded from:
https://www.thingiverse.com/thing:4652993
