# mpy-module-mcp2515
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
Micropython external C module for MCP2515 (For high speed)  

MCP2515 : Stand Alone CAN Controller with SPI
https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf

## Usage
* Micropython external module generation method  
  https://micropython-docs-ja.readthedocs.io/ja/latest/develop/natmod.html#defining-a-native-module
* If you are using RP2040, copy the already built output_module mpy file to your device.

Example micropython(.py) code:
```python
import mcp2515
from machine import Pin, SPI

sckPin = 6
mosiPin = 7
misoPin = 4
csPin = 5

spio = SPI(0, 8_000_000, sck=Pin(sckPin), mosi=Pin(mosiPin), miso=Pin(misoPin))
pin_cs = Pin(csPin, Pin.OUT)
if not mcp2515.can_init(spio,pin_cs,500000):
    print('mcp2515 error.')

while True:
    mcp2515.can_recv_poling()
    data = mcp2515.can_read()
    if not data is None:
        mcp2515.can_send(0x666,data[1],0)
```

## Performance
Repeat: Receive to Send  
Send Can Frame: StandardID:0x777 dlc:8 byte:0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  
|        | Micropython scripts Only | This Codes          | Performance     | 
| ------ | ------------------------ | ------------------- | --------------- | 
| RP2040 | 270~340 Message/s        | 1200~1400 Message/s | x4.0~4.3 faster | 

# Library used
* https://github.com/micropython/micropython  
  License: https://github.com/micropython/micropython/blob/master/LICENSE
* https://github.com/MinaHBB/MCP2515  
  License: https://github.com/MinaHBB/MCP2515/blob/master/license.md
