# mpy-module-mcp2515
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
Micropython external C module for MCP2515 (For high speed)  

MCP2515 : Stand Alone CAN Controller with SPI  
https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf

## Usage
* Micropython external module generation method  
  https://micropython-docs-ja.readthedocs.io/latest/develop/natmod.html#defining-a-native-module
* Build Example
```
#linux

git clone https://www.github.com/micropython/micropython
cd micropython/
git clone https://github.com/Wizapply/mpy-module-mcp2515 mcp2515
cd mcp2515
make
```

Example micropython(.py) code:
```python
import mcp2515
import time
from machine import Pin, SPI

sckPin = 6
mosiPin = 7
misoPin = 4
csPin = 5

spio = SPI(0, 8_000_000, sck=Pin(sckPin), mosi=Pin(mosiPin), miso=Pin(misoPin))
pin_cs = Pin(csPin, Pin.OUT)
if not mcp2515.can_init(spio,pin_cs,mcp2515.MCP20MHz,500000):
    print('mcp2515 error.')

mcp2515.can_filter(0x700,0x700,False)
mcp2515.can_mode(mcp2515.NormalMode)

while True:
    while mcp2515.can_recv_poling() > 0:
        data = mcp2515.can_read()
        if not data is None:
            mcp2515.can_send(0x666,data[1],0)
    time.sleep_ms(10)
```

## Performance
Repeat: Receive to Send  
CAN Config: 500000 bps Standard Frame  
Send Can Frame: ID:0x777 dlc:8 byte:0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  
|        | Micropython scripts Only | This Codes          | Performance     | 
| ------ | ------------------------ | ------------------- | --------------- | 
| RP2040 | 270~340 Messages/s | 1200~1400 Messages/s | x4.0~4.3 faster | 

## API Reference

import mcp2515

### can_init(spi:machine.SPI, cs_pin:machine.PIN, MCP_CLOCK:int, BAUDRATE:int)
Initialize mcp2515.  
*MCP_CLOCK  
 MCP20MHz  
 MCP16MHz  
 MCP8MHz  

### can_send(ID:int, DATA:array, OPTION:int)
*OPTION  
 OptErrorFrame  
 OptExtFrame  
 OptRemoteReq 

### can_mode(MODE:int)
*MODE  
 NormalMode  
 ListenMode  
 LoopbackMode  
 SleepMode  

### can_filter(MASK:int,ID:int,EX_OPTION:bool)

### can_recv_poling()

### can_read()
*return
 dataList = (can_id:int, data:byte_array(), opt:int)

# Library used
* https://github.com/micropython/micropython  
  License: https://github.com/micropython/micropython/blob/master/LICENSE
* https://github.com/MinaHBB/MCP2515  
  License: https://github.com/MinaHBB/MCP2515/blob/master/license.md  
  The reason I used this library is to generate native mpy using C language.
