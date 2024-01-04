# mpy-module-mcp2515
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
Micropython external C module for MCP2515 (For high speed)  

## Usage
* Micropython external module generation method  
  https://micropython-docs-ja.readthedocs.io/ja/latest/develop/natmod.html#defining-a-native-module
* If you are using RP2040, copy the already built output_module mpy file to your device.

Example micropthon code:
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

# Library used
* https://github.com/micropython/micropython  
  License: https://github.com/micropython/micropython/blob/master/LICENSE
* https://github.com/MinaHBB/MCP2515  
  License: https://github.com/MinaHBB/MCP2515/blob/master/license.md
