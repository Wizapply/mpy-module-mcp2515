# mpy-module-mcp2515

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**A high-performance MicroPython external native C module for the MCP2515 SPI CAN controller.**

Driving the MCP2515 from pure Python over SPI is often too slow for busy CAN buses. This module implements the protocol in C and exposes it to MicroPython as a native module (`.mpy`), giving you several times the throughput of a pure-Python driver while keeping a simple Python API.

---

## Table of Contents

- [Features](#features)
- [About the MCP2515](#about-the-mcp2515)
- [Requirements](#requirements)
- [Building](#building)
- [Example](#example)
- [Performance](#performance)
- [API Reference](#api-reference)
- [Libraries Used](#libraries-used)
- [License](#license)

---

## Features

- Runs as a MicroPython **native module** (`natmod`) — no firmware rebuild required
- Roughly **4× faster** than a pure-Python driver (measured on RP2040)
- Supports **standard**, **extended**, and **remote** frames
- Hardware **acceptance filter / mask** configuration
- Switchable operating modes: **Normal / Listen-only / Loopback / Sleep**

---

## About the MCP2515

The MCP2515 is a stand-alone CAN controller from Microchip that talks to a host MCU over SPI. It lets you add CAN connectivity to any board — even those without a built-in CAN peripheral.

- Datasheet: <https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf>

---

## Requirements

| Item | Notes |
| --- | --- |
| Target MCU | Any board running MicroPython (verified on RP2040) |
| Build host | Linux (WSL is fine) |
| MCP2515 crystal | 20 MHz, 16 MHz, or 8 MHz |
| Wiring | SPI — SCK / MOSI / MISO / CS |

---

## Building

This module is distributed as a MicroPython **native module**. See the official docs for background:

- <https://docs.micropython.org/en/latest/develop/natmod.html#defining-a-native-module>

### Steps (Linux)

```bash
# 1. Grab the MicroPython source
git clone https://www.github.com/micropython/micropython
cd micropython/

# 2. Clone this repo into a subdirectory named "mcp2515"
git clone https://github.com/Wizapply/mpy-module-mcp2515 mcp2515
cd mcp2515

# 3. Build
make
```

`make` produces an `.mpy` file. Copy it to your target board and import it as shown below.

---

## Example

The snippet below initializes the MCP2515, filters for IDs whose top three bits are `0b111`, and echoes every received frame back with ID `0x666`.

```python
import mcp2515
import time
from machine import Pin, SPI

# --- Pin assignments (RP2040 example) ---
sck_pin  = 6
mosi_pin = 7
miso_pin = 4
cs_pin   = 5

# --- SPI and CS setup ---
spi = SPI(0, 8_000_000,
          sck=Pin(sck_pin),
          mosi=Pin(mosi_pin),
          miso=Pin(miso_pin))
cs = Pin(cs_pin, Pin.OUT)

# --- Initialize the MCP2515 (20 MHz crystal, 500 kbps) ---
if not mcp2515.can_init(spi, cs, mcp2515.MCP20MHz, 500000):
    print('mcp2515 error.')

# --- Only accept frames whose ID starts with 0x7xx ---
mcp2515.can_filter(0x700, 0x700, False)

# --- Enter normal (send + receive) mode ---
mcp2515.can_mode(mcp2515.NormalMode)

# --- Main loop: receive, then echo back ---
while True:
    while mcp2515.can_recv_polling() > 0:
        frame = mcp2515.can_read()
        if frame is not None:
            # frame = (can_id, data_bytes, option)
            mcp2515.can_send(0x666, frame[1], 0)
    time.sleep_ms(10)
```

### What the code does

1. Set up an SPI bus and a CS output pin.
2. Call `can_init()` with the crystal frequency and desired bitrate.
3. (Optional) Narrow down the incoming traffic with `can_filter()`.
4. Enter the desired operating mode with `can_mode()`.
5. Poll with `can_recv_poling()` and pull frames out with `can_read()`.
6. Send frames with `can_send()` as needed.

---

## Performance

Throughput test: every received frame is immediately echoed back.

- **CAN config:** 500 kbps, standard frame
- **Test frame:** ID=`0x777`, DLC=8, data=`0x00 × 8`

| Target | Pure-Python driver | This module | Speed-up |
| --- | --- | --- | --- |
| RP2040 | 270–340 msg/s | 1200–1400 msg/s | **~4.0–4.3×** |

---

## API Reference

```python
import mcp2515
```

### `can_init(spi, cs_pin, mcp_clock, baudrate) -> bool`

Initialize the MCP2515. Returns `True` on success.

| Parameter | Type | Description |
| --- | --- | --- |
| `spi` | `machine.SPI` | An already-initialized SPI bus |
| `cs_pin` | `machine.Pin` | Output pin used for chip-select |
| `mcp_clock` | `int` | Crystal frequency attached to the MCP2515 |
| `baudrate` | `int` | CAN bitrate in bps, e.g. `500000` |

**Valid `mcp_clock` constants**

| Constant | Meaning |
| --- | --- |
| `mcp2515.MCP20MHz` | 20 MHz |
| `mcp2515.MCP16MHz` | 16 MHz |
| `mcp2515.MCP8MHz`  | 8 MHz  |

---

### `can_mode(mode)`

Switch the operating mode of the controller.

| Constant | Meaning |
| --- | --- |
| `mcp2515.NormalMode`   | Normal operation (send and receive) |
| `mcp2515.ListenMode`   | Listen-only — receives without ACKing |
| `mcp2515.LoopbackMode` | Loopback — useful for self-test |
| `mcp2515.SleepMode`    | Low-power sleep |

---

### `can_filter(mask, id, ex_option)`

Configure the hardware acceptance filter.

| Parameter | Type | Description |
| --- | --- | --- |
| `mask` | `int` | Filter mask — only bits set to `1` are compared |
| `id` | `int` | ID value to match against |
| `ex_option` | `bool` | `True` to filter on extended (29-bit) IDs |

> Example: `can_filter(0x700, 0x700, False)` accepts only frames whose top 3 bits are `0b111`.

---

### `can_send(id, data, option) -> bool`

Transmit a CAN frame.

| Parameter | Type | Description |
| --- | --- | --- |
| `id` | `int` | CAN ID to send |
| `data` | `bytes` / `bytearray` | Payload, up to 8 bytes |
| `option` | `int` | Bitwise OR of the flags below; `0` for a normal frame |

**Flags**

| Constant | Meaning |
| --- | --- |
| `mcp2515.OptErrorFrame` | Treat as an error frame |
| `mcp2515.OptExtFrame`   | Send as an extended (29-bit ID) frame |
| `mcp2515.OptRemoteReq`  | Send as a remote request (RTR) frame |

---

### `can_recv_poling() -> int`

Return the number of frames currently waiting in the receive buffer. Call `can_read()` while this value is greater than zero.

> Note: the spelling `poling` (rather than `polling`) is preserved from the original module.

---

### `can_read() -> tuple | None`

Pop one frame from the receive buffer. Returns `None` when the buffer is empty.

The returned tuple has the form:

```python
(can_id, data, option)
# can_id : int         ID of the received frame
# data   : bytearray   payload bytes
# option : int         flags describing the frame type
```

---

## Libraries Used

This module is built on top of the following projects:

- [micropython/micropython](https://github.com/micropython/micropython)
  License: [MIT](https://github.com/micropython/micropython/blob/master/LICENSE)
- [MinaHBB/MCP2515](https://github.com/MinaHBB/MCP2515)
  License: [see repo](https://github.com/MinaHBB/MCP2515/blob/master/license.md)
  Used as the basis for the C-level MCP2515 driver so that a native `.mpy` can be generated.

---

## License

MIT License
