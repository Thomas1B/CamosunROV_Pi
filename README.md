# CamosunROV_Pi — Testing UART to STM32F446.

Python script for testing UART communication between the Raspberry Pi (topside
control link) and the STM32F446 (Nucleo) on the ROV, used to remotely toggle
onboard/external LEDs as a sanity check before wiring up the full ROV control
link.

This is the Pi-side counterpart to the STM32 firmware in the
`CamosunROV_STM32` repo — packet format must match between the two.

## Overview

- `testingUART.py` opens the Pi's hardware UART (`/dev/serial0`) and sends
  `(ledId, state)` byte-pair packets to the STM32 over UART4.
- The STM32 receives via DMA (`HAL_UARTEx_ReceiveToIdle_DMA`) and toggles
  LEDs accordingly, confirming the link is alive.
- The script loops indefinitely, alternating LED states every 1 s, until
  interrupted (`Ctrl+C`), at which point it turns both LEDs off and closes
  the serial port cleanly.

## Requirements

- Raspberry Pi 4 with UART enabled via `raspi-config`:
  - **Interface Options → Serial Port**
    - "Login shell over serial" → **No** (disable serial console)
    - "Serial port hardware enabled" → **Yes**
- Python 3 with `pyserial` installed (see `ROV_Venv`)
- STM32F446 running the matching UART4 test firmware, connected:
  - Pi TX (GPIO14) → STM32 UART4 RX
  - Pi RX (GPIO15) → STM32 UART4 TX
  - Common GND between Pi and STM32

> **Note:** On the Pi 4, `/dev/serial0` maps to either the mini-UART or the
> PL011 UART depending on whether Bluetooth is enabled. If you get no
> response at all (rather than garbled bytes), check this mapping first.

## Usage

```bash
source ~/CamosunROV/ROV_Venv/bin/activate
python3 testingUART.py
```

The script will print each packet sent as a hex string, e.g.:

```
Sent: 0101
Sent: 0200
```

Press `Ctrl+C` to stop — both LEDs will be turned off and the port closed.

## Packet Format

Same format as the STM32 firmware expects: a sequence of `(ledId, state)`
byte pairs.

| Byte | Meaning |
|------|---------|
| 0    | LED ID |
| 1    | State (`0x01` = ON, `0x00` = OFF) |

**LED IDs used in this test:**

| ID     | Pin              |
|--------|------------------|
| `0x01` | `LED_Pin` (PC8)  |
| `0x02` | `LD2_Pin` (PA5, onboard LED) |

## Serial Settings

| Setting   | Value |
|-----------|-------|
| Port      | `/dev/serial0` |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity    | None |
| Stop bits | 1 |
| Timeout   | 1 s |
