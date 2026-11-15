# sensor-probe

Firmware that turns an Arduino Nano 33 BLE into a probe for unknown lidar and
radar modules. It is a command shell over USB serial: scan I2C, sweep UART baud
rates and match known framing, dump raw bytes, read pins.

## Wiring

**3.3 V logic only. The nRF52840 pins are not 5 V tolerant.**

| Bus  | Nano pin            | Module pin |
|------|---------------------|------------|
| UART | D0 (RX)             | TX         |
| UART | D1 (TX)             | RX         |
| I2C  | A4 (SDA), A5 (SCL)  | SDA, SCL   |
| GND  | GND                 | GND        |

- The board has no pull-ups on A4/A5. Most breakout modules carry their own;
  if not, the firmware falls back to the nRF's weak internal ones (~13k), which
  is fine for short wires at 100 kHz.
- The Nano's `+5V` pin is **not connected** unless the VUSB pads on the
  underside are bridged. Out of the box the only supply is the 3.3 V pin.
  Modules that need 5 V or more want their own supply, with grounds joined.
- A module powered from 5 V usually still talks at 3.3 V, but check before
  wiring its TX to D0. If it transmits at 5 V, use a divider or level shifter.

## Use

    ~/.venvs/esptool/bin/pio run -t upload     # build + flash
    tools/probe.py help
    tools/probe.py uart level                  # is anything on RX at all?
    tools/probe.py --timeout 30 uart sweep     # 14 baud rates, known framing
    tools/probe.py i2c scan

Every reply ends with a line `ok` or `err: ...`. `probe.py` never opens the
port at 1200 baud, which on this board reboots it into the bootloader.

UART baud is written straight to the nRF BAUDRATE register, so any rate works,
including 256000 (the HLK LD24xx radar default), which the Arduino core cannot
set on its own.

## Recognised so far

UART framing: Benewake TFmini / TF-Luna / TF02 (checksum-verified), HLK
LD2410 / 2411 / 2412 / 2450, LDROBOT LD06 / LD19, Seeed MR24 / MR60, YDLIDAR,
RPLIDAR, Nooploop TOFSense, FF-framed ultrasonics, NMEA, plain ASCII.

I2C: an address table, plus ID-register checks that tell the ST ToF chips at
0x29 apart (VL53L0X, VL53L1X, VL53L4CD, VL53L3CX, VL6180X).
