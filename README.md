# sensor-probe

**The Pico version in `pico/` is the one in use.** The Nano 33 BLE firmware
below was the first attempt; it still works but that board has one UART and no
5 V output.

## Pico (MicroPython)

The Pico runs stock MicroPython v1.29.0 with `pico/probe.py` copied on as
`/probe.py`. Everything is driven from the Python prompt, so there is no
reflash per experiment.

    M=~/.venvs/esptool/bin/mpremote
    $M cp pico/probe.py :probe.py                       # after editing
    $M exec "import probe; probe.level(0)"              # anything on UART0 RX?
    $M exec "import probe; probe.sweep(0)"              # 14 bauds, known framing
    $M exec "import probe; probe.mon(0, 115200, 2000)"  # hex, one line per burst
    $M exec "import probe; probe.scan()"                # I2C

This board is a USB-C Pico clone whose silkscreen shows **GPIO numbers**
(0, 1, GND, 2, 3 ...), not physical pin numbers. Everything below uses those.

| Port / bus            | Pads                 | Module pin      |
|-----------------------|----------------------|-----------------|
| port 0, hardware UART | 0 = TX, 1 = RX       | module TX -> RX |
| port 1, hardware UART | 8 = TX, 9 = RX       | module TX -> RX |
| port 2, software UART | 1 = TX, 2 = RX       | the LD2451 as soldered |
| I2C                   | 4 = SDA, 5 = SCL     |                 |
| 5 V out               | corner pad `Vout`    |                 |
| 3.3 V out             | pad `3V3`            |                 |

3.3 V logic only. The Pico has no I2C pull-ups of its own and its internal
ones are too weak to rely on.

Port 2 is a PIO UART. It exists because the radar is soldered with TX on pad 2
and RX on pad 1 (the wiring instructions gave physical pin numbers and the
board is labelled in GPIO numbers), and no hardware UART can use those pads.

## Modules on the bench

- **HLK-LD2451** radar: 5 V supply (>300 mA capacity, ~107 mA average), 3.3 V
  IO, 115200 8N1, per Hi-Link's user manual v1.0. Pins VIN GND OT1 TX RX OT2.
  Wired: VIN -> Vout, GND -> GND, TX -> pad 2, RX -> pad 1 (port 2). Read
  working 14 Nov: idle frames `F4 F3 F2 F1 00 00 F8 F7 F6 F5` every ~0.66 s;
  enable-config, firmware-version and end-config commands all acked.
  Settings changed 14 Nov (originals in brackets): direction 2 = both [1],
  signal threshold 4 [0]. Unchanged: max 100 m, min speed 1 km/h, delay 1 s,
  trigger count 2. Before the change two watches saw no target; after it,
  `probe.ld_watch()` reported a walker at 4-8 m, 3-7 km/h. Target fields:
  count, alarm, then per target angle (minus 0x80, degrees), distance (m),
  direction (1 = approaching), speed (km/h), signal strength.
  Chips, read from photos 14 Nov: radar front end marked `SKYRELAY RC2412`
  (lot P2CF0723202), no datasheet found; controller is a JieLi (`JL` logo,
  marked BP28842-5SC8, a lot code not a part number) with a 24 MHz crystal;
  firmware in an external PUYA P25D40SH 4 Mbit SPI flash (SOIC-8); 40 MHz
  crystal for the radar chip; an unpopulated 4-pad header on the back edge.
- **"Laser R20 D9/A" obstacle-avoidance lidar**, pins C D V G: toy-drone part,
  chip unidentified. Wired D -> pad 4, C -> pad 5. Answers on I2C at 0x52 and
  has its own pull-ups. Register 0 gives four bytes: a big-endian 16-bit value
  that looks like distance in mm (seen 264 to 4880, steady to about +/-10 when
  still), then `01 4A`, constant. Everything past that reads FF. The value
  changes faster than 40 Hz. `probe.r20()` polls it. Units not yet checked
  against a measured distance.
- **LC-307 rev 3** optical flow, marked 2V8: unidentified toy-drone part,
  no datasheet found. Supply voltage and baud unknown.

# Nano 33 BLE firmware

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
