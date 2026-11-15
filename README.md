# sensor-fusion

**Live page: https://realpapanerf.github.io/Sensor-fusion/** — open it in Chrome
on an Android phone (or a desktop), plug the Pico in with a USB-C data cable,
and tap **Connect USB**. The phone powers the rig and reads it directly; no
laptop is needed. The Pico must already hold `probe.py` and `fusion.py`
(running the desktop app once puts them there). **Record** saves every sample
to a text file, and **GPS speed** shows and logs the phone's speed so radar
reports can be compared with how fast the rig itself is moving.

**Camera** shows the phone's rear camera with each radar contact drawn on it
(across by bearing, higher up the picture the farther away) plus a clock and
GPS speed. With the camera on, **Record** also saves that view as a video, so
contacts can be matched to what was actually there; every frame carries the
same clock that stamps the log. **Mark** drops a numbered marker into both.
Mount the phone so its camera looks the same way as the radar.

What follows is the bench history: the probe that identified the sensors, then
the fusion display built on it.

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

## Fusion display

    python3 fusion/server.py        # then open http://localhost:5187

`pico/fusion.py` streams all three sensors at 30 samples a second (the radar
and flow sensor each through a PIO receiver drained by DMA into a ring
buffer, so nothing is lost while the loop reads the lidar and prints).
`fusion/server.py` relays the stream to the browser; `docs/index.html` does
the fusion and drawing. All three sensors should face the same way.

What it combines:

- **Radar + lidar.** A radar mover within 8 deg of dead ahead whose range is
  within 1 m of the lidar reading is taken to be what the lidar is hitting.
  Its range becomes the lidar's (centimetres instead of whole metres) and its
  closing speed is shown from both sensors.
- **Lidar without radar.** A lidar return with no mover there is a surface
  that is standing still, which the radar cannot see at all.
- **Flow x lidar, rig held still.** Image motion (rad/s) times lidar distance
  is the sideways speed of whatever fills the view, drawn as the across-beam
  part of the velocity arrow.
- **Flow as heading, rig being turned.** Image motion is integrated into a
  heading, and each static lidar hit is placed on a map that stays put, so
  panning the rig by hand sweeps out the room. "Calibrate turn" learns which
  way the flow camera sees a right turn.

Limits: the flow camera cannot tell the rig turning from the scene moving, so
the two modes are a switch, not something it works out; heading drifts; radar
left/right sign is unverified (there is a flip button).

## Drive test, 14 Nov 2026

Rig held out of a car window, phone reading the Pico over USB from the Pages
site, 283 s recorded at 30 samples/s with no gaps. GPS speed was not switched
on, so there is no independent speed to compare against.

- The radar reports stationary scenery as approaching targets. Of 212 frames
  with two or more approaching targets, 154 gave every target the identical
  speed and 195 were within 1 km/h, while those targets sat a median 46 m
  apart in range. Separate objects at different distances all closing at one
  speed is what a fixed world looks like from a moving car.
- That shared speed traces a plausible drive: near zero for the first minute,
  then 15 to 50 km/h with stops.
- Range: approaching reports out to 99 m (the radar's limit is set to 100),
  median 46 m; 307 of 1,112 reports were beyond 75 m.
- From the driver afterwards: it picked up a row of parked cars but not a
  brick wall or houses. Metal vehicles throw a strong echo back; flat walls
  mostly reflect it away unless met square-on.
- 463 of 1,112 approaching reports gave a bearing of exactly 0, and the rest
  lean negative, so bearing on scenery looks unreliable.
- The lidar read its out-of-range value (20000) for 6,155 of 8,589 samples.
- Bug found: with three or more targets the radar's count and length fields
  can overstate what the frame carries. The Pico parser trusted them and read
  the trailer and next header as targets (183 bogus records in this log). It
  now cuts frames at the trailer. `tools/analyze_log.py` drops the bogus
  records when reading older logs.

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
  changes faster than 40 Hz. `probe.r20()` polls it. Powered from the 3V3 pad.
  Units are mm: held 36 in (914 mm) from a wall, 100 readings gave median 930,
  range 918-942.
- **LC-307 rev 3** optical flow (UPixels; main chip marked `upixels U30`).
  Pads GND TX RX VCC. Powered from 3V3. Module TX landed on pad 8 and RX on
  pad 9, so it is read on software port 3. Silent after power-up until the
  host sends the vendor start-up sequence (`probe.up_init()`, 161 camera
  register writes at 19200 baud, taken from ArduPilot PR #29729). It then
  streams 14-byte frames at 50 Hz: `FE 0A` x(16, LE) y(16, LE) dt-us(16, LE)
  `E7 03` quality version xor `55`. `probe.up_watch()` decodes them. The
  init has to be resent after every power cycle.
  Scale check 14 Nov: hand-slid about 50 cm at 20-25 cm height. The two
  cleanest strokes summed to 20,400 and 20,900 counts, i.e. 0.96e-4 to
  1.2e-4 rad per count, which brackets ArduPilot's 1e-4. Return strokes read
  lower (7,500 to 16,400) with quality dipping to 117-186, so fast motion
  undercounts. Hand-held, not a calibration.

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
