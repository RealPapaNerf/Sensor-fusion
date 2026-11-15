# Sensor probe for a Raspberry Pi Pico running MicroPython.
#
# Lives on the Pico as /probe.py and is driven from the host through the
# Python prompt, so trying something new needs no reflash:
#
#     mpremote exec "import probe; probe.sweep()"
#
# Pins (3.3 V logic only; the Pico's GPIO is not 5 V tolerant):
#
# The numbers are GPIO numbers, which is what this board's silkscreen shows
# (it is a USB-C Pico clone labelled 0, 1, GND, 2, 3 ... not physical pins).
#
#     port 0  hardware UART0   GP0 = TX   GP1 = RX   <- module TX goes to RX
#     port 1  hardware UART1   GP8 = TX   GP9 = RX
#     port 2  software UART    GP1 = TX   GP2 = RX   <- the LD2451 as soldered
#     I2C0    GP4 = SDA  GP5 = SCL
#     5 V out = the corner pad marked Vout     3.3 V out = the pad marked 3V3
#
# Port 2 exists because the radar was soldered with TX on GP2 and RX on GP1,
# which no hardware UART can serve. It is a PIO UART, so it takes any pins.

import time
import rp2
from machine import I2C, UART, Pin

UART_PINS = {0: (0, 1), 1: (8, 9), 2: (1, 2)}  # port -> (tx, rx)
SOFT = (2,)
SDA, SCL = 4, 5

BAUDS = (9600, 19200, 38400, 57600, 115200, 128000, 230400,
         250000, 256000, 420000, 460800, 500000, 921600, 1000000)

_baud = {0: 115200, 1: 115200, 2: 115200}


@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_RIGHT, fifo_join=rp2.PIO.JOIN_RX)
def _pio_rx():
    wait(0, pin, 0)
    set(x, 7)[10]
    label("bit")
    in_(pins, 1)
    jmp(x_dec, "bit")[6]
    push(noblock)
    wait(1, pin, 0)


@rp2.asm_pio(sideset_init=rp2.PIO.OUT_HIGH, out_init=rp2.PIO.OUT_HIGH,
             out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _pio_tx():
    pull()
    set(x, 7).side(0)[7]
    label("bit")
    out(pins, 1)[6]
    jmp(x_dec, "bit")
    nop().side(1)[6]


class SoftUART:
    """PIO UART with the same read()/write() shape as machine.UART.

    The receive FIFO is 8 bytes deep, so read() must be polled in a tight
    loop (as _capture and mon do) or bytes are dropped.
    """

    def __init__(self, baud, tx, rx):
        rxp = Pin(rx, Pin.IN, Pin.PULL_UP)
        txp = Pin(tx, Pin.OUT, value=1)
        self.rx = rp2.StateMachine(0, _pio_rx, freq=8 * baud, in_base=rxp)
        self.tx = rp2.StateMachine(1, _pio_tx, freq=8 * baud, sideset_base=txp, out_base=txp)
        self.rx.active(1)
        self.tx.active(1)

    def read(self):
        n = self.rx.rx_fifo()
        if not n:
            return None
        get = self.rx.get
        out = bytearray()
        while self.rx.rx_fifo():
            out.append(get() >> 24)
        return out

    def write(self, data):
        for b in data:
            self.tx.put(b)


def _uart(u, baud=None):
    if baud:
        _baud[u] = baud
    tx, rx = UART_PINS[u]
    if u in SOFT:
        return SoftUART(_baud[u], tx, rx)
    uart = UART(u, baudrate=_baud[u], tx=Pin(tx), rx=Pin(rx), rxbuf=4096, timeout=0)
    # Hold an unconnected RX at the idle level; floating, it reads as stray FF bytes.
    Pin(rx, Pin.ALT, pull=Pin.PULL_UP, alt=Pin.ALT_UART)
    time.sleep_ms(3)
    uart.read()
    return uart


def _capture(uart, ms, cap=4096):
    buf = bytearray()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms and len(buf) < cap:
        d = uart.read()
        if d:
            buf.extend(d)
    return bytes(buf)


def _hex(b):
    return " ".join("%02X" % x for x in b)


# ------------------------------------------------------------ framing matchers

SIGS = (
    (b"\xF4\xF3\xF2\xF1", 1, "HLK LD24xx radar data frame (LD2410/2411/2412/2451)"),
    (b"\xFD\xFC\xFB\xFA", 1, "HLK LD24xx radar command/ack frame"),
    (b"\xAA\xFF\x03\x00", 1, "HLK LD2450 radar target frame"),
    (b"\x57\x00\xFF", 1, "Nooploop TOFSense frame"),
    (b"\x54\x2C", 3, "LDROBOT LD06/LD19/STL-19 lidar packet"),
    (b"\x53\x59", 3, "Seeed/MicRadar MR24/MR60 radar frame"),
    (b"\xAA\x55", 3, "YDLIDAR packet header"),
    (b"\xA5\x5A", 3, "RPLIDAR response descriptor"),
    (b"\xFE\x0A", 3, "UPixels UP-Flow optical flow frame (FE 0A ... 55)"),
    (b"$G", 3, "NMEA ($G...)"),
)


def _count(buf, pat):
    n = i = 0
    while True:
        i = buf.find(pat, i)
        if i < 0:
            return n
        n += 1
        i += len(pat)


def _benewake(buf):
    """59 59 distL distH strL strH tL tH checksum -> (good frames, dist, strength)."""
    good = dist = strength = 0
    i = 0
    while i + 9 <= len(buf):
        if buf[i] == 0x59 and buf[i + 1] == 0x59 and (sum(buf[i:i + 8]) & 0xFF) == buf[i + 8]:
            good += 1
            dist = buf[i + 2] | (buf[i + 3] << 8)
            strength = buf[i + 4] | (buf[i + 5] << 8)
            i += 9
        else:
            i += 1
    return good, dist, strength


def _ascii_pct(buf):
    if not buf:
        return 0
    a = sum(1 for c in buf if 0x20 <= c < 0x7F or c in (9, 10, 13))
    return a * 100 // len(buf)


def describe(buf):
    """Print what a capture looks like. Returns True if anything matched."""
    hit = False
    good, dist, strength = _benewake(buf)
    if good:
        print("    MATCH Benewake TFmini/TF-Luna/TF02: %d checksum-valid frames, last dist=%d strength=%d"
              % (good, dist, strength))
        hit = True
    for pat, need, name in SIGS:
        n = _count(buf, pat)
        if n >= need:
            print("    MATCH %s x%d" % (name, n))
            hit = True
    if len(buf) >= 8 and _ascii_pct(buf) >= 90:
        print("    MATCH ASCII text:", repr(bytes(buf[:96])))
        hit = True
    return hit


# ------------------------------------------------------------ UART

def level(u=0, ms=500):
    """Is anything wired to / talking on this UART's RX pin?"""
    p = Pin(UART_PINS[u][1], Pin.IN, Pin.PULL_DOWN)
    time.sleep_ms(2)
    highs = total = edges = 0
    last = p.value()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        v = p.value()
        if v != last:
            edges += 1
        last = v
        highs += v
        total += 1
    pct = highs * 100 // total
    print("UART%d RX (GP%d) over %d ms: high %d%% of samples, %d edges seen"
          % (u, UART_PINS[u][1], ms, pct, edges))
    if edges > 4:
        print("  -> something is transmitting")
    elif pct > 90:
        print("  -> held HIGH: a transmitter is connected but silent (may need a command)")
    else:
        print("  -> LOW: nothing connected, module unpowered, or TX/RX swapped")


def sweep(u=0, ms=500):
    """Listen at each baud rate and match known framing."""
    for baud in BAUDS:
        buf = _capture(_uart(u, baud), ms)
        line = "  %7d: %d bytes" % (baud, len(buf))
        if buf:
            line += ", ascii %d%%, head: %s" % (_ascii_pct(buf), _hex(buf[:20]))
        print(line)
        if buf:
            describe(buf)
    print("(no MATCH lines = nothing recognised)")


def mon(u=0, baud=None, ms=2000):
    """Hex dump, one line per burst (a gap of 3 ms or more ends a line)."""
    uart = _uart(u, baud)
    uart.read()
    t0 = time.ticks_ms()
    last = t0
    line = bytearray()
    start = 0
    total = 0
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        d = uart.read()
        now = time.ticks_ms()
        if d:
            if not line:
                start = time.ticks_diff(now, t0)
            line.extend(d)
            total += len(d)
            last = now
        elif line and time.ticks_diff(now, last) >= 3:
            print("%6d: %s" % (start, _hex(line)))
            line = bytearray()
    if line:
        print("%6d: %s" % (start, _hex(line)))
    print("(%d bytes at %d baud on UART%d)" % (total, _baud[u], u))


def grab(u=0, baud=None, ms=1000):
    """Capture and return raw bytes, for decoding on the spot."""
    uart = _uart(u, baud)
    uart.read()
    return _capture(uart, ms)


def decode(u=0, baud=None, ms=1000):
    buf = grab(u, baud, ms)
    print("%d bytes at %d baud" % (len(buf), _baud[u]))
    if not describe(buf):
        print("    nothing recognised")


def send(data, u=0, baud=None, ms=300):
    """Send bytes (bytes object or hex string), then show the reply."""
    if isinstance(data, str):
        data = bytes.fromhex(data.replace(" ", ""))
    uart = _uart(u, baud)
    uart.read()
    uart.write(data)
    buf = _capture(uart, ms)
    print("reply (%d bytes): %s" % (len(buf), _hex(buf)))
    return buf


# ------------------------------------------------------------ I2C

KNOWN = {
    0x08: "Nooploop TOFSense (I2C mode)",
    0x0D: "QMC5883L compass",
    0x10: "Benewake TF-Luna / TFmini-S / TFmini-Plus (I2C mode)",
    0x13: "VCNL4010 proximity",
    0x1E: "HMC5883L / LIS3MDL compass",
    0x29: "ST VL53L0X / VL53L1X / VL53L4CD / VL6180X ToF (or TSL2591, BNO055)",
    0x30: "VL53L0X/L1X moved address, MMC5603",
    0x39: "APDS-9960 gesture/proximity",
    0x3C: "SSD1306 OLED",
    0x40: "INA219 / HTU21 / PCA9685",
    0x41: "ams TMF8801/TMF882x ToF",
    0x48: "ADS1115 / TMP102",
    0x52: "Acconeer XM125 radar (I2C), EEPROM",
    0x57: "MAX3010x, EEPROM",
    0x62: "Garmin LIDAR-Lite v3/v4",
    0x66: "LightWare SF/LW lidar",
    0x68: "MPU-6050 / ICM-20x / DS3231",
    0x70: "MaxBotix I2CXL sonar / SRF0x / TCA9548 mux",
    0x76: "BMP280 / BME280 / MS5611",
    0x77: "BMP280 / BMP388 / MS5611",
}


def _pullups():
    """A pull-up of ~10k or stronger wins against the internal pull-down."""
    out = []
    for gp in (SDA, SCL):
        p = Pin(gp, Pin.IN, Pin.PULL_DOWN)
        time.sleep_ms(2)
        out.append(p.value() == 1)
    return out


def i2c(freq=100000):
    return I2C(0, sda=Pin(SDA), scl=Pin(SCL), freq=freq, timeout=50000)


def rd(addr, reg, n=1, wide=False, bus=None):
    """Read n bytes from an 8-bit (or 16-bit, wide=True) register."""
    bus = bus or i2c()
    return bus.readfrom_mem(addr, reg, n, addrsize=16 if wide else 8)


def wr(addr, reg, data, wide=False, bus=None):
    bus = bus or i2c()
    bus.writeto_mem(addr, reg, bytes(data), addrsize=16 if wide else 8)


def _st_tof(bus, addr):
    """The ST ToF family all sit at 0x29; ID registers tell them apart."""
    try:
        b = rd(addr, 0x010F, 2, wide=True, bus=bus)
        name = {0xEACC: "VL53L1X", 0xEBAA: "VL53L4CD", 0xEAAA: "VL53L3CX",
                0xF002: "VL53L5CX/L7CX family"}.get((b[0] << 8) | b[1])
        if name:
            return "ID reg 0x010F = %02X%02X -> %s" % (b[0], b[1], name)
    except OSError:
        pass
    try:
        if rd(addr, 0xC0, 1, bus=bus)[0] == 0xEE:
            return "ID reg 0xC0 = EE -> VL53L0X"
    except OSError:
        pass
    try:
        if rd(addr, 0x0000, 1, wide=True, bus=bus)[0] == 0xB4:
            return "ID reg 0x0000 = B4 -> VL6180X"
    except OSError:
        pass
    return "no ST ToF ID matched"


def scan():
    sda, scl = _pullups()
    print("external pull-ups: SDA(GP%d) %s, SCL(GP%d) %s"
          % (SDA, "yes" if sda else "NO", SCL, "yes" if scl else "NO"))
    if not (sda and scl):
        print("  the Pico's internal pull-ups are very weak (~50k+); results may be unreliable")
    bus = i2c()
    found = bus.scan()
    for a in found:
        print("  0x%02X  %s" % (a, "maybe: " + KNOWN[a] if a in KNOWN else "(not in my table)"))
        if a in (0x29, 0x30):
            print("      " + _st_tof(bus, a))
    print("%d device(s) answered" % len(found))
    if len(found) > 60:
        print("  (nearly every address answered: SDA is probably stuck or shorted)")
    return found


def dump(addr, start=0, n=64, wide=False):
    """Hex dump a run of registers, 16 per line."""
    bus = i2c()
    for off in range(start, start + n, 16):
        try:
            print("  %04X: %s" % (off, _hex(rd(addr, off, min(16, start + n - off), wide, bus))))
        except OSError as e:
            print("  %04X: read failed (%s)" % (off, e))
            break
