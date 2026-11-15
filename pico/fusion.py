# Streams all three sensors at once for the fusion display.
#
#     mpremote exec "import fusion; fusion.stream()"
#
# One text line per tick:
#
#     D,<ms>,<lidar mm>,<flow dx>,<flow dy>,<flow frames>,<flow quality>,<radar>
#
# <radar> is -1 when no radar frame arrived this tick, otherwise the number of
# targets followed by five fields per target: angle (deg), distance (m),
# direction (1 = approaching), speed (km/h), signal strength.
#
# The radar (115200) and flow sensor (19200) are both soldered to pads no
# hardware UART can serve, so each is received by a PIO state machine. A PIO
# FIFO holds only 8 bytes, far too few to survive a tick of I2C and printing,
# so a DMA channel drains each FIFO into a 4 KB ring buffer in the background.

import time
import rp2
import uctypes
from machine import I2C, Pin

import probe

_PIO0 = 0x50200000
RADAR_RX, FLOW_RX, FLOW_TX = 2, 8, 9
LIDAR_ADDR = 0x52


class RingRx:
    """PIO UART receiver drained by DMA into a ring buffer."""

    def __init__(self, sm_id, pin, baud, bits=12):
        self.size = 1 << bits
        # The DMA ring wraps on address bits, so the buffer must sit on a
        # size-aligned address: over-allocate and use the aligned part.
        self._raw = bytearray(self.size * 2)
        addr = uctypes.addressof(self._raw)
        self.base = (addr + self.size - 1) & ~(self.size - 1)
        self.buf = uctypes.bytearray_at(self.base, self.size)
        self.sm = rp2.StateMachine(sm_id, probe._pio_rx, freq=8 * baud,
                                   in_base=Pin(pin, Pin.IN, Pin.PULL_UP))
        self.dma = rp2.DMA()
        ctrl = self.dma.pack_ctrl(size=0, inc_read=False, inc_write=True,
                                  ring_size=bits, ring_sel=True, treq_sel=4 + sm_id)
        # +3: the received byte is the top byte of the 32-bit FIFO word.
        self.dma.config(read=_PIO0 + 0x20 + 4 * sm_id + 3, write=self.base,
                        count=0xFFFFFFFF, ctrl=ctrl, trigger=True)
        self.sm.active(1)
        self.tail = 0

    def read(self):
        head = (self.dma.write - self.base) & (self.size - 1)
        tail = self.tail
        if head == tail:
            return b""
        self.tail = head
        if head > tail:
            return bytes(self.buf[tail:head])
        return bytes(self.buf[tail:]) + bytes(self.buf[:head])

    def close(self):
        self.sm.active(0)
        self.dma.active(0)
        self.dma.close()


def _flow_init():
    """Send the UPixels start-up sequence (see probe.up_init)."""
    pin = Pin(FLOW_TX, Pin.OUT, value=1)
    tx = rp2.StateMachine(3, probe._pio_tx, freq=8 * 19200, sideset_base=pin, out_base=pin)
    tx.active(1)
    time.sleep_ms(5)

    def send(data):
        for b in data:
            tx.put(b)
        time.sleep_ms(2 + len(data) // 2)

    send(b"\xAA")
    x = 0
    for b in probe._UP_PARA:
        x ^= b
    send(b"\xAB" + probe._UP_PARA + bytes([x]))
    cfg = probe._UP_CFG
    for i in range(0, len(cfg), 2):
        send(bytes([0xBB, 0xDC, cfg[i], cfg[i + 1], 0xDC ^ cfg[i] ^ cfg[i + 1]]))
    send(b"\xDD")
    time.sleep_ms(20)
    tx.active(0)


def stream(hz=30, seconds=0):
    radar = RingRx(0, RADAR_RX, 115200)
    flow = RingRx(2, FLOW_RX, 19200)
    bus = I2C(0, sda=Pin(probe.SDA), scl=Pin(probe.SCL), freq=100000, timeout=20000)
    try:
        time.sleep_ms(250)
        if not flow.read():
            print("# flow sensor silent, sending start-up sequence")
            _flow_init()
        print("# stream start")
        rbuf = bytearray()
        fbuf = bytearray()
        period = 1000 // hz
        t0 = time.ticks_ms()
        nxt = period
        while not seconds or time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
            now = time.ticks_diff(time.ticks_ms(), t0)
            if now < nxt:
                time.sleep_ms(1)
                continue
            nxt += period
            if nxt < now:
                nxt = now + period

            try:
                d = bus.readfrom_mem(LIDAR_ADDR, 0, 2)
                mm = (d[0] << 8) | d[1]
            except OSError:
                mm = -1

            # Optical flow: FE 0A xL xH yL yH dtL dtH r r quality ver xor 55
            fbuf.extend(flow.read())
            dx = dy = fn = 0
            fq = -1
            while len(fbuf) >= 14:
                if fbuf[0] != 0xFE or fbuf[1] != 0x0A:
                    fbuf = fbuf[1:]
                    continue
                x = 0
                for i in range(2, 12):
                    x ^= fbuf[i]
                if x == fbuf[12] and fbuf[13] == 0x55:
                    vx = fbuf[2] | (fbuf[3] << 8)
                    vy = fbuf[4] | (fbuf[5] << 8)
                    dx += vx - 65536 if vx > 32767 else vx
                    dy += vy - 65536 if vy > 32767 else vy
                    fn += 1
                    fq = fbuf[10]
                    fbuf = fbuf[14:]
                else:
                    fbuf = fbuf[2:]

            # Radar: F4 F3 F2 F1 lenL lenH [count alarm (5 bytes x count)] F8 F7 F6 F5
            rbuf.extend(radar.read())
            rtxt = "-1"
            while True:
                i = rbuf.find(b"\xF4\xF3\xF2\xF1")
                if i < 0:
                    if len(rbuf) > 3:
                        rbuf = rbuf[-3:]
                    break
                if len(rbuf) < i + 6:
                    break
                n = rbuf[i + 4] | (rbuf[i + 5] << 8)
                if n > 64:
                    rbuf = rbuf[i + 4:]
                    continue
                end = i + 6 + n + 4
                if len(rbuf) < end:
                    break
                p = rbuf[i + 6:i + 6 + n]
                rbuf = rbuf[end:]
                if n < 2:
                    rtxt = "0"
                    continue
                parts = [str(p[0])]
                for k in range(p[0]):
                    t = p[2 + 5 * k:7 + 5 * k]
                    if len(t) == 5:
                        parts.append("%d,%d,%d,%d,%d" % (t[0] - 0x80, t[1], t[2], t[3], t[4]))
                rtxt = ",".join(parts)

            print("D,%d,%d,%d,%d,%d,%d,%s" % (now, mm, dx, dy, fn, fq, rtxt))
    finally:
        radar.close()
        flow.close()
