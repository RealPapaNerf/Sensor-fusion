# flashdump.py - read the LD2451 radar's SPI flash (PUYA P25D40SH, 4 Mbit) from
# the Pico. READ-ONLY: the only opcodes this file can send are in _READ_OPS, so
# it cannot erase or write the chip whatever is typed at the prompt.
#
# Wiring (hardware SPI0, pins no sensor uses):
#
#     Pico pad          flash pin (SOIC-8, dot = pin 1)
#     17  CS     ->     1  /CS
#     16  MISO   <-     2  DO
#     18  SCK    ->     6  CLK
#     19  MOSI   ->     5  DI
#     GND        --     4  GND
#     3V3        ->     8  VCC   (only when the radar's own 5 V is OFF)
#
# Pins 3 (/WP) and 7 (/HOLD) are left alone: on the radar board they are tied
# high. If ident() shows FF FF FF or 00 00 00, check /HOLD is high first.
#
#     import flashdump as f
#     f.ident()              # JEDEC ID, status, unique ID: proves the link
#     f.crc()                # whole-chip CRC32; run it twice, the two must match
#     f.dump()               # writes /radar.bin on the Pico, returns its CRC32

from machine import SPI, Pin
import binascii

CS, MISO, SCK, MOSI = 17, 16, 18, 19

# 03 read, 9F JEDEC ID, 90 manufacturer/device, 05/35 status 1/2, 4B unique
# ID, 5A SFDP, AB release from deep power-down. Nothing that writes.
_READ_OPS = (0x03, 0x9F, 0x90, 0x05, 0x35, 0x4B, 0x5A, 0xAB)

_spi = None
_cs = None


def _open(baud):
    global _spi, _cs
    _cs = Pin(CS, Pin.OUT, value=1)
    _spi = SPI(0, baudrate=baud, polarity=0, phase=0, bits=8,
               sck=Pin(SCK), mosi=Pin(MOSI), miso=Pin(MISO))


def _cmd(op, args=b"", n=0, buf=None):
    if op not in _READ_OPS:
        raise ValueError("refusing opcode %02X: this reader is read-only" % op)
    if _spi is None:
        _open(1_000_000)
    _cs(0)
    try:
        _spi.write(bytes([op]) + args)
        if buf is not None:
            _spi.readinto(buf)
            return buf
        return _spi.read(n) if n else b""
    finally:
        _cs(1)


def _hex(b):
    return " ".join("%02X" % x for x in b)


def size_bytes():
    """Chip size from the JEDEC capacity byte, 512 KB if it looks wrong."""
    cap = _cmd(0x9F, n=3)[2]
    return 1 << cap if 0x10 <= cap <= 0x18 else 512 * 1024


def ident(baud=1_000_000):
    _open(baud)
    _cmd(0xAB, b"\x00\x00\x00", 1)          # wake it if it was powered down
    jedec = _cmd(0x9F, n=3)
    print("JEDEC ID   ", _hex(jedec), "(PUYA is 85; P25D40 expected 85 60 13)")
    print("REMS       ", _hex(_cmd(0x90, b"\x00\x00\x00", 2)))
    print("status 1/2 ", _hex(_cmd(0x05, n=1)), _hex(_cmd(0x35, n=1)))
    print("unique ID  ", _hex(_cmd(0x4B, b"\x00\x00\x00\x00", 16)))
    sfdp = _cmd(0x5A, b"\x00\x00\x00\x00", 8)
    print("SFDP head  ", _hex(sfdp), "('SFDP' = 53 46 44 50)")
    if jedec in (b"\xff\xff\xff", b"\x00\x00\x00"):
        print("NO CHIP: bus reads all-%s. Check CS/DO/power/HOLD." % ("FF" if jedec[0] else "00"))
        return False
    print("size       ", size_bytes() // 1024, "KB")
    return True


def _blocks(baud, block=4096):
    _open(baud)
    total = size_bytes()
    buf = bytearray(block)
    for addr in range(0, total, block):
        _cmd(0x03, bytes([(addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]), buf=buf)
        yield addr, buf


def crc(baud=1_000_000):
    """CRC32 of the whole chip. Two runs (try different bauds) must agree; if
    they don't, something else - the radar's own controller - is on the bus."""
    c = 0
    for _, b in _blocks(baud):
        c = binascii.crc32(b, c)
    print("crc32 %08X at %d Hz" % (c, baud))
    return c


def dump(path="/radar.bin", baud=1_000_000):
    c = 0
    with open(path, "wb") as f:
        for addr, b in _blocks(baud):
            f.write(b)
            c = binascii.crc32(b, c)
    print("wrote", path, "crc32 %08X" % c)
    return c
