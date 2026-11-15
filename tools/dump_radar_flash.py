#!/usr/bin/env python3
"""Dump the LD2451 radar's SPI flash through the Pico (see pico/flashdump.py).

    python3 tools/dump_radar_flash.py            # ident, 3 CRC passes, dump, copy
    python3 tools/dump_radar_flash.py --ident    # just prove the wiring

Reads the chip three times (two speeds) and only keeps the dump if every pass
agrees, then copies it to firmware/ld2451-flash-<crc>.bin and checks the copy.
"""
import binascii
import hashlib
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
MPREMOTE = str(pathlib.Path.home() / ".venvs/esptool/bin/mpremote")


def mp(*args, timeout=300):
    r = subprocess.run([MPREMOTE, *args], capture_output=True, text=True, timeout=timeout)
    out = r.stdout + r.stderr
    print(out, end="")
    if r.returncode:
        sys.exit("mpremote failed")
    return out


def main():
    mp("cp", str(ROOT / "pico/flashdump.py"), ":flashdump.py")
    out = mp("exec", "import flashdump as f; print('LINK', f.ident())")
    if "LINK True" not in out:
        sys.exit("No flash on the bus: stop and check the wiring before reading.")
    if "--ident" in sys.argv:
        return

    crcs = []
    for baud in (1_000_000, 4_000_000, 1_000_000):
        out = mp("exec", "import flashdump as f; f.crc(%d)" % baud)
        crcs.append(re.search(r"crc32 ([0-9A-F]{8})", out).group(1))
    if len(set(crcs)) != 1:
        sys.exit("CRC passes disagree %s: bus contention or a loose wire. "
                 "Not saving a dump." % crcs)

    out = mp("exec", "import flashdump as f; f.dump()")
    if crcs[0] not in out:
        sys.exit("dump CRC differs from the verify passes")
    dest = ROOT / "firmware" / ("ld2451-flash-%s.bin" % crcs[0].lower())
    dest.parent.mkdir(exist_ok=True)
    mp("cp", ":radar.bin", str(dest))
    data = dest.read_bytes()
    local = "%08X" % binascii.crc32(data)
    if local != crcs[0]:
        sys.exit("copied file CRC %s != chip CRC %s" % (local, crcs[0]))
    mp("rm", ":radar.bin")

    blank = sum(1 for i in range(0, len(data), 4096) if data[i:i + 4096] == b"\xff" * 4096)
    print("\nSaved %s  (%d bytes)" % (dest, len(data)))
    print("crc32  %s\nsha256 %s" % (local, hashlib.sha256(data).hexdigest()))
    print("blank (all-FF) 4 KB sectors: %d of %d" % (blank, len(data) // 4096))
    print("first 64 bytes:", data[:64].hex(" "))


if __name__ == "__main__":
    main()
