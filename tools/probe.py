#!/usr/bin/env python3
"""Send one command to the sensor-probe firmware and print the reply.

    tools/probe.py help
    tools/probe.py uart sweep
    tools/probe.py i2c scan
    tools/probe.py --timeout 20 uart mon 10000

Exits 0 on "ok", 1 on "err:", 2 on timeout / no port.

Never opens the port at 1200 baud: on this board that is the signal to reboot
into the bootloader.
"""
import argparse
import glob
import sys
import time

import serial


def find_port():
    ports = glob.glob("/dev/serial/by-id/usb-Arduino_Nano_33_BLE_*")
    return ports[0] if ports else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for the reply")
    ap.add_argument("cmd", nargs="+")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print("no Nano 33 BLE found under /dev/serial/by-id", file=sys.stderr)
        return 2

    with serial.Serial(port, 115200, timeout=0.2) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.write((" ".join(args.cmd) + "\n").encode())
        deadline = time.time() + args.timeout
        buf = b""
        while time.time() < deadline:
            buf += ser.read(4096)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode(errors="replace").rstrip("\r")
                print(text, flush=True)
                if text == "ok":
                    return 0
                if text.startswith("err:"):
                    return 1
        print("timed out waiting for ok/err", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
