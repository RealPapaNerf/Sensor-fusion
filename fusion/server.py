#!/usr/bin/env python3
"""Fusion display server: Pico sensor stream -> browser.

    python3 fusion/server.py            # then open http://localhost:5187

Copies pico/probe.py and pico/fusion.py to the Pico, starts fusion.stream()
through mpremote, and relays each sample to the page as server-sent events.
If the Pico is unplugged it keeps retrying. Standard library only.
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MPREMOTE = os.path.expanduser("~/.venvs/esptool/bin/mpremote")

clients = set()
clients_lock = threading.Lock()
status = {"state": "starting", "detail": ""}


def broadcast(event, data):
    msg = ("event: %s\ndata: %s\n\n" % (event, json.dumps(data))).encode()
    with clients_lock:
        for q in list(clients):
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass  # a stalled tab just misses samples


def set_status(state, detail=""):
    status["state"], status["detail"] = state, detail
    broadcast("status", status)
    print("[%s] %s" % (state, detail), flush=True)


def parse(line):
    """D,ms,mm,dx,dy,fn,fq,radar...  ->  dict, or None."""
    f = line.split(",")
    if len(f) < 8 or f[0] != "D":
        return None
    try:
        v = [int(x) for x in f[1:]]
    except ValueError:
        return None
    out = {"t": v[0], "mm": v[1], "dx": v[2], "dy": v[3], "fn": v[4], "fq": v[5]}
    if v[6] >= 0:
        n = v[6]
        rest = v[7:]
        out["radar"] = [
            {"ang": rest[i], "dist": rest[i + 1], "dir": rest[i + 2],
             "kmh": rest[i + 3], "snr": rest[i + 4]}
            for i in range(0, min(n * 5, len(rest) - len(rest) % 5), 5)
        ]
    return out


def pump():
    while True:
        try:
            set_status("connecting", "copying code to the Pico")
            for name in ("probe.py", "fusion.py"):
                subprocess.run([MPREMOTE, "cp", os.path.join(ROOT, "pico", name), ":" + name],
                               check=True, capture_output=True, timeout=30)
            proc = subprocess.Popen(
                [MPREMOTE, "exec", "import fusion; fusion.stream()"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            set_status("connecting", "waiting for the first sample")
            got = False
            for line in proc.stdout:
                line = line.strip()
                if line.startswith("#"):
                    set_status("connecting", line[1:].strip())
                    continue
                sample = parse(line)
                if sample is None:
                    if line:
                        print("pico: " + line, flush=True)
                    continue
                if not got:
                    got = True
                    set_status("live", "")
                broadcast("sample", sample)
            proc.wait()
            set_status("lost", "stream ended (exit %s)" % proc.returncode)
        except subprocess.CalledProcessError as e:
            detail = (e.stderr or b"").decode(errors="replace").strip().splitlines()
            set_status("lost", detail[-1] if detail else "could not reach the Pico")
        except Exception as e:  # keep retrying whatever happens
            set_status("lost", str(e))
        time.sleep(2)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(ROOT, "docs", "index.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            q = queue.Queue(maxsize=600)
            with clients_lock:
                clients.add(q)
            try:
                self.wfile.write(("event: status\ndata: %s\n\n" % json.dumps(status)).encode())
                self.wfile.flush()
                while True:
                    try:
                        self.wfile.write(q.get(timeout=10))
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with clients_lock:
                    clients.discard(q)
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5187)
    args = ap.parse_args()
    threading.Thread(target=pump, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    srv.daemon_threads = True
    print("fusion display on http://localhost:%d" % args.port, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
