"""USB presence and a narrowly scoped process supervisor. Never enables motors."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

RECONNECT_EXIT = 75


def usb_signature(root=Path("/sys/bus/usb/devices")):
    devices = []
    try:
        entries = list(root.iterdir())
    except OSError:
        return ()
    for entry in entries:
        try:
            if (entry.joinpath("idVendor").read_text().strip().lower(),
                    entry.joinpath("idProduct").read_text().strip().lower()) != ("34b7", "6877"):
                continue
            devices.append(f"{entry.name}:{entry.joinpath('busnum').read_text().strip()}:"
                           f"{entry.joinpath('devnum').read_text().strip()}")
        except OSError:
            continue
    return tuple(sorted(devices))


def transport_error(exc):
    message = str(exc).lower()
    return any(word in message for word in ("dmcan", "dm_device", "usb disconnect", "usb unavailable"))


class RestartForUsb(Exception):
    pass


def supervise(script, args):
    stopping = threading.Event()
    child = None

    def stop(signum, frame):
        stopping.set()
        if child and child.poll() is None:
            child.terminate()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    attempts = 0
    while not stopping.is_set():
        environment = dict(os.environ, REBOT_PANEL_CHILD="1", REBOT_RECONNECT_COUNT=str(attempts))
        child = subprocess.Popen([sys.executable, "-u", str(script), *args], env=environment)
        code = child.wait()
        if stopping.is_set() or code != RECONNECT_EXIT:
            return 0 if stopping.is_set() else code
        attempts += 1
        # Only an explicit USB recovery request is restartable. Code errors and
        # user shutdown never start another child.
        delay = min(0.5 * attempts, 5.0)
        print(f"USB recovery #{attempts}; fresh SDK process in {delay:.1f}s; no motion replay", flush=True)
        stopping.wait(delay)
    return 0
