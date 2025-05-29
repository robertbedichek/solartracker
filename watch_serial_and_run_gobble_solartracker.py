#!/usr/bin/env python3

import os
import time
import subprocess
import serial
import errno

DEVICE = "/dev/tty.usbserial-11440"
SCRIPT = "/Users/robertbedichek/bin/gobble_solartracker.py"
CHECK_INTERVAL = 60  # seconds


def is_device_available():
    try:
        fd = os.open(DEVICE, os.O_RDONLY | os.O_NONBLOCK)
        os.close(fd)
        return True
    except OSError as e:
        if e.errno == errno.EBUSY:
            return False
        return False

def start_script():
    print("Starting gobble_solartracker.py...")
    return subprocess.Popen(["python3", "-W", "ignore::UserWarning:urllib3", SCRIPT])

def main():
    proc = None

    try:
        while True:
            device_available = is_device_available()

            if device_available:
                if proc is None:
                    proc = start_script()
                elif proc.poll() is not None:
                    # Process exited — restart it
                    print("gobble_solartracker.py crashed or exited — restarting...")
                    proc = start_script()
            else:
                if proc is not None:
                    print("Device in use — stopping gobble_solartracker.py...")
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                    proc = None

            time.sleep(CHECK_INTERVAL)

    except KeyboardInterrupt:
        print("\nExiting watcher...")
        if proc:
            proc.terminate()
            proc.wait()

if __name__ == "__main__":
    main()
