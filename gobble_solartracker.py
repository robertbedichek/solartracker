#!/usr/bin/env python3

# This attempt to supress the (harmless) warning about OpenSSL does not work, but leaving it here nonetheles
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="urllib3")

import serial
import time
import subprocess
import os
import requests

home_dir = os.path.expanduser("~")
app_token_path = os.path.join(home_dir, ".pushover", "ap_token.txt")
user_token_path = os.path.join(home_dir, ".pushover", "user_token.txt")

with open(app_token_path, "r") as f:
    app_token = f.read().strip()

with open(user_token_path, "r") as f:
    user_token = f.read().strip()

SSH_COMMAND = [
    "/usr/bin/env", "ssh",
    "-i", os.path.expanduser('~/.ssh/id_rsa'),
    "root@bedichek.org",
    "cat >> /var/www/home/solartracker.txt"
]

port = "/dev/tty.usbserial-11440"
baud_rate = 115200

ser = serial.Serial(port, baud_rate, timeout=1)

# Reset Arduino (toggle DTR)
ser.setDTR(False)
time.sleep(1)
ser.setDTR(True)
linecount = 0

def is_valid_data_line(line):
    if line.startswith("#"):
        return True  # Comment line is always OK

    parts = line.strip().split()
    if len(parts) < 14:
        print("must have at least 14 fields: ", line, " but has only ", len(parts))
        return False  # Must have at least 14 fields

    try:
        # Try to parse the timestamp fields
        date_part, time_part = parts[0], parts[1]
        from datetime import datetime
        datetime.strptime(f"{date_part} {time_part}", "%Y-%m-%d %H:%M:%S")

        # Try to parse all 12 remaining fields as integers
        for val in parts[2:]:
            float(val)

        return True
    except Exception:
        print("Unable to parse date/time field: " + line)
        return False

# Read and validate lines
# with open("solar_data.txt") as f:
#     for i, line in enumerate(f, start=1):
#         if not is_valid_data_line(line):
#             print(f"Malformed line {i}: {line.strip()}")

print(f"Connected to {port}. Reading data...\n")

try:
  while True:
    try:
      line = ser.readline().decode('utf-8').strip() 
    except UnicodeDecodeError:
      print("⚠️  Corrupted data skipped:", line)
      line = ""
    if line and is_valid_data_line(line):
      linecount = linecount + 1
      if "alert" in line.lower():
        data = {
            "token": app_token,        # This App's Pushover app token
            "user": user_token,        # My personal user key
            "message": line 
        }

        response = requests.post("https://api.pushover.net/1/messages.json", data=data)

        # Optional: log result
        print(f"Status: {response.status_code}")
        print(f"Response: {response.text}")

      try:
        print(line)
        subprocess.run(SSH_COMMAND, input=(line + "\n").encode("utf-8"), check=True)
      except subprocess.CalledProcessError as e:
        print(f"ssh error: {e}")

except KeyboardInterrupt:
  print(f"Stopping after reading and logging {linecount} lines.")
  ser.close()
