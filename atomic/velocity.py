import json
import os
import numpy as np
from dynamixel_sdk import *
import time

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(1.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

# Present speed register
ADDR_SPEED = 36

data = {}
if os.path.exists("data.json"):
    data = json.load(open("data.json"))

# Asking the user for the current voltage
volts = float(input("Voltage: "))
previous = None
total = 0
start_t = time.time()
duration = 5.0

while time.time() - start_t < duration:
    result, _, __ = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_SPEED)
    if previous is not None:
        delta = result - previous
        delta = (delta + 2048) % 4096 - 2048
        total += delta
        
        print(total)
        time.sleep(0.01)

    previous = result

# Showing the velocity in rad/s and rpm
velocity = ((total / 4096) * 2 * np.pi) / duration
rpm = (velocity * 60) / (2 * np.pi)

print(f"Velocity: {velocity} rad/s, rpm: {rpm}")
data[volts] = velocity

json.dump(data, open("data.json", "w"))