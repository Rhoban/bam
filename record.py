import json
import os
import numpy as np
from dynamixel_sdk import *
import time

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(1.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

duration = 3.0

def read_data():
    # Reading position, speed, load, voltage and temperature
    data, result, error = packetHandler.readTxRx(portHandler, 1, ADDR_READ, 8)

    # Position is a 12-bit value
    position = (data[1] << 8) | data[0]
    position = 2 * np.pi * ((position / 4096) - 0.5)

    # Speed is a 10-bit value, units are 0.11 rpm per step
    speed = (data[3] << 8) | data[2]
    if speed > 1024:
        speed = -(speed - 1024)
    speed = speed * 0.11 * 2 * np.pi / 60.
    
    # Applied "load"
    load = (data[5] << 8) | data[4]
    if load > 1024:
        load = -(load - 1024)

    # Voltage is a byte value, units are 0.1 V
    volts = data[6] / 10.
    
    # Temperature are °C
    temp = data[7]

    return {
        "position": position,
        "speed": speed,
        "load": load,
        "volts": volts,
        "temp": temp
    }

ADDR_READ = 36
start = time.time()
elapsed = 0
data = []

while elapsed < duration:
    elapsed = time.time() - start

    entry = read_data()
    elapsed2 = time.time() - start
    entry["timestamp"] = (elapsed2 - elapsed) /2.0
    data.append(entry)

    # Very bad, for test only
    time.sleep(0.01)

json.dump(data, open("data.json", "w"))
