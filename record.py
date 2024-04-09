import json
import os
import numpy as np
import argparse
from dynamixel_sdk import *
import time

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
args = arg_parser.parse_args()

os.system("setserial /dev/ttyUSB0 low_latency")

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(1.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

duration = 3.0
dt = 0.002

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
    speed = speed * 0.11 * 2 * np.pi / 60.0

    # Applied "load"
    load = (data[5] << 8) | data[4]
    if load > 1024:
        load = -(load - 1024)

    # Voltage is a byte value, units are 0.1 V
    volts = data[6] / 10.0

    # Temperature are °C
    temp = data[7]

    return {
        "position": position,
        "speed": speed,
        "load": load,
        "volts": volts,
        "temp": temp,
    }


ADDR_READ = 36
start = time.time()
data = {"mass": args.mass, "length": args.length, "dt": dt, "entries": []}

for step in range(int(duration / dt)):
    step_t = dt * step
    while (time.time() - start) < step_t:
        time.sleep(0.001)

    t0 = time.time() - start
    entry = read_data()
    t1 = time.time() - start

    print((t1-t0))
    entry["timestamp"] = (t0 + t1) /2.0

    entry["volts"] = 0.0
    entry["torque_enable"] = False
    data["entries"].append(entry)

json.dump(data, open("data.json", "w"))
