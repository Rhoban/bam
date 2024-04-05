import json
import os
import numpy as np
from dynamixel_sdk import *
import time
import matplotlib.pyplot as plt

ADDR_POSITION = 36

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(1.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

filename = str(input("Filename: "))
read_only = False
if os.path.exists(f"data/{filename}.json"):
    read_only = True

# Data collection
if not read_only:
    length = float(input("Length: "))
    mass = float(input("Mass: "))
    duration = float(input("Log duration: "))
    start_t = time.time()

    # Logging the position
    positions = []
    timestamps = []
    while time.time() - start_t < duration:
        result, _, __ = packetHandler.read2ByteTxRx(portHandler, 1, ADDR_POSITION)
        position = (result % 4096 - 2048) * -(2 * np.pi) / 4096 # rad
        positions.append(position)
        # print(position)
        timestamps.append(time.time() - start_t)

    # Saving the data
    data = {"l": length,
            "m": mass,
            "positions": positions, 
            "timestamps": timestamps}
    json.dump(data, open(f"data/{filename}.json", "w"))

# Data Loading
if read_only:
    data = json.load(open(f"data/{filename}.json"))
    length = data["l"]
    mass = data["m"]
    positions = data["positions"]
    timestamps = data["timestamps"]

# Data processing
velocities = np.diff(positions) / np.diff(timestamps)
accelerations = np.diff(velocities) / np.diff(timestamps[1:])

plt.plot(timestamps, positions)
plt.xlabel("Time (s)")
plt.ylabel("Position (rad)")
plt.show()

plt.plot(timestamps[1:], velocities, label="Velocity") 
plt.xlabel("Time (s)")
plt.ylabel("Velocity (rad/s)")
plt.show()

plt.plot(timestamps[2:], accelerations)
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (rad/s^2)")
plt.show()
exit()

# torques_friction = []
# for i in range(len(timestamps) - 2):
#     tau_f = mass * 9.81 * length * np.cos(positions[i+2]) + mass * length * accelerations[i]
#     torques_friction.append(tau_f)

# plt.plot(timestamps[2:], torques_friction)
# plt.xlabel("Time (s)")
# plt.ylabel("T_f (Nm)")
# plt.title("Friction Torque")
# plt.show()