import json
import os
import numpy as np
import sys
from tools import filename_argument

"""
Plots the no load speed vs voltage

Usage: python3 no_load_speed.py data/velocity_[motor].json
"""

filename = filename_argument()
data = json.load(open(filename))


volts = []
velocities = []
for key in data:
    volts.append(float(key))
    velocities.append(data[key])

import matplotlib.pyplot as plt

plt.scatter(velocities, volts, label="Sampled data")

plt.title(f"{filename} | No load speed")
plt.xlabel("Velocity (rad/s)")
plt.ylabel("Voltage (V)")
plt.legend()
plt.grid()
plt.show()
