import json
import os
import numpy as np
import sys
from tools import linear_regression, filename_argument

"""
Estimating kt constant from a velocity log (obtained with velocity.py)

Usage: python3 estimate_kt.py data/velocity_[motor].json
"""

filename = filename_argument()
data = json.load(open(filename))


volts = []
velocities = []
for key in data:
    volts.append(float(key))
    velocities.append(data[key])

w = linear_regression(velocities, volts)
print(f"kt: {w}")

import matplotlib.pyplot as plt

plt.scatter(velocities, volts, label="Sampled data")

ts = np.linspace(0, max(velocities), 1000)
vs = w * ts
plt.plot(ts, vs, color="red", label=f"y = {w[0]}x")

plt.title(f"{filename} | kt={w}")
plt.xlabel("Velocity (rad/s)")
plt.ylabel("Voltage (V)")
plt.legend()
plt.grid()
plt.show()
