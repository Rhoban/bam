import json
import numpy as np
import os
from tools import linear_regression, filename_argument

"""
From a PWM logfile, estimate the H constant.
The JSON file should have the following format:

{
    "pwm_period": 42.6,
    "samples": [
        {
            "target": 157.24,
            "position": 165.2,
            "voltage": 15.0,
            "period": 4.5,
            "kp": 5
        },
        ...
    ]
}
"""

filename = filename_argument()
data = json.load(open(filename))

pwm_period = data["pwm_period"]
samples = data["samples"]

angular_errors_kp = []
volts = []

for sample in samples:
    voltage = sample["voltage"]
    target = sample["target"]
    position = sample["position"]
    period = sample["period"]
    kp = sample["kp"]

    volt = voltage * period / pwm_period
    angular_error = np.deg2rad(abs(position - target))
    angular_error_kp = kp * angular_error

    angular_errors_kp.append(angular_error_kp)
    volts.append(volt)

w = linear_regression(angular_errors_kp, volts)
print(f"H: {w}")

import matplotlib.pyplot as plt

plt.scatter(angular_errors_kp, volts, label="Sampled data")
plt.xlabel("Angular error * kp")
plt.ylabel("Voltage (V)")

ts = np.linspace(0, max(angular_errors_kp), 1000)
vs = w * ts
plt.plot(ts, vs, color="red", label=f"y = {w[0]}x")
plt.title(f"{filename} | H={w}")
plt.grid()
plt.legend()
plt.show()
