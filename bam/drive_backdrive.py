# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import argparse
import numpy as np
import matplotlib.pyplot as plt
from .model import load_model

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--params", type=str, default="params.json")
arg_parser.add_argument("--max_torque", type=float, required=True)
args = arg_parser.parse_args()

plt.figure(figsize=(3, 2))

model = load_model(args.params)
for velocity in range(10):
    torques = np.linspace(0, args.max_torque, 500)
    lows = []
    highs = []
    for motor_torque in torques:
        external_torque_low = None
        external_torque_high = None
        for external_torque in torques:
            frictions, _ = model.compute_frictions(motor_torque, -external_torque, velocity)

            if motor_torque - frictions > external_torque:
                if external_torque_low is None or external_torque > external_torque_low:
                    external_torque_low = external_torque

            if external_torque_high is None and motor_torque + frictions < external_torque:
                external_torque_high = external_torque

        lows.append(external_torque_low)
        highs.append(external_torque_high)
            
    plt.plot(torques, lows, color="tab:blue", label=r"$\tau_{drive}(\tau_m)$" if velocity == 0 else None, alpha=np.exp(-velocity/2))
    plt.plot(torques, highs, color="tab:red", label=r"$\tau_{backdrive}(\tau_m)$" if velocity == 0 else None, alpha=np.exp(-velocity/2))

plt.plot([0, args.max_torque], [0, args.max_torque], color="black", linestyle="--", label=r"$\tau_f = 0$")
plt.xlabel(r"$\tau_m$ [N.m]")
plt.ylabel(r"$-\tau_e$ [N.m]")

title = r"$\mathcal{M}_1$"
if "M2" in model.title:
    title = r"$\mathcal{M}_2$"
elif "M3" in model.title:
    title = r"$\mathcal{M}_3$"
elif "M4" in model.title:
    title = r"$\mathcal{M}_4$"
elif "M5" in model.title:
    title = r"$\mathcal{M}_5$"
elif "M6" in model.title:
    title = r"$\mathcal{M}_6$"

plt.title(title)
plt.legend(ncol=3)
plt.xlim(0, args.max_torque)
plt.ylim(0, args.max_torque)
plt.grid()
plt.show()
