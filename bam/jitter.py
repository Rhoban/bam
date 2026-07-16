# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import argparse
import glob
import os
import numpy as np
import json

arg_parser = argparse.ArgumentParser(description="Show jitter histogram across logs")
arg_parser.add_argument(
    "--logdir", type=str, required=True, help="Directory containing raw log json files"
)
args = arg_parser.parse_args()

filenames = sorted(glob.glob(os.path.join(args.logdir, "*.json")))
if len(filenames) == 0:
    print(f"No json files found in {args.logdir}")
    exit(1)

dts = []
for filename in filenames:
    data = json.load(open(filename))
    ts = [entry["timestamp"] for entry in data["entries"]]
    dts.append(np.diff(ts))

dt = np.concatenate(dts)

mean = np.mean(dt)
std = np.std(dt)
print(f"Loaded {len(filenames)} logs, {len(dt)} intervals")
print(f"Mean: {mean:.6f} s, Std: {std:.6f} s")

import matplotlib.pyplot as plt

plt.hist(dt, bins=100)
plt.axvline(mean, color="red", linestyle="--", label=f"Mean: {mean:.6f} s")
plt.title(f"Histogram of time between samples\nMean: {mean:.6f} s, Std: {std:.6f} s")
plt.xlabel("Time (s)")
plt.ylabel("Count")
plt.legend()
plt.grid()
plt.show()
