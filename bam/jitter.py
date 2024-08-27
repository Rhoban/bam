import sys
import numpy as np
import json

if len(sys.argv) != 2:
    print("Usage: jitter.py <filename (raw log)>")
    sys.exit(1)

filename = sys.argv[1]

data = json.load(open(filename))

ts = [entry["timestamp"] for entry in data["entries"]]
dt = np.diff(ts)

import matplotlib.pyplot as plt

plt.plot(dt)
plt.title("Time between samples")
plt.xlabel("Sample")
plt.ylabel("Time (s)")
plt.grid()
plt.show()

# Showing histogram
plt.hist(dt, bins=100)
plt.title("Histogram of time between samples")
plt.xlabel("Time (s)")
plt.ylabel("Count")
plt.grid()
plt.show()
