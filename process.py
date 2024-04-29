import glob
from copy import deepcopy
import os
import json
import numpy as np
from dynamixel import compute_volts
import argparse
from scipy.signal import savgol_filter

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--raw", type=str, required=True)
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--dt", type=float, default=0.002)
args = arg_parser.parse_args()

for logfile in glob.glob(f"{args.raw}/*.json"):
    data = json.load(open(logfile))
    data_output = deepcopy(data)
    data_output["entries"] = []
    data_output["dt"] = args.dt

    duration = data["entries"][-1]["timestamp"]
    print(f"* Processing {logfile} with duration {duration:.2f}s")
    ts = np.arange(0.0, duration, args.dt)
    frame = 0

    # Linear interpolation on the entries
    for t in ts:
        while t > data["entries"][frame + 1]["timestamp"]:
            frame += 1
        entry_1 = data["entries"][frame]
        entry_2 = data["entries"][frame + 1]
        new_entry = {}

        for key in entry_1:
            if key == "timestamp":
                continue
            new_entry[key] = entry_1[key] + (entry_2[key] - entry_1[key]) * (
                t - entry_1["timestamp"]
            ) / (entry_2["timestamp"] - entry_1["timestamp"])

        new_entry["torque_enable"] = True if (new_entry["torque_enable"] > 0.5) else False

        position_error = new_entry["goal_position"] - new_entry["position"]
        new_entry["volts"] = compute_volts(position_error, data["kp"])
        if not new_entry["torque_enable"]:
            new_entry["volts"] = 0

        new_entry["timestamp"] = t

        data_output["entries"].append(new_entry)

    # Using Savitzky-Golay filter to smooth the position and interpolate velocity and acceleration
    # https://fr.wikipedia.org/wiki/Algorithme_de_Savitzky-Golay
    position = [entry["position"] for entry in data_output["entries"]]
    if data["trajectory"] == "up_and_down":
        window = 351
    elif data["trajectory"] == "lift_and_drop":
        window = 251
    elif data["trajectory"] == "sin_sin":
        window = 81
    elif data["trajectory"] == "sin_time_square":
        window = 101
    degree = 3

    filtered_position = savgol_filter(position, window_length=window, polyorder=degree)
    velocity = savgol_filter(position, window_length=window, polyorder=degree, deriv=1, delta=args.dt, mode="nearest")
    acceleration = savgol_filter(position, window_length=window, polyorder=degree, deriv=2, delta=args.dt, mode="nearest")

    for i, entry in enumerate(data_output["entries"]):
        entry["position"] = filtered_position[i]
        entry["velocity"] = velocity[i]
        entry["acceleration"] = acceleration[i]

    filename = os.path.basename(logfile)
    output_filename = f"{args.logdir}/{filename}"
    json.dump(data_output, open(output_filename, "w"))
