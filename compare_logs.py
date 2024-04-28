from logs import Logs
import matplotlib.pyplot as plt
from polyfit import spline, sliding_spline
import numpy as np

processed_logs = Logs("data_106_processed")
network_logs = Logs("data_106_network")

# Select the 4 logs with the 4 different trajectories
logs = []
traj = ["sin_sin", "up_and_down", "sin_time_square", "lift_and_drop"]
idx = 0
for i in range(len(processed_logs.logs)):
    if idx == 4:
        idx = 0
        # break
    if processed_logs.logs[i]["trajectory"] == traj[idx]:
        logs.append((processed_logs.logs[i], network_logs.logs[i]))
        idx += 1

# Plot the position, velocity and acceleration (and tau_l)
for processed_log, network_log in logs:
    ts_processed = [entry["timestamp"] for entry in processed_log["entries"]]
    ts_network = [entry["timestamp"] for entry in network_log["entries"]]

    pos_processed = [entry["position"] for entry in processed_log["entries"]]
    pos_network = [entry["position"] for entry in network_log["entries"]]

    plt.figure()
    plt.plot(ts_processed, pos_processed, label="Processed")
    plt.plot(ts_network, pos_network, label="Network")
    plt.legend()

    vel_network = [entry["velocity"] for entry in network_log["entries"]]
    vel_processed = [0]
    for i in range(1, len(pos_processed)):
        vel_processed.append((pos_processed[i] - pos_processed[i - 1]) / (ts_processed[i] - ts_processed[i - 1]))

    acc_network = [entry["acceleration"] for entry in network_log["entries"]]
    acc_processed = [0]
    for i in range(1, len(vel_processed)):
        acc_processed.append((vel_processed[i] - vel_processed[i - 1]) / (ts_processed[i] - ts_processed[i - 1]))

    plt.figure()
    plt.plot(ts_processed, vel_processed, label="Processed")
    plt.plot(ts_network, vel_network, label="Network")
    plt.legend()

    plt.figure()
    plt.plot(ts_processed, acc_processed, label="Processed")
    plt.plot(ts_network, acc_network, label="Network")
    plt.legend()

    tau_l = [entry["tau_l"] for entry in network_log["entries"]]
    plt.figure()
    plt.plot(ts_network, tau_l, label="tau_l")

    plt.show()