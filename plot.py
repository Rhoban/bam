import argparse
import numpy as np
import json
import time
import optuna
from model import Model, BaseModel
import simulate
import logs
import matplotlib.pyplot as plt

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--params", type=str, default="params.json")
arg_parser.add_argument("--reset_period", default=None, type=float)
args = arg_parser.parse_args()

logs = logs.Logs(args.logdir)
model = Model()
model.load_parameters(args.params)

for log in logs.logs:
    simulator = simulate.Simulate1R(log["mass"], log["length"], model)
    sim_q = simulator.rollout_log(log, reset_period=args.reset_period)

    ts = np.arange(len(sim_q)) * log["dt"]
    q = [entry["position"] for entry in log["entries"]]
    volts = [entry["volts"] for entry in log["entries"]]
    torque_enable = np.array([entry["torque_enable"] for entry in log["entries"]])
    sim_q = np.array(sim_q)

    # Using 2 x-shared subplots
    f, (ax1, ax2) = plt.subplots(2, sharex=True)

    ax1.plot(ts, q, label="log_q")
    ax1.plot(ts, sim_q, label="sim_q")
    ax1.legend()
    ax1.set_title(f'{log["filename"]}, mass={log["mass"]}, length={log["length"]}')
    ax1.grid()

    # Using torque_enable color piecewise
    ax2.plot(ts, volts, label="volts")
    # Shading the areas where torque is False
    ax2.fill_between(
        ts,
        min(volts) - 0.02,
        max(volts) + 0.02,
        where=[not torque for torque in torque_enable],
        color="red",
        alpha=0.3,
        label="torque_enable",
    )

    ax2.legend()

    plt.grid()
    plt.show()
