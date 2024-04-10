import argparse
import numpy as np
import json
import time
import optuna
from model import load_model, BaseModel
import simulate
import logs
import matplotlib.pyplot as plt

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--params", type=str, default="params.json")
arg_parser.add_argument("--reset_period", default=None, type=float)
arg_parser.add_argument("--control", action="store_true")
arg_parser.add_argument("--sim", action="store_true")
args = arg_parser.parse_args()

logs = logs.Logs(args.logdir)

if args.sim:
    model = load_model(args.params)

for log in logs.logs:
    if args.sim:
        simulator = simulate.Simulate1R(log["mass"], log["length"], model)
        sim_q, sim_volts = simulator.rollout_log(log, reset_period=args.reset_period, simulate_control=args.control)
        sim_q = np.array(sim_q)

    ts = np.arange(len(log["entries"])) * log["dt"]
    q = [entry["position"] for entry in log["entries"]]
    goal_q = [entry["goal_position"] for entry in log["entries"]]
    volts = [entry["volts"] for entry in log["entries"]]
    torque_enable = np.array([entry["torque_enable"] for entry in log["entries"]])

    # Using 2 x-shared subplots
    f, (ax1, ax2) = plt.subplots(2, sharex=True)

    ax1.plot(ts, q, label="q")
    if args.sim:
        ax1.plot(ts, sim_q, label="sim_q")
    ax1.plot(ts, goal_q, label="goal_q")
    ax1.legend()
    ax1.set_title(f'{log["filename"]}, mass={log["mass"]}, length={log["length"]}, k={log["kp"]}')
    ax1.grid()

    # Using torque_enable color piecewise
    ax2.plot(ts, volts, label="volts")
    if args.control:
        if args.sim:
            ax2.plot(ts, sim_volts, label="sim_volts")
    # Shading the areas where torque is False
    ax2.fill_between(
        ts,
        min(volts) - 0.02,
        max(volts) + 0.02,
        where=[not torque for torque in torque_enable],
        color="red",
        alpha=0.3,
        label="torque off",
    )

    ax2.legend()

    plt.grid()
    plt.show()
