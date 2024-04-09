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
arg_parser.add_argument("--rereset", default=None, type=float)
args = arg_parser.parse_args()

logs = logs.Logs(args.logdir)
model = Model()
model.load_parameters(args.params)

for log in logs.logs:
    simulator = simulate.Simulate1R(log["mass"], log["length"], model)
    sim_q = simulator.rollout_log(log, rereset=args.rereset)

    ts = np.arange(len(sim_q))*log["dt"]
    q = [entry["position"] for entry in log["entries"]]
    volts = [entry["volts"] for entry in log["entries"]]
    sim_q = np.array(sim_q)

    ax1 = plt.subplot(211)
    ax2 = plt.subplot(212)

    ax1.plot(ts, q, label="log_q")
    ax1.plot(ts, sim_q, label="sim_q")
    ax1.legend()
    ax1.set_title(log["filename"])
    ax1.grid()

    ax2.plot(ts, volts, label="volts")
    ax2.legend()

    plt.grid()
    plt.show()