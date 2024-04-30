import argparse
import numpy as np
from model import load_model, load_network
import simulate
import logs
import matplotlib.pyplot as plt

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--params", type=str, default=None)
arg_parser.add_argument("--network", type=str, default=None)
arg_parser.add_argument("--reset_period", default=None, type=float)
arg_parser.add_argument("--control", action="store_true")
arg_parser.add_argument("--sim", action="store_true")
args = arg_parser.parse_args()

logs = logs.Logs(args.logdir)

# Loading models
models = []
if args.sim:
    if args.network is not None:
        model = load_network(args.network)
        model.name = "network"
        models.append(model)
    if args.params is not None:
        model_names = args.params.split(",")
        for model_name in model_names:
            model = load_model(model_name)
            models.append(model)

# Plotting
for log in logs.logs:
    all_sim_q = []
    all_sim_volts = []
    all_names = []

    if args.sim:
        for model in models:
            all_names.append(model.name)
            simulator = simulate.Simulate1R(log["mass"], log["length"], model)
            sim_q, sim_volts = simulator.rollout_log(log, reset_period=args.reset_period, simulate_control=args.control)
            all_sim_q.append(np.array(sim_q))
            all_sim_volts.append(np.array(sim_volts))

    ts = np.arange(len(log["entries"])) * log["dt"]
    q = [entry["position"] for entry in log["entries"]]
    goal_q = [entry["goal_position"] for entry in log["entries"]]
    volts = [(entry["volts"] if entry["torque_enable"] else None) for entry in log["entries"]]
    torque_enable = np.array([entry["torque_enable"] for entry in log["entries"]])

    # Using 2 x-shared subplots
    f, (ax1, ax2) = plt.subplots(2, sharex=True)

    ax1.plot(ts, q, label="q")
    ax1.plot(ts, goal_q, label="goal_q", color="black", linestyle="--")
    if args.sim:
        for model_name, sim_q in zip(all_names, all_sim_q):
            ax1.plot(ts, sim_q, label=f"{model_name}_q")
    ax1.legend()
    title = f'{log["motor"]}, {log["trajectory"]}, m={log["mass"]}, l={log["length"]}, k={log["kp"]}'

    ax1.set_title(f'{log["motor"]}, {log["trajectory"]}, m={log["mass"]}, l={log["length"]}, k={log["kp"]}')
    ax1.set_ylabel("angle [rad]")
    ax1.grid()

    # Using torque_enable color piecewise
    ax2.plot(ts, volts, label="volts")
    if args.control:
        if args.sim:
            for model_name, sim_volts in zip(all_names, all_sim_volts):
                ax2.plot(ts, sim_volts, label=f"{model_name}_volts")
    # Shading the areas where torque is False
    ax2.fill_between(
        ts,
        min([entry["volts"] for entry in log["entries"]]) - 0.02,
        max([entry["volts"] for entry in log["entries"]]) + 0.02,
        where=[not torque for torque in torque_enable],
        color="red",
        alpha=0.3,
        label="torque off",
    )
    ax2.set_ylabel("volts [V]")
    ax2.legend()
    plt.xlabel("time [s]")

    plt.grid()
    plt.show()
