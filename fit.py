import argparse
import numpy as np
from scipy.optimize import minimize
import json
import time
import optuna
from model import Model, BaseModel
import simulate
import logs

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--output", type=str, default="params.json")
arg_parser.add_argument("--method", type=str, default="cmaes")
arg_parser.add_argument("--trials", type=int, default=100_000)
arg_parser.add_argument("--jobs", type=int, default=1)
arg_parser.add_argument("--reset_period", default=None, type=float)
arg_parser.add_argument("--control", action="store_true")
args = arg_parser.parse_args()

logs = logs.Logs(args.logdir)


def compute_score(model: BaseModel, log: dict) -> float:
    simulator = simulate.Simulate1R(log["mass"], log["length"], model)
    result = simulator.rollout_log(
        log, reset_period=args.reset_period, simulate_control=args.control
    )
    positions = result[0]
    log_positions = np.array([entry["position"] for entry in log["entries"]])

    return np.mean(np.abs(positions - log_positions))


def compute_scores(model: BaseModel):
    scores = 0
    for log in logs.logs:
        # t0 = time.time()
        scores += compute_score(model, log)
        # t1 = time.time()
        # elapsed = t1 - t0
        # print(f"Durations: {elapsed:.6f} s")

    return scores / len(logs.logs)


def objective(trial):
    model = Model()
    parameters = model.get_parameters()
    for name in parameters:
        parameter = parameters[name]
        if parameter.optimize:
            parameter.value = trial.suggest_float(name, parameter.min, parameter.max)

    return compute_scores(model)


def objective_x(x: list):
    model = Model()
    k = 0
    parameters = model.get_parameters()
    for name in parameters:
        parameter = parameters[name]
        if parameter.optimize:
            parameter.value = x[k]
            k += 1

    return compute_scores(model)


last_log = time.time()


def monitor(study, trial):
    global last_log
    elapsed = time.time() - last_log

    if elapsed > 0.050:
        last_log = time.time()
        json.dump(study.best_params, open(args.output, "w"))

        print()
        print(f"Trial {trial.number}, Best score: {study.best_value}")
        print(f"Best params found (saved to {args.output}): ")
        for key in study.best_params:
            print(f"- {key}: {study.best_params[key]}")


def monitor_x(x):
    print(x)


model = Model()

if args.method == "cmaes":
    sampler = optuna.samplers.CmaEsSampler(
        x0=model.get_parameter_values(), restart_strategy="bipop"
    )
elif args.method == "random":
    sampler = optuna.samplers.RandomSampler()
elif args.method == "nsgaii":
    sampler = optuna.samplers.NSGAIISampler()
else:
    raise ValueError(f"Unknown method: {args.method}")

study = optuna.create_study(sampler=sampler)
optuna.logging.set_verbosity(optuna.logging.WARNING)
study.optimize(objective, n_trials=args.trials, n_jobs=args.jobs, callbacks=[monitor])
