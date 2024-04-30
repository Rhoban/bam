import argparse
from datetime import datetime
import os
import sys
from multiprocessing import Process
import numpy as np
import json
from copy import deepcopy
from actuator import actuators
import json
import time
import optuna
from model import models, BaseModel
import simulate
import wandb
import logs

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--output", type=str, default="params.json")
arg_parser.add_argument("--method", type=str, default="cmaes")
arg_parser.add_argument("--actuator", type=str, required=True)
arg_parser.add_argument("--model", type=str, required=True)
arg_parser.add_argument("--trials", type=int, default=100_000)
arg_parser.add_argument("--workers", type=int, default=1)
arg_parser.add_argument("--load-study", type=str, default=None)
arg_parser.add_argument("--reset_period", default=None, type=float)
arg_parser.add_argument("--control", action="store_true")
arg_parser.add_argument("--wandb", action="store_true")
args = arg_parser.parse_args()

logs = logs.Logs(args.logdir)

study_name = f"study_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

# Study URL (when multiple workers are used)
study_url = f"sqlite:///study.db"
# study_url = f"mysql://root:root@127.0.0.1:6033/optuna"

# Json params file
params_json_filename = args.output
if not params_json_filename.endswith(".json"):
    params_json_filename = f"output/params_{params_json_filename}.json"
json.dump({}, open(params_json_filename, "w"))

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
    model = models[args.model]()
    model.set_actuator(actuators[args.actuator]())

    parameters = model.get_parameters()
    for name in parameters:
        parameter = parameters[name]
        if parameter.optimize:
            parameter.value = trial.suggest_float(name, parameter.min, parameter.max)

    return compute_scores(model)


last_log = time.time()
wandb_run = None

def monitor(study, trial):
    global last_log, wandb_run
    elapsed = time.time() - last_log

    if args.wandb and wandb_run is None:
        control = "c1" if args.control else "c0"
        wandb_run = wandb.init(
            name=f"{args.output}_{args.model}_{control}_{args.logdir}",
            # Set the project where this run will be logged
            project="dxl_identification",
            # Track hyperparameters and run metadata
            config={
                "logdir": args.logdir,
                "model": args.model,
                "control": args.control,
            },
        )

    if elapsed > 0.2:
        last_log = time.time()
        data = deepcopy(study.best_params)
        data["model"] = args.model
        data["actuator"] = args.actuator
        trial_number = trial.number
        best_value = study.best_value
        wandb_log = {
            "optim/best_value": best_value,
            "optim/trial_number": trial_number,
        }

        json.dump(data, open(params_json_filename, "w"))

        print()
        print(f"Trial {trial_number}, Best score: {best_value}")
        print(f"Best params found (saved to {args.output}): ")
        for key in data:
            print(f"- {key}: {data[key]}")
            if type(data[key]) == float:
                wandb_log[f"params/{key}"] = data[key]
        
        if wandb_run is not None:
            wandb.log(wandb_log)
    sys.stdout.flush()


def monitor_x(x):
    print(x)


if args.method == "cmaes":
    sampler = optuna.samplers.CmaEsSampler(
        # x0=model.get_parameter_values(),
        restart_strategy="bipop"
    )
elif args.method == "random":
    sampler = optuna.samplers.RandomSampler()
elif args.method == "nsgaii":
    sampler = optuna.samplers.NSGAIISampler()
else:
    raise ValueError(f"Unknown method: {args.method}")

def optuna_run(enable_monitoring = True):
    if args.workers > 1:
        study = optuna.load_study(study_name=study_name, storage=study_url)
    else:
        study = optuna.create_study(sampler=sampler)
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    callbacks = []
    if enable_monitoring:
        callbacks = [monitor]
    study.optimize(objective, n_trials=args.trials, n_jobs=1, callbacks=callbacks)

if args.workers > 1:
    optuna.create_study(study_name=study_name, storage=study_url, sampler=sampler)

# Running the other workers
for k in range(args.workers-1):
    p = Process(target=optuna_run, args=(False,))
    p.start()
        
optuna_run(True)
