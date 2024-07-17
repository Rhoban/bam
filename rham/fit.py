import argparse
import socket
from datetime import datetime
import sys
from multiprocessing import Process
import numpy as np
import json
from copy import deepcopy
import json
import time
import optuna
import wandb

from .logs import Logs
from .model import models, Model, load_model
from . import message
from . import simulate
from .actuator import actuators

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
arg_parser.add_argument("--wandb", action="store_true")
arg_parser.add_argument("--set", type=str, default="")
arg_parser.add_argument("--validation_kp", type=int, default=0)
arg_parser.add_argument("--eval", action="store_true")
args = arg_parser.parse_args()

if not args.eval:
    # Json params file
    params_json_filename = args.output
    if not params_json_filename.endswith(".json"):
        params_json_filename = f"output/params_{params_json_filename}.json"
    json.dump({}, open(params_json_filename, "w"))

logs = Logs(args.logdir)
if not args.eval and args.validation_kp > 0:
    validation_logs = logs.split(args.validation_kp)
    print(f"{len(validation_logs.logs)} logs splitted for validation")
    if len(validation_logs.logs) == 0:
        raise ValueError("No logs for validation")


def compute_score(model: Model, log: dict) -> float:
    simulator = simulate.Simulator(model)
    result = simulator.rollout_log(
        log, reset_period=args.reset_period, simulate_control=True
    )
    positions = result[0]
    log_positions = np.array([entry["position"] for entry in log["entries"]])

    return np.mean(np.abs(positions - log_positions))


def compute_scores(model: Model, compute_logs=None):
    scores = 0
    for log in compute_logs.logs:
        # t0 = time.time()
        scores += compute_score(model, log)
        # t1 = time.time()
        # elapsed = t1 - t0
        # print(f"Durations: {elapsed:.6f} s")

    return scores / len(compute_logs.logs)


def make_model() -> Model:
    model = models[args.model]()
    model.set_actuator(actuators[args.actuator]())

    if args.set != "":
        parameters = model.get_parameters()
        values = eval(args.set)
        for key in values:
            parameters[key].value = values[key]
            parameters[key].optimize = False

    return model


def objective(trial):
    model = make_model()

    parameters = model.get_parameters()
    for name in parameters:
        parameter = parameters[name]
        if parameter.optimize:
            parameter.value = trial.suggest_float(name, parameter.min, parameter.max)

    return compute_scores(model, logs)


last_log = time.time()
wandb_run = None


def monitor(study, trial):
    global last_log, wandb_run
    elapsed = time.time() - last_log

    if args.wandb and wandb_run is None:
        wandb_run = wandb.init(
            name=f"{args.output}_{args.model}_{args.logdir}",
            # Set the project where this run will be logged
            project=f"{args.actuator}_identification",
            # Track hyperparameters and run metadata
            config={
                "logdir": args.logdir,
                "model": args.model,
                "hostname": socket.gethostname(),
            },
        )

    if elapsed > 0.2:
        last_log = time.time()
        data = deepcopy(study.best_params)
        trial_number = trial.number
        best_value = study.best_value
        wandb_log = {
            "optim/best_value": best_value,
            "optim/trial_number": trial_number,
        }

        model = make_model()
        model_parameters = model.get_parameters()
        for key in model_parameters:
            if key not in data:
                data[key] = model_parameters[key].value
        data["model"] = args.model
        data["actuator"] = args.actuator

        json.dump(data, open(params_json_filename, "w"))

        if args.validation_kp > 0:
            val_model = load_model(params_json_filename)
            val_best_value = compute_scores(val_model, validation_logs)
            wandb_log["optim/val_best_value"] = val_best_value

        print()
        message.bright(f"[Trial {trial_number}, Best score: {best_value}]")
        print(
            message.emphasis(f"Best params found (saved to {params_json_filename}): ")
        )
        for key in data:
            infos, warning = None, None

            if key in model_parameters:
                if model_parameters[key].optimize:
                    infos = f"min: {model_parameters[key].min}, max: {model_parameters[key].max}"
                else:
                    warning = "not optimized"

            message.print_parameter(key, data[key], infos, warning)

            if type(data[key]) == float:
                wandb_log[f"params/{key}"] = data[key]

        if wandb_run is not None:
            wandb.log(wandb_log)
    sys.stdout.flush()


if args.eval:
    model = load_model("params.json")
    print(f"Score: {compute_scores(model, logs)}")
else:
    study_name = f"study_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    # Study URL (when multiple workers are used)
    study_url = f"sqlite:///study.db"
    # study_url = f"mysql://root:root@127.0.0.1:6033/optuna"

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

    def optuna_run(enable_monitoring=True):
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
    for k in range(args.workers - 1):
        p = Process(target=optuna_run, args=(False,))
        p.start()

    optuna_run(True)
