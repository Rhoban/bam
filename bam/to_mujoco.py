# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import argparse
from .message import bright, yellow
from .model import load_model

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--params", type=str, default="params.json")
arg_parser.add_argument("--kp", type=float, default=0.0)
arg_parser.add_argument("--kd", type=float, default=0.0)
arg_parser.add_argument("--vin", type=float, default=0.0)
args = arg_parser.parse_args()

# Load the model
model = load_model(args.params)

values = {
    "kp": args.kp,
    "kd": args.kd,
    "vin": args.vin,
    "mass": 0.0,
    "arm_mass": 0.0,
    "length": 0.0
}
model.actuator.load_log(values)

if model.name != "m1":
    print(yellow(f"WARNING: Model other than m1 can't be exported exactly to MuJoCo (model is {model.name})"))

bright(f"Reading model of type {model.name} from {args.params}")
bright(f"Parameters export for MuJoCo, actuator {model.actuator_name}")

model.actuator.to_mujoco()