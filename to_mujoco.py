import argparse
import numpy as np
import json
import time
from model import load_model, BaseModel
import matplotlib.pyplot as plt

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--params", type=str, default="params.json")
arg_parser.add_argument("--volts", type=float, default=15.0)
arg_parser.add_argument("--kp", type=int, default=32.0)
args = arg_parser.parse_args()

# Load the model
model = load_model(args.params)

print(f"Reading model of type {model.name}")
print(f" - Volts: {args.volts}")
print(f" - Kp: {args.kp}")
print()

print(f"MuJoCo parameters:")
model.to_mujoco(args.volts, args.kp)