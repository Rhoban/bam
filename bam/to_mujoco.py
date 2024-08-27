import argparse
import message
from model import load_model, BaseModel
import matplotlib.pyplot as plt

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--params", type=str, default="params.json")
arg_parser.add_argument("--set", type=str, default="")
args = arg_parser.parse_args()

# Load the model
model = load_model(args.params)

if args.set != "":
    values = eval(args.set)
    model.actuator.load_log(values)

message.bright(f"Reading model of type {model.name}")

model.actuator.to_mujoco()