import json
import datetime
import os
import numpy as np
import argparse
import time
from .dynamixel import DynamixelActuatorV1
from .trajectory_2R import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="square")
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--kp", type=int, default=32)
arg_parser.add_argument("--speed", type=float, default=1.0)
args = arg_parser.parse_args()

dxl_1 = DynamixelActuatorV1(args.port, id=1)
dxl_2 = DynamixelActuatorV1(args.port, id=2)
trajectory = trajectories[args.trajectory]

dxl_1.set_p_gain(args.kp)
dxl_2.set_p_gain(args.kp)

dxl_1.set_torque(True)
dxl_2.set_torque(True)
try:
    dxl_1.read_data()
    dxl_2.read_data()
except:
    dxl_1.set_torque(False)
    dxl_2.set_torque(False)
    exit()

start = time.time()
while time.time() - start < 1.0:
    goal_1, goal_2 = trajectory(0)
    dxl_1.set_goal_position(goal_1)
    dxl_2.set_goal_position(goal_2)

start = time.time()
data = {
    "mass": args.mass,
    "length": [0.18, 0.18],
    "kp": args.kp,
    "motor": args.motor,
    "trajectory": args.trajectory,
    "entries": []
}

while (time.time() - start) * args.speed < trajectory.init_duration + trajectory.traj_duration:
    t = (time.time() - start) * args.speed

    goal_1, goal_2 = trajectory(t)
    dxl_1.set_goal_position(goal_1)
    dxl_2.set_goal_position(goal_2)
    time.sleep(0.001)

    if t >= trajectory.init_duration:
        t0 = time.time() - start
        entry = {"r1": dxl_1.read_data(), "r2": dxl_2.read_data()}
        t1 = time.time() - start

        entry["timestamp"] = (t0 + t1) / 2.0
        entry["r1"]["goal_position"] = goal_1
        entry["r2"]["goal_position"] = goal_2
        data["entries"].append(entry)

dxl_1.set_torque(False)
dxl_2.set_torque(False)

# Format YYYY-MM-DD_HH:mm:ss
date = datetime.datetime.now().strftime("%d_%Hh%Mm%S")

filename = f"{args.logdir}/{args.trajectory}_{args.kp}_{args.motor}_{date}.json"
json.dump(data, open(filename, "w"))
