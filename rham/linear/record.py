import json
import datetime
import argparse
import time
from .linear import LinearActuator
from .trajectory import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--port", type=str, default="COM3")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="lift_and_drop")
arg_parser.add_argument("--actuator", type=str, required=True)
arg_parser.add_argument("--kp", type=int, default=18.5)
arg_parser.add_argument("--vin", type=float, default=12.0)

args = arg_parser.parse_args()

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

lin=LinearActuator(args.port)

trajectory = trajectories[args.trajectory]

start = time.time()
while time.time() - start < 1.0:
    goal_position, torque_enable = trajectory(0)
    if torque_enable:
        lin.set_goal_position(goal_position)
    lin.set_torque(torque_enable)
    lin.set_p_gain(args.kp)
start = time.time()

data = {
    "mass": args.mass,
    "length": args.length,
    "kp": args.kp,
    "vin": args.vin,
    "actuator": args.actuator,
    "trajectory": args.trajectory,
    "entries": []
}

while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, new_torque_enable = trajectory(t)
    if new_torque_enable != torque_enable:
        lin.set_torque(new_torque_enable)
        torque_enable = new_torque_enable
        time.sleep(0.001)
    if torque_enable:
        lin.set_goal_position(goal_position)
        time.sleep(0.001)

    t0 = time.time() - start
    entry = lin.read_data()
    t1 = time.time() - start
    
    entry["timestamp"] = (t0 + t1) / 2.0
    entry["goal_position"] = goal_position
    entry["torque_enable"] = torque_enable
    data["entries"].append(entry)

goal_position = data["entries"][-1]["position"]

final_position=0.2
return_dt = 0.01
if goal_position != final_position:
    lin.set_goal_position(final_position)
time.sleep(return_dt)
lin.set_torque(False)

#Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))
