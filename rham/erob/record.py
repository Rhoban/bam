import json
import datetime
import argparse
import time
from .etherban import Client
from rham.trajectory import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--host", type=str, default="127.0.0.1")
arg_parser.add_argument("--offset", type=float, required=True)
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--arm_mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="lift_and_drop")
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--kp", type=int, default=10.0)
args = arg_parser.parse_args()

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

def angle_wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

trajectory = trajectories[args.trajectory]

eth = Client(args.host)
eth.run_background()

goal_position, torque_enable = trajectory(0)
eth.wait_stability(0)
eth.goto_safe(0, args.offset + goal_position)

start = time.time()
data = {
    "mass": args.mass,
    "arm_mass": args.arm_mass,
    "length": args.length,
    "kp": args.kp,
    "motor": args.motor,
    "trajectory": args.trajectory,
    "entries": []
}

while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, torque_enable = trajectory(t)

    if torque_enable:
        eth.position_control(0, args.offset + goal_position, 0.0, args.kp)
    else:
        eth.set_order(0, "torque", 0.0)
    eth.sync()

    status = eth.get_statuses()[0]
    entry = {
        "position": angle_wrap(status["position"] - args.offset),
        "speed": status["velocity"],
        "torque_demand": status["torque_demand"],
        "control": status["current"],
        "timestamp": time.time() - start,
        "goal_position": goal_position,
        "torque_enable": torque_enable,
    }

    data["entries"].append(entry)

eth.set_order(0, "torque", 0.0)
eth.stop()

#Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))
