import json
import datetime
import os
import numpy as np
import argparse
import time
from .dynamixel import DynamixelActuatorV1
from bam.trajectory import *
from .io_330 import Dxl330IO

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="lift_and_drop")
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--kp", type=int, default=32)
arg_parser.add_argument("--vin", type=float, default=15.0)
arg_parser.add_argument("--id", type=int, required=True)
args = arg_parser.parse_args()

os.makedirs(args.logdir, exist_ok=True)

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

dxl = Dxl330IO(args.port, baudrate=3000000, use_sync_read=True)
trajectory = trajectories[args.trajectory]

start = time.time()
while time.time() - start < 1.0:
    goal_position, torque_enable = trajectory(0)
    if torque_enable:
        dxl.set_goal_position({args.id: np.rad2deg(goal_position)})
        dxl.enable_torque([args.id])
    else:
        dxl.disable_torque([args.id])
    dxl.set_pid_gain({args.id: [args.kp, 0, 0]})


start = time.time()
data = {
    "mass": args.mass,
    "length": args.length,
    "kp": args.kp,
    "vin": args.vin,
    "motor": args.motor,
    "trajectory": args.trajectory,
    "entries": [],
}


def read_data(dxl, id):
    position = dxl.get_present_position([args.id])[0]
    position = np.deg2rad(position)
    print(position)

    speed = dxl.get_present_velocity([args.id])[0]

    load = 0  # TMP

    volts = dxl.get_present_input_voltage([args.id])[0]

    temp = dxl.get_present_temperature([args.id])[0]
    return {
        "position": position,
        "speed": speed,
        "load": load,
        "input_volts": volts,
        "temp": temp,
    }


while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, new_torque_enable = trajectory(t)
    if new_torque_enable != torque_enable:
        dxl.enable_torque([args.id]) if new_torque_enable else dxl.disable_torque(
            [args.id]
        )
        torque_enable = new_torque_enable
        time.sleep(0.001)
    if torque_enable:
        dxl.set_goal_position({args.id: np.rad2deg(goal_position)})
        time.sleep(0.001)

    t0 = time.time() - start
    entry = read_data(dxl, args.id)

    t1 = time.time() - start

    entry["timestamp"] = (t0 + t1) / 2.0
    entry["goal_position"] = goal_position
    entry["torque_enable"] = torque_enable
    data["entries"].append(entry)

goal_position = data["entries"][-1]["position"]
return_dt = 0.01
max_variation = return_dt * 1.0
while abs(goal_position) > 0:
    if goal_position > 0:
        goal_position = max(0, goal_position - max_variation)
    else:
        goal_position = min(0, goal_position + max_variation)
    dxl.set_goal_position({args.id: np.rad2deg(goal_position)})
    time.sleep(return_dt)
dxl.disable_torque([args.id])

# Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))
