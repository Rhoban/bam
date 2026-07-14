# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

# from bam.feetech.feetech_pwm_control import FeetechPWMControl
from pypot.feetech import FeetechSTS3215IO
import rustypot
import json
import datetime
import os
import numpy as np
import argparse
import time
from bam.trajectory import *

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

# motors = {
#     "test": (1, "sts3215"),
# }
ids = [1]

# motor = FeetechPWMControl(id=args.id)
io = FeetechSTS3215IO("/dev/ttyACM0")
io.set_mode({1: 0})
# control = rustypot.feetech("/dev/ttyACM0", 1000000)
# control.set_mode(ids, 0)

trajectory = trajectories[args.trajectory]

start = time.time()
while time.time() - start < 1.0:
    goal_position, torque_enable = trajectory(0)
    if torque_enable:
        io.set_goal_position({1: np.rad2deg(goal_position)})
        # control.write_goal_position(ids, [goal_position])
        io.enable_torque([1])
        # control.enable_torque(ids)
        # motor.goal_position = np.rad2deg(goal_position)
        # motor.enable_torque()
    else:
        io.disable_torque([1])
        # control.disable_torque(ids)
        # motor.disable_torque()
    # control.set_kps(ids, [32])
    io.set_P_coefficient({1: args.kp})
    io.set_D_coefficient({1: 0})

    # motor.kp = args.kp


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


def convert_load(raw_load):
    sign = -1
    if raw_load > 1023:
        raw_load -= 1024
        sign = 1
    return sign * raw_load * 0.001


def read_data():

    # position = control.read_present_position(ids)[0]
    position = np.deg2rad(io.get_present_position([1])[0])
    # position = np.deg2rad(motor.io.get_present_position([motor.id])[0])

    # speed = np.deg2rad(motor.io.get_present_speed([motor.id])[0])  # TODO convert
    # speed = motor.get_present_speed()
    # speed = control.read_present_velocity(ids)[0]
    speed = np.deg2rad(io.get_present_speed([1])[0])

    load = convert_load(io.get_present_load([1])[0])

    volts = io.get_present_voltage([1])[0] * 0.1
    # volts = 0

    # temp = motor.io.get_present_temperature([motor.id])[0]
    temp = 0

    return {
        "position": float(position),
        "speed": float(speed),
        "load": float(load),
        "input_volts": float(volts),
        "temp": float(temp),
    }


while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, new_torque_enable = trajectory(t)
    if new_torque_enable != torque_enable:
        if new_torque_enable:
            # control.enable_torque(ids)
            # motor.enable_torque()
            io.enable_torque([1])
        else:
            # control.disable_torque(ids)
            io.disable_torque([1])
            # motor.disable_torque()
        torque_enable = new_torque_enable
        time.sleep(0.001)
    if torque_enable:
        # control.write_goal_position(ids, [goal_position])
        io.set_goal_position({1: np.rad2deg(goal_position)})

        # motor.goal_position = np.rad2deg(goal_position)
        time.sleep(0.001)

    t0 = time.time() - start
    entry = read_data()

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

    # control.write_goal_position(ids, [goal_position])
    # motor.goal_position = np.rad2deg(goal_position)
    io.set_goal_position({1: np.rad2deg(goal_position)})

    time.sleep(return_dt)

# control.write_goal_position(ids, [0])
io.set_goal_position({1: 0})
# motor.goal_position = 0
time.sleep(1)

# control.disable_torque(ids)
io.disable_torque([1])
# motor.disable_torque()


# Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"

json.dump(data, open(filename, "w"))
