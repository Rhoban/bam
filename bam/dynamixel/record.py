# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import json
import datetime
import os
import numpy as np
import argparse
import time
from .dynamixel import DynamixelActuatorV1, DynamixelXL320, DynamixelXM430W350
from bam.trajectory import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass",       type=float, required=True)
arg_parser.add_argument("--length",     type=float, required=True)
arg_parser.add_argument("--port",       type=str,   default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir",     type=str,   required=True)
arg_parser.add_argument("--trajectory", type=str,   default="lift_and_drop")
arg_parser.add_argument("--motor",      type=str,   required=True)
arg_parser.add_argument("--kp",         type=int,   default=32)
arg_parser.add_argument("--vin",        type=float, default=None,
                        help="Input voltage stored in the log. "
                             "Defaults: mx64/mx106=15.0, xl320=7.5, xm430=12.0")
arg_parser.add_argument("--id",         type=int,   default=1)
arg_parser.add_argument("--baudrate",   type=int,   default=None,
                        help="Serial baud rate. Defaults: mx/xl320=1000000, xm430=57600")
args = arg_parser.parse_args()

os.makedirs(args.logdir, exist_ok=True)

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

# Motor selection
motor_key = args.motor.lower()

if motor_key in ("xm430", "xm430-w350", "xm430w350", "xm430-w350-t", "xm430-w350-r"):
    baudrate = args.baudrate if args.baudrate is not None else 57600
    vin      = args.vin     if args.vin     is not None else 12.0
    dxl = DynamixelXM430W350(args.port, id=args.id, baudrate=baudrate)
elif motor_key == "xl320":
    baudrate = args.baudrate if args.baudrate is not None else 1000000
    vin      = args.vin     if args.vin     is not None else 7.5
    dxl = DynamixelXL320(args.port, id=args.id, baudrate=baudrate)
else:
    baudrate = args.baudrate if args.baudrate is not None else 1000000
    vin      = args.vin     if args.vin     is not None else 15.0
    dxl = DynamixelActuatorV1(args.port, id=args.id, baudrate=baudrate)

trajectory = trajectories[args.trajectory]

# Motor initialisation
if hasattr(dxl, "prepare_for_recording"):
    dxl.prepare_for_recording(args.kp)
else:
    dxl.set_torque(False)
    dxl.set_p_gain(args.kp)

try:
    # Warm-up: hold initial position for 1 second
    torque_enable = False
    start = time.time()
    while time.time() - start < 1.0:
        goal_position, torque_enable = trajectory(0)
        if torque_enable:
            dxl.set_goal_position(goal_position)
        dxl.set_torque(torque_enable)

    start = time.time()
    data = {
        "mass":       args.mass,
        "length":     args.length,
        "kp":         args.kp,
        "vin":        vin,
        "motor":      args.motor,
        "id":         args.id,
        "baudrate":   baudrate,
        "trajectory": args.trajectory,
        "entries":    [],
    }

    while time.time() - start < trajectory.duration:
        t = time.time() - start
        goal_position, new_torque_enable = trajectory(t)
        if new_torque_enable != torque_enable:
            dxl.set_torque(new_torque_enable)
            torque_enable = new_torque_enable
            time.sleep(0.001)
        if torque_enable:
            dxl.set_goal_position(goal_position)
            time.sleep(0.001)

        t0 = time.time() - start
        entry = dxl.read_data()
        t1 = time.time() - start

        entry["timestamp"]     = (t0 + t1) / 2.0
        entry["goal_position"] = goal_position
        entry["torque_enable"] = torque_enable
        data["entries"].append(entry)

    # Compute median dt from recorded timestamps and store in log
    if len(data["entries"]) >= 2:
        timestamps = [e["timestamp"] for e in data["entries"]]
        diffs = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
        data["dt"] = float(np.median(diffs))
    else:
        data["dt"] = 0.0

    # Slowly return to zero
    goal_position = data["entries"][-1]["position"]
    return_dt     = 0.01
    max_variation = return_dt * 1.0
    while abs(goal_position) > 0:
        if goal_position > 0:
            goal_position = max(0.0, goal_position - max_variation)
        else:
            goal_position = min(0.0, goal_position + max_variation)
        dxl.set_goal_position(goal_position)
        time.sleep(return_dt)

finally:
    try:
        dxl.set_torque(False)
    except Exception:
        pass
    if hasattr(dxl, "close"):
        dxl.close()

# Format YYYY-MM-DD_HHhMMmSS
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))
print(f"Saved: {filename}")
