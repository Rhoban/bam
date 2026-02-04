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
from rustypot import Xl330PyController
from bam.trajectory import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="lift_and_drop")
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--kp", type=int, default=32)
arg_parser.add_argument("--vin", type=float, default=7.4)
args = arg_parser.parse_args()

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")
    
_RADS_PER_SEC_PER_COUNT = 0.229 * (2.0 * np.pi / 60.0)
_PWM_LIMIT = 885  # XL330 Present PWM limit (counts)

def convert_velocity(raw_signed: float) -> float:
    return float(raw_signed) * _RADS_PER_SEC_PER_COUNT

def convert_pwm_to_duty(raw: float) -> float:
    x = float(raw)
    
    if x > 2**15-1:
        x -= 2**16
        
    y = np.clip(x / _PWM_LIMIT, -1.0, 1.0)
    
    return y

c = Xl330PyController(args.port, baudrate=1000000, timeout=0.01)
ID = 1

# dxl = DynamixelActuatorV1(args.port)
trajectory = trajectories[args.trajectory]

start = time.time()
while time.time() - start < 1.0:
    goal_position, torque_enable = trajectory(0)
    if torque_enable:
        c.write_goal_position(ID, goal_position)
    c.write_torque_enable(ID, torque_enable)
    c.write_position_p_gain(ID, args.kp)

start = time.time()
data = {
    "mass": args.mass,
    "length": args.length,
    "kp": args.kp,
    "vin": args.vin,
    "motor": args.motor,
    "trajectory": args.trajectory,
    "entries": []
}

while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, new_torque_enable = trajectory(t)
    if new_torque_enable != torque_enable:
        c.write_torque_enable(ID, new_torque_enable)
        torque_enable = new_torque_enable
        time.sleep(0.001)
    if torque_enable:
        c.write_goal_position(ID, goal_position)
        time.sleep(0.001)

    t0 = time.time() - start
    
    entry = {}
    entry["position"] = c.read_present_position(ID)[0] # rad
    entry["speed"] = convert_velocity(c.read_present_velocity(ID)[0]) # rad/s
    entry["load"] = convert_pwm_to_duty(c.read_present_pwm(ID)[0]) # duty ratio in [-1, 1]
    entry["input_volts"] = c.read_present_input_voltage(ID)[0]/10. # V
    entry["temp"] = c.read_present_temperature(ID)[0] # °C
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
    c.write_goal_position(ID, goal_position)
    
    time.sleep(return_dt)
c.write_torque_enable(ID, False)

#Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))
