import json
import datetime
import os
import numpy as np
import argparse
import time
from bam.trajectory import *
from bam.feetech.feetech import FeetechMotorsBus, configure, convert_radians_to_steps, convert_steps_to_radians

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
TRAJ_OFFSET = np.deg2rad(180)

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

motors = {
    "test": (1, "sts3215"),
}
motors_bus = FeetechMotorsBus(port=args.port, motors=motors)
motors_bus.connect()
configure(motors_bus)

trajectory = trajectories[args.trajectory]

start = time.time()
while time.time() - start < 1.0:
    goal_position, torque_enable = trajectory(0)
    goal_position += TRAJ_OFFSET
    if torque_enable:
        goal_position_steps = convert_radians_to_steps(goal_position, ["sts3215"])
        motors_bus.write("Goal_Position", goal_position_steps, ["test"])
        motors_bus.write("Torque_Enable", 1, ["test"])
    else:
        motors_bus.write("Torque_Enable", 0, ["test"])
    motors_bus.write("P_Coefficient", [args.kp], ["test"])
    motors_bus.write("I_Coefficient", [0], ["test"])
    motors_bus.write("D_Coefficient", [0], ["test"])


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


def read_data():

    position_steps = motors_bus.read("Present_Position", ["test"])
    position = convert_steps_to_radians(position_steps, ["sts3215"])[0]

    speed_steps = motors_bus.read("Present_Speed", ["test"])
    speed = convert_steps_to_radians(speed_steps, ["sts3215"])[0]


    load = 0  # TMP

    volts = motors_bus.read("Present_Voltage", ["test"])[0] * 0.1 # unit is 0.1v

    temp = motors_bus.read("Present_Temperature", ["test"])[0]
    # aze =  {
    #     "position": position,
    #     "speed": speed,
    #     "load": load,
    #     "input_volts": volts,
    #     "temp": temp,
    # }
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
    goal_position += TRAJ_OFFSET
    if new_torque_enable != torque_enable:
        if new_torque_enable:
            motors_bus.write("Torque_Enable", 1, ["test"])
        else:
            motors_bus.write("Torque_Enable", 0, ["test"])
        torque_enable = new_torque_enable
        time.sleep(0.001)
    if torque_enable:
        goal_position_steps = convert_radians_to_steps(goal_position, ["sts3215"])
        motors_bus.write("Goal_Position", goal_position_steps, ["test"])
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
while abs(goal_position) > TRAJ_OFFSET:
    if goal_position > TRAJ_OFFSET:
        goal_position = max(TRAJ_OFFSET, goal_position - max_variation)
    else:
        goal_position = min(TRAJ_OFFSET, goal_position + max_variation)
    goal_position_steps = convert_radians_to_steps(goal_position, ["sts3215"])
    motors_bus.write("Goal_Position", goal_position_steps, ["test"])

    time.sleep(return_dt)

motors_bus.write("Torque_Enable", 0, ["test"])

# Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"

json.dump(data, open(filename, "w"))
