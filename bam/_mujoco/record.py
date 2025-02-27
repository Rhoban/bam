import json
import datetime
import os
import numpy as np
# import argparse
import time
from bam.trajectory import *
from bam.mujoco_identification_rig.identification_rig import MujocoIdentificationRig

# arg_parser = argparse.ArgumentParser()
# arg_parser.add_argument("--mass", type=float, required=True)
# arg_parser.add_argument("--length", type=float, required=True)
# arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
# arg_parser.add_argument("--logdir", type=str, required=True)
# arg_parser.add_argument("--trajectory", type=str, default="lift_and_drop")
# arg_parser.add_argument("--motor", type=str, required=True)
# arg_parser.add_argument("--kp", type=int, default=32)
# arg_parser.add_argument("--vin", type=float, default=15.0)
# arg_parser.add_argument("--id", type=int, required=True)
# args = arg_parser.parse_args()

class Args:
    mass = 1.084
    # mass = 0.546
    length = 0.15
    # length = 0.1
    port = "/dev/ttyUSB0"
    logdir = "data/mujoco_sts3215/raw"
    # trajectory = "sin_sin"
    # trajectory = "up_and_down"
    # trajectory = "lift_and_drop"
    trajectory = "sin_time_square"
    # trajectory = "brutal"
    motor = "sts3215"
    kp = 32
    vin = 7.4
    id = 0

args = Args()


os.makedirs(args.logdir, exist_ok=True)

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

motors = {
    "test": (1, "sts3215"),
}

# motor = FeetechPWMControl(id=args.id)
# motor = MujocoIdentificationRig()
motor = MujocoIdentificationRig(bam=False, model_path="data/brutal_no_load_sts3215/params_m1.json")
while not motor.ready:
    time.sleep(0.1)
print("sim ready")

trajectory = trajectories[args.trajectory]

start = time.time()
while time.time() - start < 1.0:
    goal_position, torque_enable = trajectory(0)
    if torque_enable:
        motor.set_goal_position(goal_position)
        # motor.goal_position = np.rad2deg(goal_position)
        motor.enable_torque()
    else:
        motor.disable_torque()
    motor.kp = 6.55


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

    # position = np.deg2rad(motor.io.get_present_position([motor.id])[0])
    position = motor.get_present_position()

    speed = motor.get_present_velocity()

    load = 0  # TMP

    volts = 0

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
            motor.enable_torque()
        else:
            motor.disable_torque()
        torque_enable = new_torque_enable
        time.sleep(0.001)
    if torque_enable:
        # motor.goal_position = np.rad2deg(goal_position)
        motor.set_goal_position(goal_position)
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
    # motor.goal_position = np.rad2deg(goal_position)
    motor.set_goal_position(goal_position)

    time.sleep(return_dt)

# motor.goal_position = 0
motor.set_goal_position(0)
time.sleep(1)

motor.disable_torque()


# Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"

json.dump(data, open(filename, "w"))

print("done")