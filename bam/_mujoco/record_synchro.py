import json
import datetime
import os
import numpy as np
# import argparse
import time
from bam.trajectory import *
import mujoco
import mujoco.viewer
from bam.model import load_model
from bam.mujoco import MujocoController

class Args:
    mass = 1.084
    # mass = 0.546
    length = 0.15
    # length = 0.1
    port = "/dev/ttyUSB0"
    logdir = "data/mujoco_sts3215_2/raw"
    # trajectory = "sin_sin"
    # trajectory = "up_and_down"
    # trajectory = "lift_and_drop"
    # trajectory = "sin_time_square"
    trajectory = "brutal"
    motor = "sts3215"
    kp = 32
    vin = 7.4
    id = 0
    bam = False
    model_path = "data/brutal_no_load_sts3215/params_m1.json"

args = Args()

if args.bam and args.model_path is None:
    print("Bam mode requires model_path to be set")
    exit()


if args.bam:
    mj_model = mujoco.MjModel.from_xml_path(
        "bam/mujoco_identification_rig/assets/identification_rig_0_150m_1kg/scene_motor.xml"
    )
else:
    mj_model = mujoco.MjModel.from_xml_path(
        "bam/mujoco_identification_rig/assets/identification_rig_0_150m_1kg/scene.xml"
    )

mj_model.opt.timestep = 0.002
mj_data = mujoco.MjData(mj_model)
mujoco.mj_step(mj_model, mj_data)
control_decimation = 1

if args.bam:
    sts3215_model = load_model(args.model_path)
    mujoco_controller = MujocoController(
        sts3215_model, "sts3215", mj_model, mj_data
    )

os.makedirs(args.logdir, exist_ok=True)

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")


trajectory = trajectories[args.trajectory]

with mujoco.viewer.launch_passive(
    mj_model, mj_data, show_left_ui=False, show_right_ui=False
) as viewer:
    start = time.time()
    counter = 0
    while time.time() - start < 1.0:
        counter += 1
        goal_position, torque_enable = trajectory(0)
        if torque_enable:
            if counter % control_decimation == 0:
                if args.bam:
                    mujoco_controller.update(goal_position)
                else:
                    mj_data.ctrl = goal_position

            # motor.enable_torque()
        else:
            pass
            # motor.disable_torque()
        # motor.kp = 6.55
        mujoco.mj_step(mj_model, mj_data)
        # time.sleep(mj_model.opt.timestep)
        # time.sleep(0.001)


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
        position = mj_data.qpos[0]

        speed = mj_data.qvel[0]

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
        counter += 1
        t = time.time() - start
        goal_position, new_torque_enable = trajectory(t)
        if new_torque_enable != torque_enable:
            # if new_torque_enable:
            #     motor.enable_torque()
            # else:
            #     motor.disable_torque()
            torque_enable = new_torque_enable
            time.sleep(0.001)
        if torque_enable:
            # motor.goal_position = np.rad2deg(goal_position)
            if counter % control_decimation == 0:
                if args.bam:
                    mujoco_controller.update(goal_position)
                else:
                    mj_data.ctrl = goal_position
            # motor.set_goal_position(goal_position)
            viewer.sync()
            mujoco.mj_step(mj_model, mj_data)
            # time.sleep(0.001)
            # time.sleep(mj_model.opt.timestep)

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
        counter += 1
        if goal_position > 0:
            goal_position = max(0, goal_position - max_variation)
        else:
            goal_position = min(0, goal_position + max_variation)
        # motor.goal_position = np.rad2deg(goal_position)
        if counter % control_decimation == 0:
            if args.bam:
                mujoco_controller.update(goal_position)
            else:
                mj_data.ctrl = goal_position
        # motor.set_goal_position(goal_position)

        viewer.sync()
        mujoco.mj_step(mj_model, mj_data)
        # time.sleep(0.001)
        # time.sleep(mj_model.opt.timestep)

    # motor.goal_position = 0
    # motor.set_goal_position(0)
    time.sleep(1)

    # motor.disable_torque()


# Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"

json.dump(data, open(filename, "w"))

print("done")