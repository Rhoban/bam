# Copyright 2026 BAM Project
# Licensed under the Apache License, Version 2.0 (the "License");

import argparse
import os
import time

from bam.trajectory import trajectories as available_trajectories

arg_parser = argparse.ArgumentParser(description="Batch record all BAM trajectories for Feetech HLS servo")
arg_parser.add_argument("--mass", type=float, required=True, help="Pendulum mass (kg)")
arg_parser.add_argument(
    "--arm-mass",
    type=float,
    required=True,
    help="Mass of the swinging arm itself [kg]; both the gravity and inertia "
         "terms scale with it, so it must be measured, not assumed.",
)
arg_parser.add_argument("--length", type=float, required=True, help="Pendulum length (m)")
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port")
arg_parser.add_argument("--id", type=int, default=1, help="Servo ID")
arg_parser.add_argument("--logdir", type=str, required=True, help="Output directory for raw logs")
arg_parser.add_argument(
    "--vin",
    type=float,
    default=None,
    help="Supply voltage (V). Default: each run reads the servo's own bus voltage.",
)
arg_parser.add_argument(
    "--kps",
    type=str,
    default="2,3,4,6",
    help="Comma-separated proportional gains in real A/rad, INTEGER values only: "
         "bam.fit's --validation_kp is declared int, so a fractional gain could "
         "never be held out. These are the old command-unit set [8,12,16,24] "
         "rescaled by 0.241, rounded - the same physical loop as before.",
)
arg_parser.add_argument(
    "--angle-offset",
    type=float,
    default=0.0,
    help="Added to the LOGGED angles (position and goal) [rad]; see record.py. "
         "0.179 for this rig: the vertical as measured by the friction-cancelling "
         "sweep, which keeps the fitted q_offset inside bam.model's +-0.1 bound.",
)
arg_parser.add_argument(
    "--max-amps",
    type=float,
    default=0.36,
    help="Current saturation limit (real A). 0.36 A is the old 1.5 command-unit "
         "clamp rescaled; the servo's measured maximum is 1.37 A.",
)
arg_parser.add_argument(
    "--damping",
    type=float,
    default=0.08,
    help="Damping gain. 0.08 measured best on this rig (9%% clamp clipping, "
         "4x less tracking error than 0.5, zeta = 0.94).",
)
arg_parser.add_argument("--motor", type=str, default="feetech_hls", help="Motor identifier")
args = arg_parser.parse_args()

kps = [float(x) for x in args.kps.split(",") if x.strip()]
trajectory_names = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]

unknown = [name for name in trajectory_names if name not in available_trajectories]
if unknown:
    raise ValueError(
        f"Unknown trajectories: {unknown}. Available: {sorted(available_trajectories)}"
    )

command_base = (
    f"python3 -m bam.feetech_hls.record "
    f"--mass {args.mass} --arm-mass {args.arm_mass} --length {args.length} "
    f"--port {args.port} --id {args.id} --logdir {args.logdir} "
    f"--damping {args.damping} --motor {args.motor} --max-amps {args.max_amps} "
    f"--angle-offset {args.angle_offset}"
)
# Only pin the supply voltage when the caller knows it: otherwise the recorder
# reads it from the servo, which is what the model's voltage window needs.
if args.vin is not None:
    command_base += f" --vin {args.vin}"

print(f"==================================================")
print(f" Starting Batch Recording for Motor: {args.motor}")
print(f" Kp values: {kps}")
print(f" Trajectories: {trajectory_names}")
print(f"==================================================")

for kp in kps:
    for trajectory in trajectory_names:
        print(f"\n>>> Running Kp={kp}, Trajectory={trajectory}...")
        cmd = f"{command_base} --kp {kp} --trajectory {trajectory}"
        ret = os.system(cmd)
        if ret != 0:
            print(f"Warning: command exited with code {ret}")
        time.sleep(2.0)
