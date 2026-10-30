# Copyright 2026 BAM Project
# Licensed under the Apache License, Version 2.0 (the "License");

import argparse
import subprocess
import sys
import time

from bam.trajectory import trajectories


parser = argparse.ArgumentParser(
    description="Batch-record BAM trajectories with an HLS2915 in Position mode"
)
parser.add_argument("--mass", type=float, required=True, help="Pendulum mass [kg]")
parser.add_argument("--arm-mass", type=float, required=True, help="Arm mass [kg]")
parser.add_argument("--length", type=float, required=True, help="Pendulum length [m]")
parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
parser.add_argument("--id", type=int, default=1, help="Servo ID")
parser.add_argument("--logdir", required=True, help="Raw-log output directory")
parser.add_argument("--vin", type=float, default=None, help="Optional fixed supply voltage [V]")
parser.add_argument(
    "--kps",
    default="4,8,12,16",
    help="Comma-separated HLS Position-mode firmware Kp registers [0, 255]",
)
parser.add_argument("--goal-min", type=float, default=-1.20, help="Lower target clamp [rad]")
parser.add_argument("--goal-max", type=float, default=1.20, help="Upper target clamp [rad]")
parser.add_argument("--cooldown-temp", type=float, default=42.0, help="Maximum start temperature [C]")
parser.add_argument("--motor", default="feetech_hls2915", help="Motor identifier")
args = parser.parse_args()

kps = [int(value) for value in args.kps.split(",") if value.strip()]
if any(not 0 <= kp <= 0xFF for kp in kps):
    raise ValueError(f"Kp values must be in [0, 255], got {kps}")

trajectory_names = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]
unknown = [name for name in trajectory_names if name not in trajectories]
if unknown:
    raise ValueError(f"Unknown trajectories: {unknown}")

base = [
    sys.executable,
    "-m",
    "bam.feetech_hls.record",
    "--mass", str(args.mass),
    "--arm-mass", str(args.arm_mass),
    "--length", str(args.length),
    "--port", args.port,
    "--id", str(args.id),
    "--logdir", args.logdir,
    "--goal-min", str(args.goal_min),
    "--goal-max", str(args.goal_max),
    "--cooldown-temp", str(args.cooldown_temp),
    "--motor", args.motor,
]
if args.vin is not None:
    base.extend(["--vin", str(args.vin)])

print("=" * 50)
print(f"Position-mode batch recording: {args.motor}")
print(f"Kp registers: {kps}")
print(f"Trajectories: {trajectory_names}")
print("=" * 50)

for kp in kps:
    for trajectory in trajectory_names:
        command = base + ["--kp", str(kp), "--trajectory", trajectory]
        print(f"\n>>> Kp={kp}, trajectory={trajectory}")
        subprocess.run(command, check=True)
        time.sleep(2.0)
