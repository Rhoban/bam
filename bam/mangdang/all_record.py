# Copyright 2026 MangDang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Batch runner that sweeps P-gains and trajectories for the MD-300-150.

Mirrors ``bam/feetech/all_record.py``: for every (kp, trajectory) pair it
prints/announces the current run and shells out to ``bam.mangdang.record``.

``--vin`` defaults to 12 V. The module does not measure its supply, so set it
to the supply's real output. One gain is held out for validation
(``bam.fit --validation_kp``), so keep at least four gains.

Each child run is checked: a non-zero exit or a missing log aborts the sweep.
"""

import argparse
import glob
import os
import subprocess
import sys
import time

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument(
    "--arm-mass",
    "--arm_mass",
    dest="arm_mass",
    type=float,
    default=0.0,
    help="arm mass [kg]",
)
arg_parser.add_argument("--motor", type=str, default="md300")
arg_parser.add_argument("--port", type=str, default=None)
arg_parser.add_argument("--id", type=int, default=1, help="bus id 1..12")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--vin", type=float, default=12.0)
arg_parser.add_argument("--speak", action="store_true")
arg_parser.add_argument(
    "--dry-run",
    action="store_true",
    help="exercise the sweep without hardware",
)
arg_parser.add_argument(
    "--kps",
    type=str,
    default=None,
    help="comma-separated P gains (default: the KPS list below)",
)
arg_parser.add_argument(
    "--trajectories",
    type=str,
    default=None,
    help="comma-separated trajectory names (default: the list below)",
)
arg_parser.add_argument("--kd", type=float, default=400.0)
arg_parser.add_argument(
    "--shuffle",
    action="store_true",
    help="record the kps in a random order (seeded by --seed) so gain is not confounded "
    "with the housing temperature; the order is printed and appended to <logdir>/order.txt",
)
arg_parser.add_argument(
    "--seed", type=int, default=None, help="seed for --shuffle (default: time)"
)
arg_parser.add_argument(
    "--extra",
    type=str,
    default="",
    help="extra arguments passed verbatim to bam.mangdang.record, e.g. "
    "'--zero-dd 1540 --pwm-scale 0.001 --supply-v 12'",
)
args = arg_parser.parse_args()

#: P-gain sweep. At least one value is held out for validation in bam.fit.
KPS = [40, 80, 120, 160]

#: Trajectory sweep (names from bam.trajectory and bam.mangdang.trajectories).
trajectories = [
    "lift_and_drop",
    "sin_sin",
    "sin_time_square",
    "up_and_down",
]

if args.kps:
    KPS = [float(x) for x in args.kps.split(",")]
if args.shuffle:
    import random

    seed = args.seed if args.seed is not None else int(time.time())
    random.Random(seed).shuffle(KPS)
    os.makedirs(args.logdir, exist_ok=True)
    with open(os.path.join(args.logdir, "order.txt"), "a") as f:
        f.write(
            f"{time.strftime('%Y-%m-%d %H:%M:%S')} mass {args.mass} length {args.length} "
            f"seed {seed} kps {' '.join(str(int(k)) for k in KPS)}\n"
        )
    print(f"kp order (seed {seed}): {KPS}")
if args.trajectories:
    trajectories = [x.strip() for x in args.trajectories.split(",") if x.strip()]


def run_all() -> None:
    """Sweep every (kp, trajectory) pair by invoking ``bam.mangdang.record``."""
    command_base = [
        sys.executable,
        "-m",
        "bam.mangdang.record",
        "--mass",
        str(args.mass),
        "--arm-mass",
        str(args.arm_mass),
        "--length",
        str(args.length),
        "--id",
        str(args.id),
        "--motor",
        args.motor,
        "--vin",
        str(args.vin),
        "--logdir",
        args.logdir,
        "--kd",
        str(args.kd),
    ]
    if args.port:
        command_base += ["--port", args.port]
    if args.dry_run:
        command_base.append("--dry-run")
    if args.extra:
        command_base += args.extra.split()

    for kp in KPS:
        for trajectory in trajectories:
            sentence = f"Kp {kp}, trajectory {trajectory.replace('_', ' ')}"
            print(sentence)

            if args.speak:
                from gtts import gTTS

                myobj = gTTS(text=sentence, lang="en", slow=False)
                myobj.save("/tmp/message.mp3")
                os.system("mpg321 /tmp/message.mp3")

            command = command_base + [
                "--kp",
                str(int(kp) if float(kp).is_integer() else kp),
                "--trajectory",
                trajectory,
            ]
            result = subprocess.run(command, check=False)

            if result.returncode != 0:
                raise SystemExit(
                    f"run failed (exit {result.returncode}): {' '.join(command)}\n"
                    "Aborting the sweep."
                )

            produced = glob.glob(f"{args.logdir}/*.json")
            if not produced:
                raise SystemExit(
                    f"no log produced in {args.logdir} for kp={kp}, "
                    f"trajectory={trajectory}; aborting"
                )

            if trajectory == "sin_time_square":
                time.sleep(3)


if __name__ == "__main__":
    run_all()
