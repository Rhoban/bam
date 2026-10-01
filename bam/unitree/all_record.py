# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Record a full J288 identification session for one pendulum setup.

Runs :mod:`bam.unitree.record` once per recording: the BAM position
trajectories at each gain pair, then the torque-only, speed-mode and
damping-mode profiles. Each recording waits for the winding to cool first, so a
session needs no supervision beyond keeping clear of the arm::

    python -m bam.unitree.all_record --port /dev/cu.usbmodemXXXX --zero-file zero.json \\
        --mass 0.2 --arm-mass 0.02 --length 0.075 --logdir data_raw/m200_l75

Use ``--dry-run`` to print the plan without moving anything, and
``--skip-existing`` to resume an interrupted session: a recording is skipped
when the log directory already has a complete (not aborted) log for the same
profile and gains. Torque profile
amplitudes default to a quarter (chirp) and half (steps) of the setup's peak
gravity torque.
"""

import argparse
import glob
import os
import shlex
import subprocess
import sys

from bam.unitree.digital_servo import sent_gains

G = 9.80665

DEFAULT_GAINS = ["0.97:0.02", "0.97:0.05", "2.01:0.02", "2.01:0.05"]
DEFAULT_TRAJECTORIES = ["sin_time_square", "sin_sin", "lift_and_drop", "up_and_down"]
DEFAULT_TORQUE_PROFILES = ["torque_chirp", "torque_steps"]
DEFAULT_DAMPING_KD = ["0.02", "0.05"]


def plan(args) -> list[list[str]]:
    gravity = (args.mass + args.arm_mass / 2) * G * args.length
    # The chirp sweeps through the pendulum's natural frequency, where it builds
    # up: a quarter of the gravity torque for it, half for the steps.
    steps_amplitude = args.torque_amplitude or round(0.5 * gravity, 3)
    chirp_amplitude = args.torque_amplitude or round(0.25 * gravity, 3)
    runs = []
    for pair in args.gains:
        kp, kd = pair.split(":")
        for trajectory in args.trajectories:
            runs.append(["--profile", trajectory, "--kp", kp, "--kd", kd])
    for profile in args.torque_profiles:
        amplitude = chirp_amplitude if profile == "torque_chirp" else steps_amplitude
        runs.append(["--profile", profile, "--amplitude", str(amplitude)])
    for kd in args.damping_kd:
        runs.append(
            [
                "--profile",
                "speed_sine",
                "--kd",
                kd,
                "--amplitude",
                str(args.speed_amplitude),
            ]
        )
        runs.append(["--profile", "lift_and_damp", "--kp", args.lift_kp, "--kd", kd])
    return runs


def already_recorded(logdir: str, run: list[str]) -> str | None:
    """Path of a complete log for this run, as record.py names them."""
    opts = dict(zip(run[::2], run[1::2]))
    kp, kd = sent_gains(float(opts.get("--kp", 0.97)), float(opts.get("--kd", 0.02)))
    # record.py names logs <YYYY-MM-DD_HHhMMmSS>_<profile>_kp.._kd...json; anchor
    # on the end of the time so "steps" does not match "torque_steps".
    pattern = f"*m[0-9][0-9]_{opts['--profile']}_kp{kp:.2f}_kd{kd:.3f}.json"
    matches = sorted(glob.glob(os.path.join(logdir, pattern)))
    return matches[-1] if matches else None


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", required=True)
    p.add_argument("--id", type=int, default=0)
    p.add_argument("--zero-file", required=True)
    p.add_argument("--mass", type=float, required=True)
    p.add_argument("--arm-mass", type=float, required=True)
    p.add_argument("--length", type=float, required=True)
    p.add_argument("--mass-inertia", type=float, default=0.0)
    p.add_argument("--logdir", required=True)
    p.add_argument(
        "--gains", nargs="+", default=DEFAULT_GAINS, help="kp:kd pairs, output side"
    )
    p.add_argument("--trajectories", nargs="+", default=DEFAULT_TRAJECTORIES)
    p.add_argument("--torque-profiles", nargs="*", default=DEFAULT_TORQUE_PROFILES)
    p.add_argument(
        "--torque-amplitude",
        type=float,
        help="[N.m] for both torque profiles (default: a quarter of the gravity torque for the chirp, half for the steps)",
    )
    p.add_argument("--damping-kd", nargs="*", default=DEFAULT_DAMPING_KD)
    p.add_argument(
        "--speed-amplitude", type=float, default=2.0, help="speed_sine [rad/s]"
    )
    p.add_argument("--lift-kp", default="2.01", help="kp for the lift in lift_and_damp")
    p.add_argument("--wait-below", type=float, default=45.0)
    p.add_argument("--note", default="")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument(
        "--skip-existing",
        action="store_true",
        help="skip recordings that already have a complete log in --logdir",
    )
    args = p.parse_args()

    common = [
        "--port",
        args.port,
        "--id",
        str(args.id),
        "--zero-file",
        args.zero_file,
        "--mass",
        str(args.mass),
        "--arm-mass",
        str(args.arm_mass),
        "--length",
        str(args.length),
        "--mass-inertia",
        str(args.mass_inertia),
        "--logdir",
        args.logdir,
        "--wait-below",
        str(args.wait_below),
        "--note",
        args.note,
    ]
    runs = plan(args)
    print(f"{len(runs)} recordings of about 6 s, plus cooling between them")
    failed = []
    for k, run in enumerate(runs, 1):
        cmd = [sys.executable, "-m", "bam.unitree.record", *common, *run]
        print(f"[{k}/{len(runs)}] {' '.join(run)}", flush=True)
        if args.skip_existing and (done := already_recorded(args.logdir, run)):
            print(f"  already recorded: {os.path.basename(done)}")
            continue
        if args.dry_run:
            print("  " + shlex.join(cmd[1:]))
            continue
        if subprocess.run(cmd).returncode != 0:
            failed.append(" ".join(run))
    if failed:
        print(f"{len(failed)} recordings failed to run:")
        for run in failed:
            print(f"  {run}")


if __name__ == "__main__":
    main()
