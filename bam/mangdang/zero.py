# Copyright 2026 MangDang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Measure the pendulum vertical of an MD-300-150 rig by two-sided settling.

A single torque-off rest is not the vertical: static friction stops the arm
anywhere within ``asin(tau_s / (m g l))`` of it, on the side it came from
(7-11 deg for a 0.26 Nm load). Settling from both sides and taking the
midpoint cancels that offset. The result is the ``--zero-dd`` the recorder
maps ``0 rad`` to, so logs are recorded around the true vertical whatever
spline tooth the arm went on.

    python -m bam.mangdang.zero --id 8 --logdir data_raw [--reps 3]

Mount the arm with a payload heavy enough to fall on its own (never the bare
arm) before running this. The servo is lifted to +/-60 deg from the current
zero estimate with a modest current cap, released, and read after the swing
has died out; the arm must be free to swing on both sides.
"""

import argparse
import json
import math
import os
import time

import numpy as np

from bam.mangdang.spi import PARAM, SpiMD300IO, pick_port

#: The robot's loop (control_mode 1), written in RAM before lifting and restored
#: afterwards: modules do not all boot into it.
ROBOT = {"control_mode": 1.0, "kp_current": 0.0, "kff_current": 0.001, "pwm_scale": 0.001,
         "ki_position": 0.0, "max_pwm_duty_cycle": 0.99}

parser = argparse.ArgumentParser(description="Two-sided rig zero for the MD-300-150")
parser.add_argument("--id", type=int, required=True, help="bus id 1..12")
parser.add_argument("--port", type=str, default=None)
parser.add_argument("--baud", type=int, default=921600)
parser.add_argument("--logdir", type=str, required=True, help="where zero.json goes")
parser.add_argument("--reps", type=int, default=3, help="settles per side")
parser.add_argument("--lift-deg", type=float, default=60.0)
parser.add_argument("--cur", type=int, default=600, help="current cap while lifting [mA]")
parser.add_argument("--kp", type=float, default=80.0)
parser.add_argument("--settle", type=float, default=4.0, help="seconds after release")
parser.add_argument("--start-dd", type=float, default=None, help="initial zero guess (default: pot centre)")
parser.add_argument("--dry-run", action="store_true")
args = parser.parse_args()


def main() -> None:
    port = None if args.dry_run else pick_port(args.port)
    io = SpiMD300IO(port, baud=args.baud, dry_run=args.dry_run, zero_dd=args.start_dd, cur_ma=args.cur)
    servo = args.id
    flash = None
    if not args.dry_run:
        io.sync_scale_from_board(servo)
        flash = io.dump_params(servo)
        io.set_P_coefficient({servo: args.kp})
        io.set_D_coefficient({servo: 0.0})
        for name, value in ROBOT.items():
            io.set_param(servo, PARAM[name], value)
        active = io.dump_params(servo)
        assert abs(active["control_mode"] - 1.0) < 1e-6, active
    guess = io.zero_dd
    lift = args.lift_deg * 10.0
    rests = {"+": [], "-": []}
    try:
        for rep in range(args.reps):
            for side, sign in (("+", 1.0), ("-", -1.0)):
                target = guess + sign * lift
                # Ramp to the lifted pose so the arm is not thrown, then release.
                current = io.ping(servo)["pos_dd"]
                for step in np.linspace(current, target, 40):
                    io.set_pos_dd(servo, int(step))
                    time.sleep(0.03)
                time.sleep(0.8)
                io.set_idle(servo)
                time.sleep(args.settle)
                # The AT32 answers with the state at the previous frame, so
                # the first reply after the settle is the release position.
                io.ping(servo)
                samples = [io.ping(servo)["pos_dd"] for _ in range(20)]
                spread = max(samples) - min(samples)
                if spread > 5:
                    time.sleep(args.settle)
                    io.ping(servo)
                    samples = [io.ping(servo)["pos_dd"] for _ in range(20)]
                    spread = max(samples) - min(samples)
                rest = float(np.mean(samples))
                rests[side].append(rest)
                print(
                    f"rep {rep + 1} side {side}: rest {rest:.1f} dd "
                    f"({(rest - guess) * 0.1:+.2f} deg from guess), spread "
                    f"{spread} dd{'  STILL MOVING' if spread > 5 else ''}"
                )
    finally:
        try:
            io.set_idle(servo)
            if flash is not None:
                for name in ROBOT:
                    io.set_param(servo, PARAM[name], flash[name])
                io.set_P_coefficient({servo: flash["kp_position"]})
                io.set_D_coefficient({servo: flash["kd_position"]})
        finally:
            io.close()

    plus, minus = np.mean(rests["+"]), np.mean(rests["-"])
    zero = (plus + minus) / 2.0
    band = (plus - minus) / 2.0
    result = {
        "servo_id": servo,
        "zero_dd": zero,
        "rest_plus_dd": rests["+"],
        "rest_minus_dd": rests["-"],
        "stiction_half_band_deg": band * 0.1,
        "spread_plus_dd": float(np.ptp(rests["+"])) if rests["+"] else 0.0,
        "spread_minus_dd": float(np.ptp(rests["-"])) if rests["-"] else 0.0,
        "scale_dd": io.scale_dd,
        "offset_from_centre_deg": (zero - io.scale_dd / 2.0) * 0.1,
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    os.makedirs(args.logdir, exist_ok=True)
    path = os.path.join(args.logdir, "zero.json")
    with open(path, "w") as f:
        json.dump(result, f, indent=2)
    print(
        f"zero {zero:.1f} dd = {result['offset_from_centre_deg']:+.2f} deg from the pot centre; "
        f"stiction half-band {band * 0.1:.2f} deg; side spreads "
        f"{result['spread_plus_dd']:.0f}/{result['spread_minus_dd']:.0f} dd -> {path}"
    )
    print(f"pass --zero-dd {zero:.0f} to bam.mangdang.record / all_record --extra")


if __name__ == "__main__":
    main()
