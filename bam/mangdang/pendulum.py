# Copyright 2026 MangDang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Write a log's measured pendulum geometry as BAM's single-length pendulum.

:class:`bam.testbench.Pendulum` has a point mass at ``length`` on a uniform arm
of the same length:

    J = (mass + arm_mass / 3) * length**2
    tau_g = (mass + arm_mass / 2) * g * length * sin(q)

A payload whose centre sits beyond the arm's end, with its own inertia, does
not fit that form directly. With the measured geometry (``geometry_*`` keys,
written by :mod:`bam.mangdang.record`)

    L = arm_length + payload_offset
    G = payload_mass * L + arm_mass * arm_length / 2
    I = payload_mass * L**2 + arm_mass * arm_length**2 / 3 + payload_inertia

the log gets ``length = L``, ``arm_mass = 6 (G / L - I / L**2)`` and
``mass = G / L - arm_mass / 2``, which give exactly G and I. When that
``arm_mass`` would be negative (a large payload inertia), the equivalent point
mass ``G**2 / I`` at ``I / G`` is written instead. A bare arm is written as
itself; a log with neither arm nor payload gets a negligible point load (BAM's
MuJoCo pendulum divides by the total mass).

Run it on processed logs, after ``bam.process``::

    python -m bam.mangdang.pendulum --logdir processed

It rewrites ``mass``, ``arm_mass`` and ``length`` of every log in place from
its ``geometry_*`` keys, so running it twice gives the same result.
"""

import argparse
import glob
import json
import os

GEOMETRY_KEYS = (
    "payload_mass",
    "arm_mass",
    "arm_length",
    "payload_offset",
    "payload_inertia",
)
EMPTY = {"mass": 0.0, "arm_mass": 1e-6, "length": 0.01}


def fold(geometry: dict) -> dict:
    """Return BAM's ``mass``, ``arm_mass`` and ``length`` for a measured geometry."""
    p = geometry["payload_mass"]
    a = geometry["arm_mass"]
    la = geometry["arm_length"]
    off = geometry.get("payload_offset", 0.0)
    ip = geometry.get("payload_inertia", 0.0)
    if p == 0 and a == 0:
        return dict(EMPTY)
    if p == 0:
        if ip:
            raise ValueError("payload_inertia without a payload")
        return {"mass": 0.0, "arm_mass": a, "length": la}
    L = la + off
    G = p * L + a * la / 2
    I = p * L**2 + a * la**2 / 3 + ip
    arm_mass = 6 * (G / L - I / L**2)
    mass = G / L - arm_mass / 2
    if arm_mass < 0:
        return {"mass": G**2 / I, "arm_mass": 0.0, "length": I / G}
    if mass < 0:
        raise ValueError(f"geometry not representable: mass {mass}, arm_mass {arm_mass}")
    return {"mass": mass, "arm_mass": arm_mass, "length": L}


def fold_log(data: dict) -> dict:
    """Rewrite a log's ``mass``, ``arm_mass`` and ``length`` from its ``geometry_*`` keys."""
    missing = [k for k in GEOMETRY_KEYS if f"geometry_{k}" not in data]
    if missing:
        raise KeyError(f"missing geometry_{missing[0]}")
    data.update(fold({k: data[f"geometry_{k}"] for k in GEOMETRY_KEYS}))
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--logdir", type=str, required=True)
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.logdir, "*.json")))
    for filename in files:
        with open(filename) as f:
            data = json.load(f)
        try:
            fold_log(data)
        except KeyError as exc:
            raise SystemExit(f"{filename}: {exc.args[0]}")
        with open(filename, "w") as f:
            json.dump(data, f)
    print(f"rewrote {len(files)} logs in {args.logdir}")


if __name__ == "__main__":
    main()
