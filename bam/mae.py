#!/usr/bin/env python3
# Copyright 2025 Marc Duclusaud & Grégoire Passault
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Compare BAM models by mean absolute error (MAE) on a log directory.

Example::

    uv run python mae.py --params params/xl330/ --logdir data_processed/
"""

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from bam.logs import Logs
from bam.model import load_model
from bam import simulate

arg_parser = argparse.ArgumentParser(description="Compare BAM model MAEs")
arg_parser.add_argument("--params", type=str, required=True,
                        help="Directory containing *.json param files")
arg_parser.add_argument("--logdir", type=str, required=True,
                        help="Directory containing log files")
arg_parser.add_argument("--reset_period", type=float, default=None,
                        help="Reset period for simulation rollouts (s)")
arg_parser.add_argument("--sort", action="store_true", default=False,
                        help="Sort bars by MAE (default: keep evaluation order)")
arg_parser.add_argument("--no-sort", dest="sort", action="store_false")
arg_parser.add_argument("--json", type=str, default=None,
                        help="Write results to this JSON file instead of plotting")
arg_parser.add_argument("--mujoco", action="store_true",
                        help="Use the MuJoCo simulator backend instead of the reference one")
args = arg_parser.parse_args()

if args.mujoco:
    # Imported lazily so the default (reference) backend doesn't require MuJoCo.
    from bam import mujoco as mujoco_backend

# ── Load logs ─────────────────────────────────────────────────────────────────
logs = Logs(args.logdir)
print(f"Loaded {len(logs.logs)} logs from {args.logdir}")

# ── Discover param files ──────────────────────────────────────────────────────
params_dir = Path(args.params)
param_files = sorted(params_dir.glob("*.json"))
if not param_files:
    raise FileNotFoundError(f"No *.json files found in {params_dir}")
print(f"Found {len(param_files)} param files: {[p.name for p in param_files]}")


# ── MAE computation ───────────────────────────────────────────────────────────
def compute_mae(model, log: dict) -> float:
    if args.mujoco:
        simulator = mujoco_backend.Simulator(model)
        positions, _, _ = simulator.rollout_log(log, reset_period=args.reset_period)
    else:
        simulator = simulate.Simulator(model)
        positions, _, _ = simulator.rollout_log(
            log, reset_period=args.reset_period, simulate_control=True
        )
    log_positions = np.array([entry["position"] for entry in log["entries"]])
    return float(np.mean(np.abs(np.array(positions) - log_positions)))


results = {}  # name → list of per-log MAEs

for param_file in param_files:
    try:
        model = load_model(str(param_file))
    except Exception as e:
        print(f"  {'[SKIP] ' + param_file.stem:30s} ({e})")
        continue
    label = param_file.stem  # e.g. "m6", "m4", ...
    print(f"  {label:30s}", end="", flush=True)

    maes = []
    for log in logs.logs:
        maes.append(compute_mae(model, log))

    mean_mae = float(np.mean(maes))
    std_mae  = float(np.std(maes))
    results[label] = {"mean": mean_mae, "std": std_mae, "per_log": maes}
    print(f"MAE = {mean_mae*1000:.2f} ± {std_mae*1000:.2f} mrad")


# ── JSON output ───────────────────────────────────────────────────────────────
if args.json is not None:
    with open(args.json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Wrote results for {len(results)} models to {args.json}")
    raise SystemExit(0)


# ── Box plot ──────────────────────────────────────────────────────────────────
labels  = list(results.keys())
per_log = [np.array(results[k]["per_log"]) * 1000 for k in labels]   # → mrad
means   = np.array([results[k]["mean"] for k in labels]) * 1000
medians = np.array([np.median(d) for d in per_log])

if args.sort:
    order   = np.argsort(means)
    labels  = [labels[i] for i in order]
    per_log = [per_log[i] for i in order]
    means   = means[order]
    medians = medians[order]

fig, ax = plt.subplots(figsize=(max(6, len(labels) * 0.9 + 1), 5))
positions = np.arange(1, len(labels) + 1)
bp = ax.boxplot(
    per_log,
    positions=positions,
    widths=0.6,
    showmeans=True,
    meanline=True,
    patch_artist=True,
    medianprops=dict(color="black"),
    meanprops=dict(color="firebrick", linestyle="--"),
)
for patch in bp["boxes"]:
    patch.set_facecolor("steelblue")
    patch.set_alpha(0.6)
    patch.set_edgecolor("black")
    patch.set_linewidth(0.7)

# Custom annotation: median MAE next to each box, at the median line's level.
for pos, median in zip(positions, medians):
    ax.text(pos + 0.35, median, f"{median:.1f}",
            ha="left", va="center", fontsize=8, color="black")

ax.set_xticks(positions, labels)
ax.set_ylabel("MAE [mrad]")
ax.set_xlabel("Model")
ax.set_title(f"Model comparison — {len(logs.logs)} logs from {Path(args.logdir).name}")
ax.grid(axis="y", linestyle="--", alpha=0.5)
plt.tight_layout()
plt.show()
