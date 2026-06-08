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
arg_parser.add_argument("--sort", action="store_true", default=True,
                        help="Sort bars by MAE (default: True)")
arg_parser.add_argument("--no-sort", dest="sort", action="store_false")
args = arg_parser.parse_args()

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


# ── Bar plot ──────────────────────────────────────────────────────────────────
labels = list(results.keys())
means  = np.array([results[k]["mean"] for k in labels]) * 1000   # → mrad
stds   = np.array([results[k]["std"]  for k in labels]) * 1000

if args.sort:
    order  = np.argsort(means)
    labels = [labels[i] for i in order]
    means  = means[order]
    stds   = stds[order]

fig, ax = plt.subplots(figsize=(max(6, len(labels) * 0.9 + 1), 5))
bars = ax.bar(labels, means, yerr=stds, capsize=4,
              color="steelblue", edgecolor="black", linewidth=0.7)

for bar, val, std in zip(bars, means, stds):
    ax.text(bar.get_x() + bar.get_width() / 2, val + std + 0.3,
            f"{val:.1f}", ha="center", va="bottom", fontsize=8)

ax.set_ylabel("MAE [mrad]")
ax.set_xlabel("Model")
ax.set_title(f"Model comparison — {len(logs.logs)} logs from {Path(args.logdir).name}")
ax.grid(axis="y", linestyle="--", alpha=0.5)
plt.tight_layout()
plt.show()
