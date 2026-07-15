# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Animated version of ``bam.plot``.

Renders, side by side, an animated pendulum (left) and the identification
plots (right).  The plots are revealed in real time as the simulation plays.
The pendulum shows the *simulated* state (solid) overlaid on the *measured*
state (alpha=0.5).

When several models are given (e.g. ``m1 m6``), they play *sequentially*: each
model's phase animates the pendulum and grows that model's curve on the plots,
while previously played models stay on screen.  The final frame is identical to
what ``bam.plot`` renders statically.
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

import json

from .model import load_model, DummyModel
from .actuators import actuators
from . import simulate

# --- Flat "poster" palette (matches the reference illustration) ---
COL_ARM = "#c9a94e"
COL_MASS = "#8a8a93"
COL_RING = "#26262f"
COL_PILLAR = "#d8cba6"
COL_FLOOR = "#6f6f7a"
COL_PIVOT = "#26262f"

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument(
    "--log", type=str, required=True, help="Path to a single processed JSON log."
)
arg_parser.add_argument("--params", type=str, default=["params.json"], nargs="+")
arg_parser.add_argument("--actuator", type=str, required=True)
arg_parser.add_argument("--reset_period", default=None, type=float)
arg_parser.add_argument(
    "--fps",
    type=int,
    default=25,
    help="Frames per second (drives both playback pacing and output). "
    "The log is downsampled so that, at --speed 1, playback is real time.",
)
arg_parser.add_argument(
    "--speed",
    type=float,
    default=1.0,
    help="Playback speed factor (1.0 = real time).",
)
arg_parser.add_argument(
    "--angle-offset",
    dest="angle_offset",
    type=float,
    default=0.0,
    help="Offset [rad] added to the joint angle when drawing the arm "
    "(0 = arm points to the right at q=0).",
)
arg_parser.add_argument(
    "--save",
    type=str,
    default=None,
    help="Save to this path (.mp4 or .gif) instead of showing a window.",
)
args = arg_parser.parse_args()


def load_log():
    with open(args.log) as f:
        log = json.load(f)
    log["filename"] = args.log
    if "arm_mass" not in log:
        log["arm_mass"] = 0.0
    return log


def compute_series(log):
    """Reproduce the data ``bam.plot`` computes, plus per-model rollouts."""
    ts = np.arange(len(log["entries"])) * log["dt"]
    q = np.array([e["position"] for e in log["entries"]])
    goal_q = np.array([e["goal_position"] for e in log["entries"]])
    speed = np.array(
        [e["speed"] if "speed" in e else 0.0 for e in log["entries"]]
    )
    has_speed = any("speed" in e for e in log["entries"])

    # Recorded control (reference), computed exactly like bam.plot.
    dummy = DummyModel()
    dummy.set_actuator(actuators[args.actuator]())
    _, __, controls = simulate.Simulator(dummy).rollout_log(
        log, simulate_control=False
    )
    controls = np.array([0.0 if c is None else c for c in controls])
    torque_enable = np.array([e["torque_enable"] for e in log["entries"]])

    models = []
    for model_name in args.params:
        model = load_model(model_name)
        sim_q, sim_speed, sim_controls = simulate.Simulator(model).rollout_log(
            log, reset_period=args.reset_period, simulate_control=True
        )
        models.append(
            {
                "name": model.name or model_name,
                "q": np.array(sim_q),
                "speed": np.array(sim_speed),
                "controls": np.array(sim_controls, dtype=float),
            }
        )

    return {
        "ts": ts,
        "q": q,
        "goal_q": goal_q,
        "speed": speed,
        "has_speed": has_speed,
        "controls": controls,
        "torque_enable": torque_enable,
        "control_unit": dummy.actuator.control_unit(),
        "models": models,
    }


class Pendulum:
    """Draws the flat-style pendulum and updates it to a given angle.

    Convention: ``q=0`` points straight *down* (the pendulum's rest position);
    positive ``q`` swings the arm counter-clockwise.  The pivot is elevated and
    the arm hangs from it, mounted on a gallows-style stand.
    """

    def __init__(self, ax, angle_offset=0.0):
        self.ax = ax
        self.offset = angle_offset
        self.L = 1.2
        self.pivot = np.array([0.0, 0.95])

        ax.set_xlim(-1.45, 1.45)
        ax.set_ylim(-1.45, 1.45)
        ax.set_aspect("equal")
        ax.axis("off")

        px, py = self.pivot

        # Floor, vertical stand and top beam holding the pivot (static).
        ax.add_patch(
            Rectangle((-1.45, -1.45), 2.9, 0.34, color=COL_FLOOR, zorder=0)
        )
        ax.add_patch(
            FancyBboxPatch(
                (-1.30, -1.11),
                0.22,
                py + 1.20,
                boxstyle="round,pad=0,rounding_size=0.08",
                fc=COL_PILLAR,
                ec="none",
                zorder=1,
            )
        )
        ax.add_patch(
            FancyBboxPatch(
                (-1.24, py - 0.09),
                1.24 + px,
                0.18,
                boxstyle="round,pad=0,rounding_size=0.08",
                fc=COL_PILLAR,
                ec="none",
                zorder=1,
            )
        )

        # Two arms + masses: index 0 = measured (alpha), 1 = simulated (solid).
        self.arms = []
        self.masses = []
        for alpha, z in ((0.5, 2), (1.0, 3)):
            (arm,) = ax.plot(
                [], [], color=COL_ARM, lw=22, solid_capstyle="round",
                alpha=alpha, zorder=z,
            )
            mass = Circle(
                (0, 0), 0.20, fc=COL_MASS, ec=COL_RING, lw=7,
                alpha=alpha, zorder=z + 0.1,
            )
            ax.add_patch(mass)
            self.arms.append(arm)
            self.masses.append(mass)
        ax.add_patch(Circle(tuple(self.pivot), 0.06, fc=COL_PIVOT, zorder=5))

    def _tip(self, q):
        theta = q + self.offset
        # q=0 -> straight down; positive q -> counter-clockwise.
        return (
            self.pivot[0] + self.L * np.sin(theta),
            self.pivot[1] - self.L * np.cos(theta),
        )

    def set_measured(self, q):
        self._set(0, q, visible=True)

    def set_simulated(self, q, visible=True):
        self._set(1, q, visible=visible)

    def _set(self, i, q, visible):
        x, y = self._tip(q)
        self.arms[i].set_data([self.pivot[0], x], [self.pivot[1], y])
        self.arms[i].set_visible(visible)
        self.masses[i].center = (x, y)
        self.masses[i].set_visible(visible)

    def artists(self):
        return self.arms + self.masses


def build_figure(data):
    has_speed = data["has_speed"]
    # Larger fonts and thicker lines so the figure stays legible on small
    # devices / when the video is scaled down.
    plt.rcParams.update(
        {
            "font.size": 18,
            "axes.titlesize": 20,
            "axes.labelsize": 18,
            "xtick.labelsize": 15,
            "ytick.labelsize": 15,
            "legend.fontsize": 15,
            "lines.linewidth": 3.0,
            "axes.linewidth": 1.8,
            "grid.linewidth": 1.2,
        }
    )

    n_right = 3 if has_speed else 2
    fig = plt.figure(figsize=(16, 9))
    gs = GridSpec(n_right, 2, width_ratios=[1, 1.6], figure=fig)

    ax_pend = fig.add_subplot(gs[:, 0])
    pendulum = Pendulum(ax_pend, angle_offset=args.angle_offset)

    right_axes = []
    ax1 = fig.add_subplot(gs[0, 1])
    right_axes.append(ax1)
    if has_speed:
        ax2 = fig.add_subplot(gs[1, 1], sharex=ax1)
        ax3 = fig.add_subplot(gs[2, 1], sharex=ax1)
        right_axes += [ax2, ax3]
    else:
        ax2 = None
        ax3 = fig.add_subplot(gs[1, 1], sharex=ax1)
        right_axes.append(ax3)

    ts = data["ts"]
    unit = data["control_unit"]
    lines = {}

    # --- ax1: angle ---
    # Read (measured) curves are drawn in full from the start; only the
    # simulated curves are revealed progressively.
    (lines["q"],) = ax1.plot(ts, data["q"], label="q")
    (lines["goal_q"],) = ax1.plot(
        ts, data["goal_q"], label="goal_q", color="black", linestyle="--"
    )
    lines["sim_q"] = [
        ax1.plot([], [], label=f"{m['name']}_q")[0] for m in data["models"]
    ]
    ax1.set_title(
        f'{data_log["motor"]}, {data_log["trajectory"]}, '
        f'm={data_log["mass"]}, l={data_log["length"]}, k={data_log["kp"]}'
    )
    ax1.set_ylabel("angle [rad]")

    # --- ax2: speed ---
    if has_speed:
        (lines["speed"],) = ax2.plot(ts, data["speed"], label="speed")
        lines["sim_speed"] = [
            ax2.plot([], [], label=f"{m['name']}_speed")[0] for m in data["models"]
        ]
        ax2.set_ylabel("speed [rad/s]")

    # --- ax3: control ---
    (lines["control"],) = ax3.plot(ts, data["controls"], label=unit)
    lines["sim_control"] = [
        ax3.plot([], [], label=f"{m['name']}_{unit}")[0] for m in data["models"]
    ]
    # torque-off shading (static context).
    cmin = float(np.min(data["controls"])) - 0.02
    cmax = float(np.max(data["controls"])) + 0.02
    ax3.fill_between(
        ts, cmin, cmax, where=~data["torque_enable"],
        color="red", alpha=0.3, label="torque off",
    )
    ax3.set_ylabel(unit)
    ax3.set_xlabel(f"time [s] / simulator: reference")

    # Fixed limits so nothing rescales mid-animation, and vertical time cursor.
    cursors = []
    for ax in right_axes:
        ax.set_xlim(ts[0], ts[-1])
        ax.grid(True)
        ax.legend(loc="upper right")
        cursors.append(ax.axvline(ts[0], color="0.4", lw=2.2, alpha=0.0))

    _set_ylim(ax1, [data["q"], data["goal_q"]] + [m["q"] for m in data["models"]])
    if has_speed:
        _set_ylim(ax2, [data["speed"]] + [m["speed"] for m in data["models"]])
    ax3.set_ylim(cmin, cmax)

    fig.tight_layout()
    return fig, pendulum, lines, cursors


def _set_ylim(ax, arrays):
    lo = min(float(np.min(a)) for a in arrays if len(a))
    hi = max(float(np.max(a)) for a in arrays if len(a))
    pad = 0.05 * (hi - lo + 1e-9)
    ax.set_ylim(lo - pad, hi + pad)


def main():
    global data_log
    data_log = load_log()
    print(f"Animating {data_log['filename']}")
    data = compute_series(data_log)

    fig, pendulum, lines, cursors = build_figure(data)

    ts = data["ts"]
    n = len(ts)
    n_models = max(1, len(data["models"]))
    dt = data_log["dt"]
    # Downsample so that, at --speed 1, one output frame advances real time
    # by 1/fps seconds (i.e. real-time playback).
    step = max(1, round(args.speed / (args.fps * dt)))
    idxs = list(range(0, n, step))
    if idxs[-1] != n - 1:
        idxs.append(n - 1)
    per_phase = len(idxs)
    total_frames = n_models * per_phase

    def reveal(line, xs, ys, upto):
        line.set_data(xs[:upto], ys[:upto])

    def full(line, xs, ys):
        line.set_data(xs, ys)

    def update(frame):
        phase = min(frame // per_phase, n_models - 1)
        i = idxs[frame % per_phase]
        upto = i + 1

        # --- Pendulum ---
        pendulum.set_measured(data["q"][i])
        if data["models"]:
            sim_q = data["models"][phase]["q"]
            pendulum.set_simulated(sim_q[i])
            # Cumulated MAE (rad) between simulated and measured angle so far.
            mae = float(np.mean(np.abs(sim_q[:upto] - data["q"][:upto])))
            pendulum.ax.set_title(
                f'{data["models"][phase]["name"]} (MAE: {mae:.3f} rad)',
                fontsize=24,
                fontweight="bold",
            )
        else:
            pendulum.set_simulated(data["q"][i], visible=False)

        # Read (measured) curves are static (drawn in full at build time).

        # --- Per-model sim curves ---
        for k, m in enumerate(data["models"]):
            grp = [
                (lines["sim_q"][k], m["q"]),
                (lines["sim_control"][k], m["controls"]),
            ]
            if data["has_speed"]:
                grp.append((lines["sim_speed"][k], m["speed"]))
            for line, ys in grp:
                if k < phase:
                    full(line, ts, ys)
                elif k == phase:
                    reveal(line, ts, ys, upto)
                else:
                    line.set_data([], [])

        # --- Time cursor ---
        for c in cursors:
            c.set_xdata([ts[i], ts[i]])
            c.set_alpha(0.6)

        return []

    interval_ms = 1000.0 / args.fps
    anim = FuncAnimation(
        fig, update, frames=total_frames, interval=interval_ms, blit=False
    )

    if args.save:
        fps = args.fps
        if args.save.lower().endswith(".gif"):
            writer = PillowWriter(fps=fps)
        else:
            writer = FFMpegWriter(fps=fps, bitrate=4000)
        print(f"Saving {total_frames} frames to {args.save} at {fps} fps...")
        anim.save(args.save, writer=writer)
        print("Done.")
    else:
        plt.show()


if __name__ == "__main__":
    main()
