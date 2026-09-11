# Copyright 2026 BAM Project
# Overlay real logs, the BAM reference simulator, and MuJoCo (CPU).

"""sim2sim / sim2real report for an identified BAM actuator.

For each processed log the script rolls out:

* the BAM analytic pendulum (``bam.simulate``, ``simulate_control=True``)
* the BAM MuJoCo pendulum (``bam.mujoco.Simulator`` + ``MujocoController``)

and overlays both against the recorded joint angle. The MuJoCo scene is the
same equivalent pendulum used for identification (tip mass + uniform rod),
not a CAD URDF — a visual CAD model would not share the identified inertia.
"""

from __future__ import annotations

import argparse
import base64
import io
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _png(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def _rollouts(log: dict, params: str):
    from bam.model import load_model
    from bam import simulate
    from bam import mujoco as mujoco_backend

    model_ref = load_model(params)
    bam_q, bam_dq, bam_u = simulate.Simulator(model_ref).rollout_log(
        log, simulate_control=True
    )

    model_mj = load_model(params)
    mj_q, mj_dq, mj_u = mujoco_backend.Simulator(
        model_mj, command_delay=True
    ).rollout_log(log)

    real_q = np.array([e["position"] for e in log["entries"]], dtype=float)
    real_u = np.array([e["control"] for e in log["entries"]], dtype=float)
    goal = np.array([e["goal_position"] for e in log["entries"]], dtype=float)
    t = np.arange(len(real_q)) * float(log["dt"])

    return {
        "t": t,
        "goal": goal,
        "real_q": real_q,
        "bam_q": np.asarray(bam_q, dtype=float),
        "mj_q": np.asarray(mj_q, dtype=float),
        "real_u": real_u,
        "bam_u": np.asarray(bam_u, dtype=float),
        "mj_u": np.asarray(mj_u, dtype=float),
    }


def _mae(a, b) -> float:
    return float(np.mean(np.abs(np.asarray(a) - np.asarray(b))))


def _plot_traj(r: dict, title: str) -> str:
    fig, axes = plt.subplots(2, 1, figsize=(8.02, 3.60), sharex=True)
    ax, axu = axes
    ax.plot(r["t"], r["goal"], color="#9aa0a6", lw=1.0, label="goal")
    ax.plot(r["t"], r["real_q"], color="#1a73e8", lw=1.4, label="real")
    ax.plot(r["t"], r["bam_q"], color="#e37400", lw=1.1, ls="--", label="BAM sim")
    ax.plot(r["t"], r["mj_q"], color="#188038", lw=1.1, ls=":", label="MuJoCo")
    ax.set_ylabel("q [rad]")
    ax.set_title(title, fontsize=11)
    ax.legend(loc="upper right", ncol=4, fontsize=8, frameon=False)
    ax.grid(True, alpha=0.25)

    axu.plot(r["t"], r["real_u"], color="#1a73e8", lw=1.0, label="real cmd")
    axu.plot(r["t"], r["bam_u"], color="#e37400", lw=0.9, ls="--", label="BAM")
    axu.plot(r["t"], r["mj_u"], color="#188038", lw=0.9, ls=":", label="MuJoCo")
    axu.set_ylabel("I [A]")
    axu.set_xlabel("t [s]")
    axu.grid(True, alpha=0.25)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.90, bottom=0.12, hspace=0.12)
    return _png(fig)


def _export_mjcf(log: dict, params: str, out: Path) -> None:
    from bam.model import load_model
    from bam.testbench_mujoco import Pendulum

    model = load_model(params)
    model.actuator.load_log(log)
    spec = Pendulum(
        {
            "mass": model.actuator.testbench.mass,
            "arm_mass": model.actuator.testbench.arm_mass,
            "length": model.actuator.testbench.length,
        }
    ).build_spec("pendulum", q_offset=model.q_offset.value)
    out.write_text(spec.to_xml())


def main() -> None:
    ap = argparse.ArgumentParser(description="BAM / MuJoCo / real overlay report")
    ap.add_argument("--logs", default=os.path.expanduser("~/hls_bam/proc_v2"))
    ap.add_argument(
        "--params",
        default=os.path.join(
            os.path.dirname(__file__), "..", "bam", "params", "feetech_hls", "m1.json"
        ),
    )
    ap.add_argument("--plot-kp", type=float, default=3.0)
    ap.add_argument("--out", default=os.path.expanduser("~/hls_bam/report_mujoco.html"))
    ap.add_argument(
        "--mjcf",
        default=os.path.expanduser("~/hls_bam/pendulum.xml"),
        help="Write the MuJoCo pendulum MJCF used for the rollouts",
    )
    args = ap.parse_args()

    from bam.logs import Logs

    logs = Logs(os.path.expanduser(args.logs))
    params = os.path.abspath(os.path.expanduser(args.params))
    if not logs.logs:
        raise SystemExit(f"no logs in {args.logs}")

    _export_mjcf(logs.logs[0], params, Path(os.path.expanduser(args.mjcf)))

    rows = []
    plots = []
    order = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]
    plotted = set()

    for log in sorted(logs.logs, key=lambda d: (d["trajectory"], d["kp"])):
        r = _rollouts(log, params)
        mae_bam = _mae(r["real_q"], r["bam_q"])
        mae_mj = _mae(r["real_q"], r["mj_q"])
        mae_s2s = _mae(r["bam_q"], r["mj_q"])
        rows.append(
            {
                "traj": log["trajectory"],
                "kp": log["kp"],
                "mae_bam": mae_bam,
                "mae_mj": mae_mj,
                "mae_s2s": mae_s2s,
            }
        )
        key = log["trajectory"]
        if log["kp"] == args.plot_kp and key not in plotted:
            plots.append(
                (
                    key,
                    _plot_traj(
                        r,
                        f"{key}  ·  kp={log['kp']:.0f}  ·  "
                        f"BAM {mae_bam:.4f}  MuJoCo {mae_mj:.4f}  "
                        f"sim2sim {mae_s2s:.4f} rad",
                    ),
                )
            )
            plotted.add(key)

    plots.sort(key=lambda x: order.index(x[0]) if x[0] in order else 99)

    def avg(field, pred=None):
        xs = [row[field] for row in rows if pred is None or pred(row)]
        return float(np.mean(xs)) if xs else float("nan")

    tr = "".join(
        f"<tr><td>{r['traj']}</td><td>{r['kp']:.0f}</td>"
        f"<td>{r['mae_bam']:.5f}</td><td>{r['mae_mj']:.5f}</td>"
        f"<td>{r['mae_s2s']:.5f}</td></tr>"
        for r in rows
    )
    figs = "".join(
        f'<h3>{name}</h3><img src="data:image/png;base64,{b64}" alt="{name}"/>'
        for name, b64 in plots
    )

    html = f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8"/>
<title>HLS2915 · BAM / MuJoCo / real</title>
<style>
 body {{ font-family: system-ui, sans-serif; max-width: 860px; margin: 24px auto; color: #202124; }}
 table {{ border-collapse: collapse; width: 100%; font-size: 14px; }}
 th, td {{ border: 1px solid #dadce0; padding: 6px 8px; text-align: right; }}
 th:first-child, td:first-child {{ text-align: left; }}
 img {{ width: 802px; max-width: 100%; }}
 .note {{ color: #5f6368; font-size: 13px; }}
</style></head><body>
<h1>HLS2915 sim2sim / sim2real</h1>
<p class="note">
Real = processed logs. BAM = analytic pendulum + identified m1.
MuJoCo = <code>bam.testbench_mujoco.Pendulum</code> (same m / arm-mass / length)
driven by <code>MujocoController</code>. CAD URDF is not used for dynamics.
MJCF written to <code>{os.path.expanduser(args.mjcf)}</code>.
</p>
<p>
Mean MAE — BAM vs real <b>{avg('mae_bam'):.5f}</b> rad,
MuJoCo vs real <b>{avg('mae_mj'):.5f}</b> rad,
BAM vs MuJoCo (sim2sim) <b>{avg('mae_s2s'):.5f}</b> rad.
Held-out kp=4: BAM {avg('mae_bam', lambda r: r['kp']==4):.5f},
MuJoCo {avg('mae_mj', lambda r: r['kp']==4):.5f}.
</p>
<table>
<tr><th>trajectory</th><th>kp</th><th>BAM vs real</th><th>MuJoCo vs real</th><th>BAM vs MuJoCo</th></tr>
{tr}
</table>
<h2>kp = {args.plot_kp:.0f} overlays</h2>
{figs}
</body></html>
"""
    out = Path(os.path.expanduser(args.out))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding="utf-8")
    print(f"wrote {out} ({out.stat().st_size/1024:.0f} KB)")
    print(f"wrote {os.path.expanduser(args.mjcf)}")
    print(
        f"mean MAE bam={avg('mae_bam'):.5f}  mujoco={avg('mae_mj'):.5f}  "
        f"sim2sim={avg('mae_s2s'):.5f}"
    )


if __name__ == "__main__":
    main()
