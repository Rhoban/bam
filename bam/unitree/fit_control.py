# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""J288 identification helpers: control-law fit (stage 1) and evaluation.

The J288 reports its motor torque in every state frame, and that torque is a
known function of the logged commands, position and speed plus four firmware
and motor parameters: ``speed_filter_tau``, ``torque_limit``, ``R`` and ``ke``.
Stage 1 fits those directly by regression on the logged torque (and on the
servo's reported speed, which pins the filter), with no simulation. Stage 2 is
BAM's usual fit with them held fixed::

    python -m bam.unitree.fit_control fit --logdir processed/ --output j288_control.json
    python -m bam.fit --actuator unitree_j288 --model m6 --logdir processed/ \\
        --set "$(python -m bam.unitree.fit_control set j288_control.json)"

``eval`` rolls a parameter file out against logs with BAM's simulator and
reports the position error (as ``bam.fit`` scores it) and the error of the
simulated reported torque against the logged one::

    python -m bam.unitree.fit_control eval --params bam/params/unitree_j288/m6.json --logdir validation/

Logs must carry the J288 fields written by the recorder (``torque``,
``reported_speed``, per-entry commands); see :class:`bam.unitree.actuator.J288Actuator`.
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from bam.logs import Logs
from bam.model import load_model, models
from bam.actuators import actuators
from bam.unitree.actuator import J288Actuator
from bam.unitree.digital_servo import TORQUE_STEP

#: Parameters stage 1 can identify.
CONTROL_PARAMETERS = ("speed_filter_tau", "torque_limit", "R", "ke")


def make_model(params: str | None, model_name: str = "m6"):
    if params is not None:
        return load_model(params)
    model = models[model_name]()
    model.set_actuator(actuators["unitree_j288"]())
    model.actuator_name = "unitree_j288"
    return model


def replay_control(
    model, log: dict, rounded: bool = True
) -> tuple[np.ndarray, np.ndarray]:
    """Run the firmware law on the *logged* state of one log.

    :param rounded: Round the torque to the protocol's resolution, as the servo
        reports it. The fit uses unrounded torque: rounding makes the error
        piecewise flat in the parameters, which stalls a gradient solver.
    :returns: ``(reported_torque, filtered_speed)`` predicted at each entry.
    """
    act: J288Actuator = model.actuator
    act.load_log(log)
    act.reset()
    dt = float(np.asarray(log["dt"]).reshape(-1)[0])
    torque, speed = [], []
    for entry in log["entries"]:
        control = act.compute_control(
            entry["goal_position"], entry["position"], entry["speed"], dt
        )
        out = act.compute_torque(
            control, entry["torque_enable"], entry["position"], entry["speed"]
        )
        torque.append(np.copy(act.reported_torque if rounded else out))
        speed.append(np.copy(act.w_filtered))
    return np.array(torque, dtype=float), np.array(speed, dtype=float)


def _logged(log: dict, key: str) -> np.ndarray:
    return np.array([e[key] for e in log["entries"]], dtype=float)


def fit_control(
    model,
    logs: list[dict],
    fit: tuple[str, ...] = CONTROL_PARAMETERS,
    speed_weight: float = 0.01,
    skip: float = 0.05,
) -> dict:
    """Least-squares fit of the control-law parameters to logged torque.

    Residuals are the reported-torque error [N.m] and ``speed_weight`` times the
    reported-speed error [rad/s], so with the default 1 rad/s of speed error
    weighs like 0.01 N.m of torque. The first ``skip`` seconds of each log are
    left out while the filter settles from its seed.
    """
    from scipy.optimize import least_squares

    params = [getattr(model, name) for name in fit]
    for p in params:
        p.value = float(np.clip(p.value, p.min, p.max))
    # Scale each parameter by its range so the solver steps are balanced.
    lo = np.array([p.min for p in params])
    hi = np.array([p.max for p in params])
    x0 = (np.array([p.value for p in params]) - lo) / (hi - lo)

    targets = []
    for log in logs:
        dt = float(np.asarray(log["dt"]).reshape(-1)[0])
        start = int(round(skip / dt))
        targets.append((start, _logged(log, "torque"), _logged(log, "reported_speed")))

    def residuals(x):
        for p, v in zip(params, lo + x * (hi - lo)):
            p.value = v
        res = []
        for log, (start, tau, w) in zip(logs, targets):
            tau_hat, w_hat = replay_control(model, log, rounded=False)
            res.append((tau_hat - tau)[start:])
            res.append(speed_weight * (w_hat - w)[start:])
        return np.concatenate(res)

    result = least_squares(
        residuals, x0, bounds=(0.0, 1.0), loss="soft_l1", f_scale=0.01, diff_step=1e-3
    )
    values = lo + result.x * (hi - lo)
    for p, v in zip(params, values):
        p.value = float(v)

    # Rough 1-sigma from the Jacobian: a large value flags a parameter the logs
    # do not constrain (for example R and ke separately at low speed).
    r = result.fun
    dof = max(1, len(r) - len(params))
    try:
        cov = np.linalg.pinv(result.jac.T @ result.jac) * (r @ r) / dof
        sigma = np.sqrt(np.clip(np.diag(cov), 0.0, None)) * (hi - lo)
    except (np.linalg.LinAlgError, ValueError):
        sigma = np.full(len(params), np.nan)

    torque_err = np.concatenate(
        [
            np.abs(replay_control(model, log)[0] - t[1])[t[0] :]
            for log, t in zip(logs, targets)
        ]
    )
    return {
        "values": {name: float(v) for name, v in zip(fit, values)},
        "sigma": {name: float(s) for name, s in zip(fit, sigma)},
        "torque_mae": float(np.mean(torque_err)),
        "torque_p95": float(np.percentile(torque_err, 95)),
        "success": bool(result.success),
    }


def evaluate(model, logs: list[dict]) -> list[dict]:
    """Simulate each log and compare position and reported torque."""
    from bam.simulate import Simulator

    act: J288Actuator = model.actuator
    rows = []
    for log in logs:
        positions, velocities, controls = Simulator(model).rollout_log(
            log, simulate_control=True
        )
        enable = np.array([e["torque_enable"] for e in log["entries"]], dtype=bool)
        v = np.array(velocities, dtype=float)
        control = np.array(controls, dtype=float)
        torque = np.where(enable, control, -act.braking_gain() * v)
        if act.quantize:
            torque = np.round(torque / TORQUE_STEP) * TORQUE_STEP
        rows.append(
            {
                "log": log.get("source", log.get("filename", "?")),
                "position_mae": float(
                    np.mean(np.abs(np.array(positions) - _logged(log, "position")))
                ),
                "torque_mae": float(np.mean(np.abs(torque - _logged(log, "torque")))),
            }
        )
    return rows


def _full_params(model) -> dict:
    data = {name: p.value for name, p in model.get_parameters().items()}
    data["model"] = model.name
    data["actuator"] = "unitree_j288"
    return data


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_fit = sub.add_parser("fit", help="stage 1: fit the control law to logged torque")
    p_fit.add_argument("--logdir", required=True)
    p_fit.add_argument(
        "--params", help="starting parameter file (default: j288 defaults)"
    )
    p_fit.add_argument(
        "--model", default="m6", help="friction model when starting from defaults"
    )
    p_fit.add_argument(
        "--fit", nargs="+", default=list(CONTROL_PARAMETERS), choices=CONTROL_PARAMETERS
    )
    p_fit.add_argument("--speed-weight", type=float, default=0.01)
    p_fit.add_argument("--output", required=True, help="parameter file to write")

    p_set = sub.add_parser(
        "set", help="print a bam.fit --set string holding stage 1 fixed"
    )
    p_set.add_argument("params")
    p_set.add_argument("--keys", nargs="+", default=list(CONTROL_PARAMETERS))

    p_eval = sub.add_parser(
        "eval", help="simulate logs, report position and torque error"
    )
    p_eval.add_argument("--params", required=True)
    p_eval.add_argument("--logdir", required=True)

    args = parser.parse_args()

    if args.command == "set":
        data = json.load(open(args.params))
        print(repr({k: data[k] for k in args.keys}))
        return

    logs = Logs(args.logdir).logs
    if args.command == "fit":
        model = make_model(args.params, args.model)
        result = fit_control(model, logs, tuple(args.fit), args.speed_weight)
        for name in args.fit:
            print(
                f"  {name:18s} {result['values'][name]:.6g}  (±{result['sigma'][name]:.2g})"
            )
        print(
            f"  torque error: mean {result['torque_mae']:.4f} N.m, 95th percentile {result['torque_p95']:.4f} N.m"
        )
        print(f"  braking gain kt*ke/R = {model.actuator.braking_gain():.4f} N.m.s/rad")
        json.dump(_full_params(model), open(args.output, "w"), indent=2)
        print(f"wrote {args.output}")
    else:
        model = make_model(args.params)
        for row in evaluate(model, logs):
            print(
                f"  {row['log']:45s} position MAE {row['position_mae']:.4f} rad   torque MAE {row['torque_mae']:.4f} N.m"
            )


if __name__ == "__main__":
    main()
