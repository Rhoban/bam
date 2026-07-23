# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Per-parameter sensitivity analysis for identified models.

For a given params file (or a directory of them) and a directory of processed
logs, this varies each parameter *independently* (all others held at their
fitted value) and finds the range around the fitted value over which the
mean-absolute-error (MAE) stays within a tolerance (default 3%) of the best
(fitted) MAE.

The lower/upper bound of that range is reported for every parameter and saved to
a JSON file::

    # single file, explicit output
    uv run python -m bam.sensitivity \\
        --params params/xl330/m4.json --logdir data/xl330 \\
        --output sensitivity_m4.json

    # single file, output written next to the input (m4_sensitivity.json)
    uv run python -m bam.sensitivity \\
        --params params/xl330/m4.json --logdir data/xl330 --save

    # a whole directory: every <model>.json is processed and written to
    # <model>_sensitivity.json next to it
    uv run python -m bam.sensitivity \\
        --params params/xl330 --logdir data/xl330 --save

A wide range means the parameter is weakly identified (MAE barely moves as it
changes); a narrow range means it is tightly constrained by the data. Bounds
are searched within the model's declared ``[min, max]`` for each parameter; a
bound that lands exactly on ``min``/``max`` means the parameter is insensitive
up to its allowed limit (flagged ``clamped`` in the printed table).
"""

import argparse
import glob
import json
import os

import numpy as np

from .logs import Logs
from .model import load_model
from . import simulate


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--params",
        type=str,
        required=True,
        help="Params JSON file, or a directory whose *.json models are all processed",
    )
    parser.add_argument("--logdir", type=str, required=True, help="Processed logs dir")
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output JSON path (single input). With --save, or a directory input, "
        "this is ignored in favour of the <input>_sensitivity.json convention.",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Write results next to each input as <input>_sensitivity.json",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.03,
        help="Relative MAE increase defining the range edge (default 0.03 = 3%%)",
    )
    parser.add_argument(
        "--scan",
        type=int,
        default=20,
        help="Coarse scan points per direction before bisection",
    )
    parser.add_argument(
        "--bisect",
        type=int,
        default=24,
        help="Bisection iterations to refine each crossing",
    )
    return parser


def compute_mae(model, batch, log_positions) -> float:
    simulator = simulate.Simulator(model)
    result = simulator.rollout_log(batch, reset_period=None, simulate_control=True)
    positions = np.array(result[0])
    return float(np.mean(np.abs(positions - log_positions)))


def analyze(model, batch, log_positions, tolerance: float, scan: int, bisect: int):
    """Compute the per-parameter sensitivity bounds for one model.

    :returns: ``(base_mae, threshold, results)`` where ``results`` maps each
        parameter name to ``{value, lower, upper, lower_clamped, upper_clamped}``.
    """

    def mae() -> float:
        return compute_mae(model, batch, log_positions)

    base_mae = mae()
    threshold = base_mae * (1.0 + tolerance)
    parameters = model.get_parameters()

    def find_bound(parameter, v0: float, far: float) -> float:
        """Value in [v0, far] where MAE first exceeds ``threshold``.

        ``far`` is the model bound in the searched direction (``max`` for the
        upper bound, ``min`` for the lower). Returns ``far`` (clamped) if the MAE
        never crosses the threshold before reaching it.
        """
        if (far - v0) == 0.0:
            return v0

        previous = v0
        for i in range(1, scan + 1):
            value = v0 + (far - v0) * (i / scan)
            parameter.value = value
            if mae() > threshold:
                inside, outside = previous, value
                for _ in range(bisect):
                    mid = 0.5 * (inside + outside)
                    parameter.value = mid
                    if mae() <= threshold:
                        inside = mid
                    else:
                        outside = mid
                return inside
            previous = value

        return far

    results = {}
    for name, parameter in parameters.items():
        v0 = parameter.value
        upper = find_bound(parameter, v0, parameter.max)
        parameter.value = v0
        lower = find_bound(parameter, v0, parameter.min)
        parameter.value = v0

        results[name] = {
            "value": v0,
            "lower": lower,
            "upper": upper,
            "lower_clamped": lower == parameter.min,
            "upper_clamped": upper == parameter.max,
        }

    return base_mae, threshold, results


def output_path(params_file: str, args, is_dir_input: bool) -> str:
    """Resolve where the sensitivity JSON for ``params_file`` should be written."""
    stem = params_file[:-5] if params_file.endswith(".json") else params_file
    suffixed = f"{stem}_sensitivity.json"

    # --save, or a directory input, always uses the <input>_sensitivity.json rule.
    if args.save or is_dir_input:
        return suffixed
    # Single input with an explicit --output.
    if args.output:
        return args.output
    # Single input, nothing specified: fall back to the same convention.
    return suffixed


def print_table(model, logdir, n_logs, base_mae, threshold, tolerance, results, out):
    print(
        f"Model {model.name} on {logdir} ({n_logs} logs)\n"
        f"Baseline MAE: {base_mae:.6f}   threshold (+{tolerance * 100:.0f}%): "
        f"{threshold:.6f}\n"
    )
    print(f"{'parameter':30} {'fitted':>12} {'lower':>12} {'upper':>12}   width")
    for name, r in results.items():
        lo = f"{r['lower']:.5g}" + ("*" if r["lower_clamped"] else "")
        hi = f"{r['upper']:.5g}" + ("*" if r["upper_clamped"] else "")
        width = r["upper"] - r["lower"]
        print(f"{name:30} {r['value']:12.5g} {lo:>12} {hi:>12}   {width:.4g}")
    print("(* = bound clamped at the parameter's model min/max)")
    print(f"Saved to {out}\n")


def main():
    args = build_arg_parser().parse_args()

    is_dir_input = os.path.isdir(args.params)
    if is_dir_input:
        input_files = sorted(
            f
            for f in glob.glob(os.path.join(args.params, "*.json"))
            if not f.endswith("_sensitivity.json")
        )
    else:
        input_files = [args.params]

    # Logs are shared across all processed models.
    logs = Logs(args.logdir)
    batch = logs.make_batch()
    log_positions = np.array([entry["position"] for entry in batch["entries"]])
    n_logs = len(logs.logs)

    for params_file in input_files:
        try:
            model = load_model(params_file)
        except Exception as error:
            if is_dir_input:
                print(f"Skipping {params_file}: not a valid model params file ({error})")
                continue
            raise

        base_mae, threshold, results = analyze(
            model, batch, log_positions, args.tolerance, args.scan, args.bisect
        )

        out = output_path(params_file, args, is_dir_input)
        json.dump(
            {
                "model": model.name,
                "actuator": model.actuator_name,
                "params": params_file,
                "logdir": args.logdir,
                "base_mae": base_mae,
                "tolerance": args.tolerance,
                "threshold_mae": threshold,
                "sensitivity": results,
            },
            open(out, "w"),
            indent=2,
        )

        print_table(
            model,
            args.logdir,
            n_logs,
            base_mae,
            threshold,
            args.tolerance,
            results,
            out,
        )


if __name__ == "__main__":
    main()
