# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Find the hanging angle of a J288 pendulum and store it against the encoder.

BAM measures pendulum angles from the arm hanging straight down. Letting the arm
settle is not accurate enough on a J288: its gearbox friction can hold a
pendulum well off vertical. Inferring the angle from holding torque is not
either, as the same friction is wide and load-dependent. Instead the arm is set
horizontal by hand on each side, checked with a spirit level or a phone
inclinometer, and the output encoder is read at each::

    python -m bam.unitree.calibrate --port /dev/cu.usbmodemXXXX --output zero.json

Take the masses off (the bare arm stays where it is put; with a mass on, mode 0
only brakes and the arm would sag), or hold the arm still while each reading is
taken. The face the level sits on must be parallel to the line from the output
axis to the mass centre, which holds for a straight arm with the masses on its
centreline. Horizontal on the two sides must be pi apart; the gap is reported as
a check and the two readings are averaged.

The result is stored against the servo's absolute output encoder, which keeps
its angle across power cycles, so :mod:`bam.unitree.record` recovers the zero
at the start of every recording. The hanging angle depends on how the arm is
mounted, not on the mass at its end, so calibrate once per arm, and again
whenever the arm or servo is remounted.
"""

from __future__ import annotations

import argparse
import datetime
import json

import numpy as np

from bam.unitree.digital_servo import DigitalServo


def read_at_rest(servo: DigitalServo, samples: int = 100) -> tuple[float, float]:
    """Mean rotor-derived position and circular-mean output encoder, in mode 0."""
    states = []
    while len(states) < samples:
        s = servo.stop()
        if s is not None:
            states.append(s)
    q = float(np.mean([s["position"] for s in states]))
    enc = float(np.angle(np.mean([np.exp(1j * s["output_encoder"]) for s in states])))
    return q, enc


def wrap(x: float) -> float:
    return float(np.angle(np.exp(1j * x)))


def level_calibration(servo: DigitalServo) -> dict:
    """Hanging angle from the arm set horizontal by hand on both sides.

    The first reading, with the arm roughly hanging, only tells which side each
    horizontal reading is on (+pi/2 or -pi/2 from hanging in the servo's sense).
    """
    print(
        "Take the masses off, or hold the arm still for each reading. The servo stays"
        " in mode 0 (it brakes, it does not hold)."
    )
    readings = {}
    for name, prompt in (
        ("down", "Point the arm roughly straight down, let go, then Enter: "),
        (
            "side_a",
            "Turn the arm to horizontal on one side, check it with the level, let go, then Enter: ",
        ),
        (
            "side_b",
            "Now horizontal on the other side, check it with the level, let go, then Enter: ",
        ),
    ):
        input(prompt)
        servo.recover()
        q, enc = read_at_rest(servo)
        readings[name] = {"position": q, "output_encoder": enc}
        print(f"  {name}: encoder {enc:+.4f} rad, rotor-derived position {q:+.4f} rad")

    down = readings["down"]["output_encoder"]
    zeros = []
    for name in ("side_a", "side_b"):
        enc = readings[name]["output_encoder"]
        offset = wrap(enc - down)
        if abs(abs(offset) - np.pi / 2) > 0.5:
            raise RuntimeError(
                f"{name} is {offset:+.2f} rad from the 'down' reading, not about ±pi/2: "
                "was the arm horizontal, and roughly down for the first reading?"
            )
        zeros.append(wrap(enc - np.sign(offset) * np.pi / 2))
    if np.sign(wrap(readings["side_a"]["output_encoder"] - down)) == np.sign(
        wrap(readings["side_b"]["output_encoder"] - down)
    ):
        raise RuntimeError("both horizontal readings are on the same side of hanging")

    spread = wrap(zeros[1] - zeros[0])
    zero = wrap(zeros[0] + spread / 2)
    gap = abs(
        wrap(
            readings["side_a"]["output_encoder"] - readings["side_b"]["output_encoder"]
        )
    )
    print(
        f"the two sides give hanging {abs(spread):.4f} rad apart; horizontal to horizontal "
        f"is {gap:.4f} rad (pi = {np.pi:.4f})"
    )
    if abs(spread) > 0.02:
        print("  WARNING: more than 0.02 rad apart; check the level and repeat")
    return {
        "method": "level",
        "output_encoder_zero": zero,
        "side_spread": abs(spread),
        "horizontal_gap_error": gap - np.pi,
        "readings": readings,
    }


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", required=True)
    p.add_argument("--id", type=int, default=0)
    p.add_argument("--output", required=True, help="calibration file to write")
    args = p.parse_args()

    servo = DigitalServo(args.port, args.id)
    try:
        result = level_calibration(servo)
    finally:
        servo.close()
    result["date"] = datetime.datetime.now().isoformat(timespec="seconds")
    result["servo_id"] = args.id
    with open(args.output, "w") as f:
        json.dump(result, f, indent=2)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
