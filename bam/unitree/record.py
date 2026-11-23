# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Record one J288 identification log on a pendulum testbench.

Plays a command profile on the servo at a fixed loop rate and logs every
command/reply pair in BAM's raw log format, with the J288 fields the actuator
model replays and is scored against (see
:class:`bam.unitree.actuator.J288Actuator`)::

    python -m bam.unitree.record --port /dev/cu.usbmodemXXXX --zero-file zero.json \\
        --mass 0.2 --arm-mass 0.02 --length 0.075 --logdir data_raw \\
        --profile sin_time_square --kp 0.97 --kd 0.02

Profiles:

* any BAM position trajectory (``lift_and_drop``, ``sin_time_square``,
  ``up_and_down``, ``sin_sin``, ``half_sine``, ``steps``, ``nothing``), played
  with ``--kp`` / ``--kd``; torque-off steps are mode 0;
* ``torque_chirp`` and ``torque_steps``: pure torque (kp = kd = 0),
  ``--amplitude`` in N.m;
* ``speed_sine``: speed mode (kp = 0), ``--amplitude`` in rad/s, damping ``--kd``;
* ``lift_and_damp``: lift to -pi/2 with ``--kp`` / ``--kd``, then damping mode
  (kp = 0, w_des = 0, ``--kd``) while it falls.

Angles are BAM's: zero with the arm hanging straight down. The hanging angle is
found once per rig by :mod:`bam.unitree.calibrate` and stored against the servo's
absolute output encoder in ``--zero-file``.

Safety: every frame is sent with timeout protection, so the servo stops by itself
about 1 s after this process dies. A recording is aborted, braked and saved with
an ``aborted`` reason if the arm leaves ``--limit`` of hanging, moves faster than
``--speed-limit``, the winding passes ``--abort-above`` or the servo reports a
fault. Before each recording the script waits for the winding to cool below
``--wait-below``.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import time

import numpy as np

from bam.trajectory import cubic_interpolate, trajectories
from bam.unitree.digital_servo import MODE_FOC, MODE_STOP, DigitalServo, sent_gains

# ─────────────────────────────────────────────────────────────────────────────
# Command profiles
# ─────────────────────────────────────────────────────────────────────────────


def command(
    goal_position=0.0,
    goal_speed=0.0,
    goal_torque=0.0,
    kp=0.0,
    kd=0.0,
    torque_enable=True,
):
    return {
        "goal_position": goal_position,
        "goal_speed": goal_speed,
        "goal_torque": goal_torque,
        "kp": kp,
        "kd": kd,
        "torque_enable": torque_enable,
    }


class Profile:
    """A command profile: ``profile(t)`` gives the full J288 command at ``t``."""

    duration = 6.0

    def __call__(self, t: float) -> dict:
        raise NotImplementedError


class PositionProfile(Profile):
    """A BAM position trajectory, played with fixed gains."""

    def __init__(self, name: str, kp: float, kd: float):
        self.trajectory = trajectories[name]
        self.duration = self.trajectory.duration
        self.kp, self.kd = kp, kd

    def __call__(self, t):
        goal, enable = self.trajectory(t)
        return command(goal_position=goal, kp=self.kp, kd=self.kd, torque_enable=enable)


class TorqueChirp(Profile):
    """Pure torque, a sine sweeping from ``f0`` to ``f1`` Hz."""

    def __init__(self, amplitude: float, f0: float = 0.3, f1: float = 3.0):
        self.amplitude, self.f0, self.f1 = amplitude, f0, f1

    def __call__(self, t):
        T = self.duration
        phase = 2 * np.pi * (self.f0 * t + (self.f1 - self.f0) * t**2 / (2 * T))
        # Faded in and out over 0.5 s so the pendulum starts and ends near rest.
        fade = min(1.0, t / 0.5, max(0.0, (T - t) / 0.5))
        return command(goal_torque=self.amplitude * fade * np.sin(phase))


class TorqueSteps(Profile):
    """Pure torque steps: +A, 0, -A, 0, +A/2, -A/2, ... held 0.5 s each."""

    LEVELS = (1.0, 0.0, -1.0, 0.0, 0.5, -0.5, 0.0, 0.75, -0.75, 0.0, 0.25, 0.0)

    def __init__(self, amplitude: float):
        self.amplitude = amplitude

    def __call__(self, t):
        k = min(int(t / 0.5), len(self.LEVELS) - 1)
        return command(goal_torque=self.amplitude * self.LEVELS[k])


class SpeedSine(Profile):
    """Speed mode: w_des a 0.4 Hz sine of ``amplitude`` rad/s, damping ``kd``."""

    def __init__(self, amplitude: float, kd: float):
        self.amplitude, self.kd = amplitude, kd

    def __call__(self, t):
        fade = min(1.0, t / 0.5)
        return command(
            goal_speed=self.amplitude * fade * np.sin(2 * np.pi * 0.4 * t), kd=self.kd
        )


class LiftAndDamp(Profile):
    """Lift to -pi/2 in 2 s with position control, then damping mode for 4 s."""

    def __init__(self, kp: float, kd: float):
        self.kp, self.kd = kp, kd

    def __call__(self, t):
        if t < 2.0:
            # (time, position, velocity): start and finish at rest
            goal = cubic_interpolate([(0.0, 0.0, 0.0), (2.0, -np.pi / 2, 0.0)], t)
            return command(goal_position=goal, kp=self.kp, kd=self.kd)
        return command(kd=self.kd)


class SyncPrefix(Profile):
    """Start a profile with a sharp, visible flick, to line up a video.

    Holds the profile's starting command for 0.5 s, steps the target 0.35 rad
    out and straight back (0.25 s out), settles for 0.75 s, then plays the
    profile. The flick is in the log, so a simulation replayed from the log
    flicks at the same instant as the filmed arm.
    """

    PREFIX = 1.5

    def __init__(self, profile: Profile, kp: float = 2.0, kd: float = 0.05):
        self.profile, self.kp, self.kd = profile, max(kp, 2.0), kd
        self.duration = self.PREFIX + profile.duration

    def __call__(self, t):
        if t >= self.PREFIX:
            return self.profile(t - self.PREFIX)
        start = self.profile(0.0)["goal_position"]
        goal = start + 0.35 if 0.5 <= t < 0.75 else start
        return command(goal_position=goal, kp=self.kp, kd=self.kd)


def make_profile(name: str, kp: float, kd: float, amplitude: float | None) -> Profile:
    if name in trajectories:
        return PositionProfile(name, kp, kd)
    if name == "torque_chirp":
        return TorqueChirp(0.1 if amplitude is None else amplitude)
    if name == "torque_steps":
        return TorqueSteps(0.1 if amplitude is None else amplitude)
    if name == "speed_sine":
        return SpeedSine(2.0 if amplitude is None else amplitude, kd)
    if name == "lift_and_damp":
        return LiftAndDamp(kp, kd)
    raise ValueError(f"Unknown profile {name!r}")


PROFILES = sorted(trajectories) + [
    "torque_chirp",
    "torque_steps",
    "speed_sine",
    "lift_and_damp",
]

# ─────────────────────────────────────────────────────────────────────────────
# Servo loop
# ─────────────────────────────────────────────────────────────────────────────


class Abort(Exception):
    pass


class Runner:
    """Runs a servo at a fixed rate in BAM angles (zero hanging), logging every
    reply and enforcing the safety limits."""

    def __init__(
        self,
        servo: DigitalServo,
        zero: float,
        rate: float = 1000.0,
        limit: float = 2.0,
        speed_limit: float = 20.0,
        abort_above: float = 80.0,
    ):
        self.servo = servo
        self.zero = zero
        self.dt = 1.0 / rate
        self.limit, self.speed_limit, self.abort_above = limit, speed_limit, abort_above
        self.entries: list[dict] = []
        self.t0 = time.perf_counter()
        self.state = servo.recover()
        self._q_prev, self._t_prev, self.speed_est = self.q, 0.0, 0.0

    @property
    def q(self) -> float:
        return self.state["position"] - self.zero

    def now(self) -> float:
        return time.perf_counter() - self.t0

    def step(self, cmd: dict, log: bool = True, check: bool = True) -> dict:
        """Send one command (BAM angles), hold the loop rate."""
        t_send = self.now()
        mode = MODE_FOC if cmd["torque_enable"] else MODE_STOP
        state = self.servo.command(
            mode,
            torque=cmd["goal_torque"],
            speed=cmd["goal_speed"],
            position=cmd["goal_position"] + self.zero,
            kp=cmd["kp"],
            kd=cmd["kd"],
        )
        t_recv = self.now()
        if state is not None:
            self.state = state
            t = 0.5 * (t_send + t_recv)
            if t > self._t_prev:
                raw = (self.q - self._q_prev) / (t - self._t_prev)
                self.speed_est += 0.5 * (raw - self.speed_est)
            self._q_prev, self._t_prev = self.q, t
            if log:
                kp, kd = sent_gains(cmd["kp"], cmd["kd"])
                self.entries.append(
                    {
                        **cmd,
                        "kp": kp,
                        "kd": kd,
                        "timestamp": t,
                        "position": self.q,
                        "reported_speed": state["speed"],
                        "torque": state["torque"],
                        "output_encoder": state["output_encoder"],
                        "input_volts": state["input_volts"],
                        "temp": state["temp"],
                        "winding_temp": state["winding_temp"],
                    }
                )
            if check:
                self._check()
        while self.now() - t_send < self.dt:
            pass
        return state

    def _check(self):
        s = self.state
        if s["error"]:
            raise Abort(f"servo fault {int(s['error']):#x}")
        if abs(self.q) > self.limit:
            raise Abort(f"position {self.q:+.2f} rad outside ±{self.limit:.2f}")
        if abs(self.speed_est) > self.speed_limit:
            raise Abort(
                f"speed {self.speed_est:+.1f} rad/s over {self.speed_limit:.1f}"
            )
        if s["winding_temp"] > self.abort_above:
            raise Abort(f"winding {s['winding_temp']} °C over {self.abort_above:.0f}")

    def hold(self, duration: float, cmd: dict, **kw):
        end = self.now() + duration
        while self.now() < end:
            self.step(cmd, **kw)

    def brake(self, kd: float = 0.05):
        """Damping until nearly still, then stop. Never raises."""
        end = self.now() + 1.5
        while self.now() < end and abs(self.speed_est) > 0.05:
            self.step(command(kd=kd), log=False, check=False)
        self.hold(0.2, command(kd=kd), log=False, check=False)
        self.step(command(torque_enable=False), log=False, check=False)

    def move_to(
        self,
        goal: float,
        kp: float = 1.0,
        kd: float = 0.05,
        speed: float = 1.0,
        settle: float = 0.5,
    ):
        """Ramp the target to ``goal`` at ``speed`` rad/s under PD, then settle."""
        start = self.q
        ramp = abs(goal - start) / speed
        t0 = self.now()
        while (t := self.now() - t0) < ramp:
            g = start + (goal - start) * t / ramp
            self.step(command(goal_position=g, kp=kp, kd=kd), log=False)
        self.hold(settle, command(goal_position=goal, kp=kp, kd=kd), log=False)

    def idle(self, duration: float):
        self.hold(duration, command(torque_enable=False), log=False, check=False)


def load_zero(servo: DigitalServo, zero_file: str) -> tuple[float, dict]:
    """Rotor-position offset of the hanging angle, from the stored encoder zero.

    The multi-turn rotor position is relative to power-up; the 13-bit output
    encoder is absolute. The stored ``output_encoder_zero`` is the encoder angle
    with the arm hanging; at rest now, the rotor position of hanging is the
    current position minus the (wrapped) encoder distance from it.
    """
    calibration = json.load(open(zero_file))
    states = []
    for _ in range(50):
        s = servo.stop()
        if s is not None:
            states.append(s)
        time.sleep(0.002)
    q = np.mean([s["position"] for s in states])
    enc = np.angle(np.mean([np.exp(1j * s["output_encoder"]) for s in states]))
    offset = np.angle(np.exp(1j * (enc - calibration["output_encoder_zero"])))
    return float(q - offset), calibration


def loaded_zero(
    runner: Runner, encoder_zero: float, preload: float, samples: int = 100
) -> float:
    """Rotor position of hanging, read with the gear play loaded.

    The zero from :func:`load_zero` compares the rotor-derived position with the
    output encoder at rest. Hanging, the gears carry no load, so the output can
    sit anywhere in the gear play relative to the rotor (tens of mrad on the
    bench) and the zero would differ from one recording to the next. Here the
    arm is lifted to ``preload`` rad under position control, approached from
    hanging, so gravity and friction load the gears the same way every time,
    and the comparison is made there.

    In practice this did not reduce the scatter: the J288's gearbox winds up
    under load and holds part of it through friction, so the rotor to output
    relation depends on the recent load history, not just on which side the
    play sits. Both readings scattered by about 10 mrad (standard deviation).
    Off by default (``--zero-preload 0``).
    """
    kp, kd = 2.0, 0.05
    runner.move_to(preload, kp=kp, kd=kd, speed=0.5, settle=0.5)
    states = []
    for _ in range(samples):
        runner.step(command(goal_position=preload, kp=kp, kd=kd), log=False)
        states.append(runner.state)
    q = np.mean([s["position"] for s in states])
    enc = np.angle(np.mean([np.exp(1j * s["output_encoder"]) for s in states]))
    return float(q - np.angle(np.exp(1j * (enc - encoder_zero))))


def wait_cool(servo: DigitalServo, below: float, verbose: bool = True):
    """Send stop frames until the winding is at or below ``below`` °C."""
    last_print = 0.0
    t0 = time.perf_counter()
    while True:
        s = servo.stop()
        if s is not None and s["winding_temp"] <= below:
            return s
        now = time.perf_counter() - t0
        if verbose and s is not None and now - last_print > 5.0:
            print(
                f"  cooling: winding {s['winding_temp']} °C, waiting for {below:.0f} °C",
                flush=True,
            )
            last_print = now
        time.sleep(0.01)


# ─────────────────────────────────────────────────────────────────────────────
# Recording
# ─────────────────────────────────────────────────────────────────────────────


def record(args) -> str:
    servo = DigitalServo(args.port, args.id)
    try:
        servo.recover()
        wait_cool(servo, args.wait_below)
        servo.recover()
        if args.zero_file:
            zero, calibration = load_zero(servo, args.zero_file)
        else:
            zero, calibration = servo.stop()["position"], None
            print(
                "  no --zero-file: using the current position as hanging (debug only)"
            )

        profile = make_profile(args.profile, args.kp, args.kd, args.amplitude)
        if args.sync:
            profile = SyncPrefix(profile, args.kp, args.kd)
        runner = Runner(
            servo, zero, args.rate, args.limit, args.speed_limit, args.abort_above
        )
        zero_at_rest = zero
        if calibration is not None and args.zero_preload > 0.0:
            zero = loaded_zero(
                runner, calibration["output_encoder_zero"], args.zero_preload
            )
            runner.zero = zero
            runner._q_prev = runner.q
            print(f"  zero: {zero - zero_at_rest:+.4f} rad from the unloaded reading")
        # Start from the profile's first target, held still.
        first = profile(0.0)
        if first["kp"] > 0.0 and first["torque_enable"]:
            runner.move_to(
                first["goal_position"],
                kp=max(first["kp"], 0.5),
                kd=max(first["kd"], 0.02),
            )
        else:
            runner.idle(0.3)

        aborted = None
        runner.entries.clear()
        t_start = runner.now()
        try:
            while (t := runner.now() - t_start) < profile.duration:
                runner.step(profile(t))
        except Abort as e:
            aborted = str(e)
            print(f"  ABORTED: {aborted}")
        except KeyboardInterrupt:
            aborted = "interrupted"
        runner.brake()

        if aborted is None:
            # Back to hanging gently, then relax.
            try:
                runner.move_to(0.0, kp=1.0, kd=0.05, speed=0.5)
            except Abort as e:
                print(f"  return aborted: {e}")
                runner.brake()
        runner.idle(0.2)
    finally:
        servo.close()

    return save(
        args,
        runner.entries,
        zero,
        calibration,
        aborted,
        {"zero_position_at_rest": zero_at_rest},
    )


def save(
    args,
    entries: list[dict],
    zero: float,
    calibration: dict | None,
    aborted: str | None,
    extra: dict | None = None,
) -> str:
    if not entries:
        raise RuntimeError("nothing recorded")
    t = np.array([e["timestamp"] for e in entries])
    # A pause in the loop leaves the servo running an old command while the
    # processed log interpolates across it; set such recordings aside.
    longest_gap = float(np.max(np.diff(t)))
    if aborted is None and longest_gap > args.max_gap:
        aborted = f"loop gap of {longest_gap * 1000:.1f} ms"
        print(f"  ABORTED: {aborted} (over {args.max_gap * 1000:.0f} ms)")
    t -= t[0]
    q = np.array([e["position"] for e in entries])
    # True speed from position; the servo's own speed is filtered (reported_speed).
    speed = np.gradient(q, t)
    for e, ti, wi in zip(entries, t, speed):
        e["timestamp"] = float(ti)
        e["speed"] = float(wi)

    kp, kd = sent_gains(args.kp, args.kd)
    volts = float(np.mean([e["input_volts"] for e in entries]))
    data = {
        "motor": "unitree_j288",
        "profile": args.profile,
        "trajectory": args.profile,
        "mass": args.mass,
        "arm_mass": args.arm_mass,
        "length": args.length,
        "mass_inertia": args.mass_inertia,
        "kp": kp,
        "kd": kd,
        "amplitude": args.amplitude,
        "vin": args.vin if args.vin is not None else volts,
        "measured_vin": volts,
        "zero_position": zero,
        "zero_preload": args.zero_preload,
        **(extra or {}),
        "zero_file": args.zero_file,
        "calibration": calibration,
        "rate": args.rate,
        "longest_gap": longest_gap,
        "note": args.note,
        "aborted": aborted,
        "entries": entries,
    }
    if getattr(args, "sync", False):
        # Only in sync recordings, which are for video, not fitting: BAM batches
        # logs by their fields, so a fitting set must not mix fields.
        data["sync"] = True
    # Aborted logs go to a subfolder, so bam.process does not pick them up.
    logdir = os.path.join(args.logdir, "aborted") if aborted else args.logdir
    os.makedirs(logdir, exist_ok=True)
    date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
    gains = f"kp{kp:.2f}_kd{kd:.3f}"
    name = f"{date}_{args.profile}_{gains}{'_ABORTED' if aborted else ''}.json"
    path = os.path.join(logdir, name)
    with open(path, "w") as f:
        json.dump(data, f)
    peak = max(e["winding_temp"] for e in entries)
    print(
        f"  saved {len(entries)} entries to {path} (winding peak {peak} °C, {volts:.1f} V)"
    )
    return path


def add_arguments(p: argparse.ArgumentParser):
    p.add_argument("--port", required=True)
    p.add_argument("--id", type=int, default=0)
    p.add_argument("--zero-file", help="hanging calibration from bam.unitree.calibrate")
    p.add_argument("--mass", type=float, required=True, help="mass at the arm end [kg]")
    p.add_argument(
        "--arm-mass", type=float, required=True, help="uniform arm mass [kg]"
    )
    p.add_argument(
        "--length", type=float, required=True, help="pivot to mass centre [m]"
    )
    p.add_argument(
        "--mass-inertia", type=float, default=0.0, help="mass's own inertia [kg.m^2]"
    )
    p.add_argument("--logdir", required=True)
    p.add_argument(
        "--vin", type=float, help="supply voltage for the log (default: measured)"
    )
    p.add_argument("--rate", type=float, default=1000.0, help="loop rate [Hz]")
    p.add_argument(
        "--limit", type=float, default=2.0, help="abort beyond this from hanging [rad]"
    )
    p.add_argument("--speed-limit", type=float, default=20.0, help="[rad/s]")
    p.add_argument(
        "--wait-below",
        type=float,
        default=45.0,
        help="start when the winding is at or below [°C]",
    )
    p.add_argument("--abort-above", type=float, default=80.0, help="[°C]")
    p.add_argument(
        "--zero-preload",
        type=float,
        default=0.0,
        help="read the zero at this angle with the gears loaded, not at rest [rad]; see loaded_zero()",
    )
    p.add_argument(
        "--max-gap",
        type=float,
        default=0.010,
        help="set a recording aside if the loop paused longer than this [s]",
    )
    p.add_argument("--note", default="")


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    add_arguments(p)
    p.add_argument("--profile", required=True, choices=PROFILES)
    p.add_argument("--kp", type=float, default=0.97, help="[N.m/rad]")
    p.add_argument("--kd", type=float, default=0.02, help="[N.m.s/rad]")
    p.add_argument(
        "--amplitude", type=float, help="torque [N.m] or speed [rad/s] profiles"
    )
    p.add_argument(
        "--sync",
        action="store_true",
        help="start with a sharp 0.35 rad flick at 0.5 s, to line up a video",
    )
    record(p.parse_args())


if __name__ == "__main__":
    main()
