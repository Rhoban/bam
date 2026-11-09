# Copyright 2026 MangDang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""Identification recorder for the MangDang MD-300-150.

The module is driven through the ESP32-S3 SPI bridge, whose host protocol lives
in :mod:`bam.mangdang.spi`. This module plays
the generic BAM trajectories on one servo and logs the response to JSON in the
schema ``bam.fit`` expects (see ``docs/identification/acquisition.rst``).

Typical use (driven by :mod:`bam.mangdang.all_record`)::

    uv run python -m bam.mangdang.record \
        --port /dev/ttyUSB0 --id 1 --mass 0.5 --arm-mass 0.02 --length 0.15 \
        --vin 12.0 --kp 80 --kd 400 --trajectory lift_and_drop --logdir data_raw

Units: positions are radians and speeds rad/s. The driver reports ``0.0`` at
the centre of the potentiometer travel, and that same zero is used for the log,
so **mount the arm so the potentiometer centre is the pendulum's downward
pose** (the zero of :class:`~bam.testbench.Pendulum`). The offset between the
two is measured during the pre-roll hold and stored as ``q_offset``.

Two logged quantities are estimates, and ``bam.fit`` uses neither:

- ``duty_cycle`` in each entry, reconstructed with the firmware's law (see
  :meth:`Recorder.compute_duty_cycle`; control_mode 0 uses the reported
  current); ``--live pwm_duty`` also logs the firmware's own duty per sample, and
- ``q_offset``, the hold error during the pre-roll at goal 0.

Current: ``load`` is the current as the AT32 reports it, the supply-side
current ``I_motor * |duty|`` (see :func:`bam.mangdang.spi.motor_current_ma`).
``--motor-current`` also reads ``pwm_duty`` after every sample and adds
``load_motor``, the motor current ``load / |duty|``; the extra read slows the
sampling (three USB round trips per sample instead of one), so it is off by
default.

Use ``--dry-run`` to exercise the whole pipeline without hardware.
"""

import argparse
import datetime
import json
import os
import subprocess
import time

import numpy as np

from bam.mangdang.spi import (
    DEFAULT_SCALE_DD,
    DEFAULT_VIN,
    LIVE,
    MIN_DUTY_FOR_CURRENT,
    PARAM,
    SpiMD300IO,
    motor_current_ma,
    pick_port,
)
from bam.mangdang.trajectories import trajectories  # BAM's set plus the module's own

#: Current cap [mA] sent in every position frame, as on the robot.
ROBOT_CUR_MA = 1500

#: The zero must leave at least this many deci-degrees of pot track on
#: both sides of the vertical: the swing reaches ~120 deg and the 50 deg dead
#: zone starts 155 deg from the centre.
ZERO_WINDOW_DD = 350

#: Fraction of the PWM range the controller is assumed to reach, inherited from
#: the measurements on the other BAM voltage-controlled servos.
DEFAULT_MAX_PWM = 1.0

#: Position-error to duty-cycle gain for the reconstructed ``duty_cycle``,
#: inherited from the Feetech STS3215. In control_mode 1 the firmware's gain is
#: ``pwm_scale * 180 / pi``, which the actuator model reads from
#: ``at32_pwm_scale``.
DEFAULT_ERROR_GAIN = 0.0104

#: Duration [s] of the pre-roll hold used to settle the pendulum and estimate the
#: mechanical zero, mirroring the 1 s hold in ``bam/feetech/record.py``.
PREROLL = 1.0

#: How often the control signal and torque-enable flag are recomputed. The
#: bridge runs no local trajectory generator, so a delay here shows up directly
#: in the fit's ``command_delay``; keep it at or below the logging period.
CONTROL_PERIOD = 0.001

#: Safety limits; they only catch an obviously wrong setup.
MAX_CURRENT_MA = 2000
MAX_ABSOLUTE_SPEED = 60.0  # rad/s


def build_parser() -> argparse.ArgumentParser:
    """Build the recorder's argument parser."""
    parser = argparse.ArgumentParser(
        description="Record a BAM identification trajectory from an MD-300-150"
    )
    parser.add_argument("--mass", type=float, required=True)
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument(
        "--arm-mass",
        "--arm_mass",
        dest="arm_mass",
        type=float,
        default=0.0,
        help="arm mass [kg] (the recorder writes the arm_mass key)",
    )
    parser.add_argument(
        "--arm-length",
        type=float,
        default=None,
        help="arm length [m] (default: --length)",
    )
    parser.add_argument(
        "--payload-offset",
        type=float,
        default=0.0,
        help="distance from the arm's end to the payload's centre [m]",
    )
    parser.add_argument(
        "--payload-inertia",
        type=float,
        default=0.0,
        help="payload inertia about its centre [kg m^2]",
    )
    parser.add_argument("--port", type=str, default=None)
    parser.add_argument("--logdir", type=str, required=True)
    parser.add_argument("--trajectory", type=str, default="lift_and_drop")
    parser.add_argument("--motor", type=str, default="md300")
    parser.add_argument("--kp", type=float, default=80)
    parser.add_argument("--kd", type=float, default=400.0)
    parser.add_argument("--vin", type=float, default=DEFAULT_VIN)
    parser.add_argument(
        "--id",
        type=int,
        required=True,
        help="bus id 1..12; in module slot k, 3k+1 abduction, 3k+2 hip, 3k+3 knee",
    )
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument(
        "--cur",
        type=int,
        default=ROBOT_CUR_MA,
        help="firmware current cap in position mode [mA] (robot: 1500)",
    )
    parser.add_argument(
        "--zero-dd",
        type=float,
        default=None,
        help="raw deci-degree reading of the pendulum vertical (bam.mangdang.zero); "
        "default: pot centre",
    )
    parser.add_argument(
        "--allow-offset",
        action="store_true",
        help="record even if the zero leaves < 350 dd of track on one side",
    )
    for name in (
        "kp_current",
        "kff_current",
        "max_pwm_duty_cycle",
        "ki_position",
        "pwm_scale",
    ):
        parser.add_argument(
            f"--{name.replace('_', '-')}",
            dest=name,
            type=float,
            default=None,
            help=f"write AT32 {name} (RAM) before the run; default: leave and log",
        )
    parser.add_argument(
        "--control-mode",
        dest="control_mode",
        type=int,
        choices=(0, 1),
        default=None,
        help="AT32 loop: 0 = legacy P+FF current loop, 1 = position PID -> PWM "
        "(voltage mode); default: leave and log",
    )
    parser.add_argument(
        "--motor-current",
        dest="motor_current",
        action="store_true",
        help="also read pwm_duty every sample and log load_motor = load/|duty| (the AT32 reports "
        "the supply-side current I_motor*|duty|); about 3x slower sampling",
    )
    parser.add_argument(
        "--live",
        type=str,
        default="",
        help="comma-separated live value names read after every sample (e.g. pwm_duty); "
        "each costs one extra round trip per sample -- for probes, not full sweeps",
    )
    parser.add_argument(
        "--supply-v", type=float, default=None, help="bench supply voltage setting [V]"
    )
    parser.add_argument(
        "--supply-ilim", type=float, default=None, help="bench supply current limit [A]"
    )
    parser.add_argument(
        "--supply-note", type=str, default="", help="free text: supply, cable, setup"
    )
    parser.add_argument(
        "--module",
        type=str,
        default="",
        help="label of the physical module (servo ids repeat across modules on the same connector)",
    )
    parser.add_argument(
        "--scale-dd",
        type=float,
        default=DEFAULT_SCALE_DD,
        help="full-scale travel in deci-degrees (3100 = 310 deg)",
    )
    parser.add_argument(
        "--error-gain",
        type=float,
        default=DEFAULT_ERROR_GAIN,
        help="kp * error * error_gain -> duty cycle",
    )
    parser.add_argument("--max-pwm", type=float, default=DEFAULT_MAX_PWM)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="do not touch hardware; synthesise feedback",
    )
    return parser


#: CLI arguments. Parsed at import time so ``python -m bam.mangdang.record``
#: behaves like every other recorder; tests monkeypatch it instead of passing
#: argv. Set to ``None`` by ``main()`` after parsing to expose a clean import.
args = build_parser().parse_args()

os.makedirs(args.logdir, exist_ok=True)

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")
if not 1 <= args.id <= 12:
    raise ValueError(f"--id must be in 1..12, got {args.id}")


class Recorder:
    """Turns a driver into the sampled state and entries the fit consumes."""

    def __init__(
        self,
        io: SpiMD300IO,
        servo: int,
        kp: float,
        error_gain: float,
        max_pwm: float,
    ):
        self.io = io
        self.servo = servo
        self.kp = kp
        self.error_gain = error_gain
        self.max_pwm = max_pwm
        #: AT32 control_mode, current-loop gains, pwm_scale and frame cap [mA]
        #: of the run (set after configure_at32): with them the reconstruction
        #: uses the firmware's own law.
        self.control_mode = 0
        self.kp_current = None
        self.kff_current = None
        self.pwm_scale = None
        self.cap_ma = None
        #: Reconstructed duty cycle of the last commanded sample, in [-1, 1].
        self.duty_cycle = 0.0
        #: Mechanical zero estimated during the pre-roll hold [rad].
        self.q_offset = 0.0

    def compute_duty_cycle(
        self, goal_position: float, position: float, current_ma: float | None = None
    ) -> float:
        """Reconstruct the firmware duty cycle from the position error.

        The bridge does not report the duty cycle, so it is recomputed with the
        firmware's law where the run's configuration is known:

        * control_mode 1: ``duty = kp * pwm_scale * error_deg``;
        * control_mode 0, given the reported current ``current_ma``:
          ``duty = kp_current * (i_set - current_ma) + kff_current * i_set``
          with ``i_set = clip(kp * error_deg, +/-cap)`` [mA];
        * otherwise ``duty = kp * error_gain * (goal - position)``,

        each clipped at ``max_pwm``. These are estimates: the D term, the
        dead band and any saturation on the board are not seen. Read
        ``pwm_duty`` over SPI instead if a measured command is needed (one
        extra transaction per sample, so acquisition is slower).
        """
        error = goal_position - position
        scale = self.pwm_scale if self.pwm_scale is not None else self.kff_current
        loop_known = self.kp_current is not None and self.kff_current is not None
        if self.control_mode == 1 and scale is not None:
            duty = self.kp * scale * np.degrees(error)
        elif self.control_mode == 0 and loop_known and current_ma is not None:
            # P + feed-forward current loop; its feedback is the reported current.
            i_set = self.kp * np.degrees(error)
            if self.cap_ma is not None:
                i_set = float(np.clip(i_set, -self.cap_ma, self.cap_ma))
            duty = self.kp_current * (i_set - current_ma) + self.kff_current * i_set
        else:
            duty = self.kp * self.error_gain * error
        self.duty_cycle = float(np.clip(duty, -self.max_pwm, self.max_pwm))
        return self.duty_cycle

    def read_data(self, response: dict | None = None) -> dict:
        """Read one sample from the module.

        :param response: The reply of a position command just sent; when
            given, it is the sample and no extra round trip is made.

        :returns: Dict with ``position`` [rad], ``speed`` [rad/s],
            ``duty_cycle``, ``load`` [mA of motor current], ``input_volts`` [V]
            and ``temp``. The ``timestamp``, ``goal_position`` and
            ``torque_enable`` keys are added by the acquisition loop, which
            straddles the sample with two clock reads.

        Position, current and velocity come from a single SPI transaction
        (:meth:`~bam.mangdang.spi.SpiMD300IO.read_sample`) — the firmware returns
        position and current together, and the velocity is differentiated from
        the position. Issuing one round-trip instead of three roughly triples
        the achievable logging rate, which keeps the fit's ``command_delay``
        estimate meaningful.
        """
        sample = self.io.read_sample(self.servo, response)
        volts = self.io.get_present_voltage([self.servo])[0]

        return {
            "position": float(sample["position"]),
            "speed": float(sample["speed"]),
            # BAM's voltage-controlled models consume `duty_cycle`; recording it
            # (rather than a voltage) keeps `vin` an independent fittable input.
            "duty_cycle": float(self.duty_cycle),
            "load": float(sample["current_mA"]),
            "input_volts": float(volts),
            "temp": decode_temp(self.io.last_res),
        }


def open_io(port: str | None):
    """Open the bridge and return the driver, with the scale synced.

    :param port: Serial device path, or ``None`` to auto-detect it.
    :returns: A :class:`~bam.mangdang.spi.SpiMD300IO` ready for position mode.
    """
    io = SpiMD300IO(
        port,
        baud=args.baud,
        scale_dd=args.scale_dd,
        dry_run=args.dry_run,
        zero_dd=args.zero_dd,
        cur_ma=args.cur,
    )
    if not args.dry_run:
        # Adopt the AT32's range_position_deg so the host and board agree on
        # what "310 degrees" means; a mismatch would silently squash the range.
        try:
            io.sync_scale_from_board(args.id)
        except (OSError, TimeoutError) as exc:
            print(
                f"warning: could not read the board scale ({exc}); "
                f"using {io.scale_dd / 10.0:.1f} deg"
            )
    margin = min(io.zero_dd, io.scale_dd - io.zero_dd)
    if margin < io.scale_dd / 2.0 - ZERO_WINDOW_DD and not args.allow_offset:
        raise SystemExit(
            f"zero {io.zero_dd:.0f} dd leaves only {margin:.0f} dd of track on one "
            f"side (need {io.scale_dd / 2.0 - ZERO_WINDOW_DD:.0f}); re-mount the arm "
            "closer to the pot centre or pass --allow-offset"
        )
    return io


def configure_at32(io, servo: int) -> dict:
    """Write the requested loop parameters, read all of them back, verify.

    ``kp_position``/``kd_position`` come from ``--kp``/``--kd``; the
    current-loop gains, ``ki_position`` and ``control_mode`` are written only
    when given on the command line. The readback of all parameters is returned (flattened into the log as
    ``at32_<name>``) so every dataset carries the configuration it was
    recorded under. Nothing is saved to the AT32 flash.
    """
    requested = {"kp_position": args.kp, "kd_position": args.kd}
    for name in (
        "kp_current",
        "kff_current",
        "max_pwm_duty_cycle",
        "ki_position",
        "pwm_scale",
        "control_mode",
    ):
        value = getattr(args, name)
        if value is not None:
            requested[name] = float(value)
    for name, value in requested.items():
        if name == "kp_position":
            io.set_P_coefficient({servo: value})
        elif name == "kd_position":
            io.set_D_coefficient({servo: value})
        else:
            io.set_param(servo, PARAM[name], value)
    if args.dry_run:
        return {name: float(v) for name, v in requested.items()}
    readback = io.dump_params(servo)
    if "control_mode" in readback and not args.dry_run:
        mode = int(readback["control_mode"])
        print(
            f"AT32 control_mode = {mode} ({'PID->PWM voltage loop' if mode else 'legacy P+FF current loop'})"
        )
    bad = {
        name: (value, readback[name])
        for name, value in requested.items()
        if abs(readback[name] - value) > 1e-6 * max(1.0, abs(value))
    }
    if bad:
        raise RuntimeError(f"AT32 parameter readback mismatch: {bad}")
    return readback


def decode_temp(res: int) -> float:
    """Decode the feedback ``res`` word into a temperature [C].

    The AT32 puts an integer in tenths of a degree there (334 -> 33.4 C).
    """
    res &= 0xFFFFFFFF
    return res / 10.0 if 0 < res < 3000 else 0.0


def read_temperature_live(io, servo: int):
    """Housing temperature [C] from live id 12, ``None`` if unanswered."""
    try:
        t = float(io.get_live(servo, LIVE["temperature_c"])["val"])
    except (OSError, TimeoutError):
        return None
    return t if 0.0 < t < 150.0 else None


def git_revision() -> str:
    """Short git revision of the checkout the recorder runs from, if any."""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        return subprocess.check_output(
            ["git", "-C", here, "rev-parse", "--short", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def settle_and_measure_zero(io, servo: int, trajectory) -> float:
    """Hold the start pose, set the gains, and estimate the mechanical zero.

    Commands the trajectory's ``t = 0`` pose while the pendulum settles, applies the
    firmware gains, then averages the position over the hold. Averaging over a
    full second rejects both the single-sample noise and the small droop that
    follows the first application of the setpoint.

    :returns: The estimated mechanical zero [rad].
    """
    goal_position, _ = trajectory(0.0)

    io.set_mode({servo: 1})
    configure_at32(io, servo)
    io.set_goal_position({servo: goal_position})
    io.enable_torque([servo])

    samples = []
    start = time.perf_counter()
    while time.perf_counter() - start < PREROLL:
        samples.append(io.get_present_position([servo])[0])
        time.sleep(CONTROL_PERIOD)

    # The zero is the position it rests at while held at the trajectory
    # origin, which is how the pendulum testbench (arm down, q = 0) is defined.
    return float(np.mean(samples)) - goal_position


def run_trajectory(
    io,
    servo: int,
    recorder: Recorder,
    trajectory,
    live: tuple = (),
    motor_current: bool = False,
) -> list[dict]:
    """Play ``trajectory`` and return the timestamped entries.

    Torque is toggled when the trajectory's enable flag flips, and the goal
    position is only commanded while torque is on — exactly like the Feetech
    recorder, so ``lift_and_drop`` still produces a free gravity drop.
    ``live`` names AT32 live values read after every sample (one extra
    transaction each) and stored under their name.

    With ``motor_current`` (``live`` must then hold ``pwm_duty``)
    the reported current ``load`` is supply-side, ``I_motor * |duty|``, and
    ``load_motor`` = ``load / |duty|`` is added. The reported current describes
    the state one transaction back, so it is divided by the duty read on the
    previous sample (the first sample uses its own).
    """
    entries: list[dict] = []
    start = time.perf_counter()
    torque_enable = False
    goal_position, _ = trajectory(0.0)
    # The AT32 answers a frame with the state it had when the PREVIOUS frame
    # arrived (a step's current appears one frame after it), so the feedback of
    # transaction k is stamped with transaction k-1's time, goal and enable.
    previous: tuple[float, float, bool] | None = None
    supply = motor_current and "pwm_duty" in live
    previous_duty: float | None = None

    while True:
        t = time.perf_counter() - start
        if t >= trajectory.duration:
            break

        new_goal, new_enable = trajectory(t)

        if new_enable != torque_enable:
            if new_enable:
                io.set_goal_position({servo: new_goal})
                io.enable_torque([servo])
            else:
                io.disable_torque([servo])
            torque_enable = new_enable
            time.sleep(CONTROL_PERIOD)

        goal_position = new_goal

        # One USB round trip per sample: while torque is on the position
        # command's own reply is the sample (the AT32 answers with the state
        # it had when the command landed); with torque off a PING refreshes
        # the feedback. Straddle the transaction with clock reads so the
        # timestamp sits in the middle, as bam.process and bam.fit assume.
        t0 = time.perf_counter() - start
        response = None
        if torque_enable:
            response = io.set_pos_dd(servo, io.rad_to_dd(goal_position))
        entry = recorder.read_data(response)
        for name in live:
            entry[name] = float(io.get_live(servo, LIVE[name])["val"])
        if supply:
            duty_now = entry["pwm_duty"]
            duty = duty_now if previous_duty is None else previous_duty
            previous_duty = duty_now
            entry["load_motor"] = motor_current_ma(entry["load"], duty)
        t1 = time.perf_counter() - start

        this = ((t0 + t1) / 2.0, float(goal_position), bool(torque_enable))
        if previous is not None:
            entry["timestamp"], entry["goal_position"], entry["torque_enable"] = (
                previous
            )
            recorder.compute_duty_cycle(
                entry["goal_position"], entry["position"], entry["load"]
            )
            entry["duty_cycle"] = float(recorder.duty_cycle)
            entries.append(entry)
        previous = this

        if abs(entry["speed"]) > MAX_ABSOLUTE_SPEED:
            print(
                f"warning: speed {entry['speed']:.1f} rad/s exceeds the "
                f"{MAX_ABSOLUTE_SPEED} rad/s safety limit"
            )
        # The motor current is what heats the winding; below
        # MIN_DUTY_FOR_CURRENT it is only a noise-amplified bound, so the
        # reading itself is checked there.
        checked = entry["load"]
        if supply and abs(previous_duty or 0.0) >= MIN_DUTY_FOR_CURRENT:
            checked = entry["load_motor"]
        if abs(checked) > MAX_CURRENT_MA:
            raise RuntimeError(
                f"current {entry['load']:.0f} mA exceeds the {MAX_CURRENT_MA} mA "
                "safety limit; aborting and releasing torque"
            )

        time.sleep(CONTROL_PERIOD)

    return entries


def return_to_zero(io, servo: int, recorder: Recorder) -> None:
    """Ramp the arm back to zero, then release torque."""
    goal = io.get_present_position([servo])[0] - recorder.q_offset
    dt = 0.01
    max_step = dt * 1.0  # rad per step

    while abs(goal) > max_step:
        goal -= np.sign(goal) * max_step
        io.set_goal_position({servo: goal + recorder.q_offset})
        time.sleep(dt)

    io.set_goal_position({servo: recorder.q_offset})
    time.sleep(0.2)
    io.disable_torque([servo])


def main() -> None:
    """Run one identification trajectory and dump the log to ``--logdir``."""
    port = None if args.dry_run else pick_port(args.port)
    io = open_io(port)
    recorder = Recorder(
        io, args.id, kp=args.kp, error_gain=args.error_gain, max_pwm=args.max_pwm
    )

    trajectory = trajectories[args.trajectory]
    date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
    filename = f"{args.logdir}/{date}.json"

    entries: list[dict] = []
    at32: dict = {}
    live_cap = None
    temp_start = 0.0
    temp_live_start = temp_live_end = None
    live = tuple(x.strip() for x in args.live.split(",") if x.strip())
    if args.motor_current and "pwm_duty" not in live:
        live = live + ("pwm_duty",)  # needed for the motor current
    for name in live:
        if name not in LIVE:
            raise SystemExit(f"unknown live value {name!r}; known: {', '.join(LIVE)}")
    try:
        recorder.q_offset = settle_and_measure_zero(io, args.id, trajectory)
        at32 = (
            io.dump_params(args.id) if not args.dry_run else configure_at32(io, args.id)
        )
        recorder.control_mode = int(at32.get("control_mode", 0) or 0)
        recorder.kp_current = at32.get("kp_current")
        recorder.kff_current = at32.get("kff_current")
        recorder.pwm_scale = at32.get("pwm_scale")
        recorder.cap_ma = args.cur
        if not args.dry_run:
            live_cap = io.get_live(args.id, LIVE["max_current_mA"])["val"]
            temp_start = decode_temp(io.last_res)
            temp_live_start = read_temperature_live(io, args.id)
        print(
            f"servo {args.id}: zero {io.zero_dd:.0f} dd, hold error "
            f"{recorder.q_offset:+.4f} rad, scale {io.scale_dd / 10.0:.1f} deg, "
            f"kp {at32.get('kp_position')} kd {at32.get('kd_position')} "
            f"kp_c {at32.get('kp_current')} kff {at32.get('kff_current')} "
            f"pwm_scale {at32.get('pwm_scale')} "
            f"max_pwm {at32.get('max_pwm_duty_cycle')} cap {live_cap} mA "
            f"mode {at32.get('control_mode')} ki {at32.get('ki_position')} "
            f"temp {temp_start:.1f} C (live {temp_live_start})"
        )

        entries = run_trajectory(
            io, args.id, recorder, trajectory, live, args.motor_current
        )
        return_to_zero(io, args.id, recorder)
        if not args.dry_run:
            temp_live_end = read_temperature_live(io, args.id)
    finally:
        # Always release the motor and flush whatever was captured, so an
        # interrupt still leaves a usable (truncated) log behind.
        try:
            io.disable_torque([args.id])
        except (OSError, TimeoutError) as exc:
            print(f"warning: could not disable torque: {exc}")

        data = {
            "mass": args.mass,
            "arm_mass": args.arm_mass,
            "length": args.length,
            # Measured pendulum geometry, the input of bam.mangdang.pendulum.
            "geometry_payload_mass": args.mass,
            "geometry_arm_mass": args.arm_mass,
            "geometry_arm_length": (
                args.length if args.arm_length is None else args.arm_length
            ),
            "geometry_payload_offset": args.payload_offset,
            "geometry_payload_inertia": args.payload_inertia,
            "kp": args.kp,
            "kd": args.kd,
            "vin": args.vin,
            "motor": args.motor,
            "trajectory": args.trajectory,
            "q_offset": recorder.q_offset,
            "error_gain": args.error_gain,
            "max_pwm": args.max_pwm,
            "servo_id": args.id,
            "module": args.module,
            # Setup and configuration record: everything the fit does not read
            # but the dataset must carry.
            "zero_dd": io.zero_dd,
            "scale_dd": io.scale_dd,
            "cur_cap_ma": args.cur,
            "at32_live_max_current_mA": live_cap,
            "supply_v": args.supply_v,
            "supply_ilim_a": args.supply_ilim,
            "supply_note": args.supply_note,
            "temp_start_c": temp_start,
            "temp_live_start_c": temp_live_start,
            "temp_live_end_c": temp_live_end,
            "live_values": list(live),
            "sample_scheme": "set_pos_reply_lag1",
            "recorder_git": git_revision(),
            **{f"at32_{name}": value for name, value in at32.items()},
            "entries": entries,
        }
        with open(filename, "w") as logfile:
            json.dump(data, logfile)
        io.close()

    print(f"recorded {len(entries)} entries -> {filename}")


if __name__ == "__main__":
    main()
