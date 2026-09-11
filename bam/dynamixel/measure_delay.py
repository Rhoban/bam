# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

"""
Measure the delay between a goal-position command being sent and the motor
actually reacting, in the very same setup as ``bam.dynamixel.record``.

The identification pipeline has a ``command_delay`` parameter (see
``bam/model.py`` and ``fractional_delay_shift`` in ``bam/simulate.py``) which is
currently *fitted* on the logs. This script measures it physically instead.

For each trial the motor is held at rest, a position step is issued, and the bus
is polled as fast as possible until the motor reacts. Several criteria are
timestamped, from the earliest (proof the order was received) to the latest
(proof the axis physically moved):

* ``write``     duration of the write call itself (packet on the wire)
* ``goalback``  goal position register reads back the new target (reception)
* ``pwm``       present PWM leaves its resting baseline (the servo acts)
* ``move``      present position leaves its resting baseline (physical motion)
* ``velocity``  present velocity becomes non-zero (heavily filtered, see
                ``velocity_compute.py``: expect ~30 ms of intrinsic sensor lag)

Polling quantizes the answer, so each criterion is reported as a bracket
``[t_previous_sample, t_detection]``: the true onset lies in between.

Two modes:

* ``--mode tight``   poll only what is needed, as fast as the bus allows. Best
                     time resolution, gives the physical truth.
* ``--mode record``  reproduce the exact cadence of ``bam/dynamixel/record.py``
                     (write goal, sleep 1 ms, then the 5 reads), and inject the
                     step at a cycle boundary. Gives the delay as it is actually
                     experienced while recording logs.

Usage:
    uv run -m bam.dynamixel.measure_delay --trials 30
    uv run -m bam.dynamixel.measure_delay --mode record --trials 20
"""

import argparse
import datetime
import json
import os
import statistics
import time

import numpy as np

# Encoder resolution of the XL330 (protocol 2.0, X series)
POSITION_UNIT = 2 * np.pi / 4096  # rad / tick
VELOCITY_UNIT = 0.229 * 2 * np.pi / 60  # rad/s / LSB
PWM_LIMIT = 885  # counts


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure the command to motion delay of a Dynamixel XL330"
    )
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--kp", type=int, default=300)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--step", type=float, default=0.3, help="Step amplitude [rad]")
    parser.add_argument("--settle", type=float, default=0.7, help="Settling time [s]")
    parser.add_argument("--timeout", type=float, default=0.3, help="Poll timeout [s]")
    parser.add_argument("--mode", type=str, default="tight", choices=["tight", "record"])
    parser.add_argument("--no-low-latency", action="store_true")
    parser.add_argument("--output", type=str, default=None)
    return parser.parse_args()


def pwm_to_duty(raw: float) -> float:
    x = float(raw)
    if x > 2**15 - 1:
        x -= 2**16
    return float(np.clip(x / PWM_LIMIT, -1.0, 1.0))


def latency_timer(port: str):
    """Read back the FTDI latency timer of a serial port, in ms (None if absent)."""
    path = f"/sys/bus/usb-serial/devices/{os.path.basename(port)}/latency_timer"
    try:
        with open(path) as f:
            return int(f.read().strip())
    except OSError:
        return None


class Bus:
    """Timestamped access to the motor, same controller as bam/dynamixel/record.py."""

    def __init__(self, port: str, id: int, low_latency: bool = True):
        if low_latency:
            # Same as DynamixelActuatorV1.__init__: the logs were recorded this way
            if os.system(f"setserial {port} low_latency") != 0:
                raise Exception(
                    "Failed to set low latency mode (you can try: sudo apt install setserial)"
                )

        from rustypot import Xl330PyController

        self.c = Xl330PyController(port, baudrate=1000000, timeout=0.01)
        self.id = id
        self.t0 = time.time()
        self.durations = []

    def now(self) -> float:
        return time.time() - self.t0

    def _timed(self, fn, *fn_args):
        """Run a bus transaction, returning (value, t_before, t_after)."""
        t_before = self.now()
        value = fn(self.id, *fn_args)
        t_after = self.now()
        self.durations.append(t_after - t_before)
        return value, t_before, t_after

    # Reads (rustypot returns a list, one element per motor)
    def position(self):
        v, a, b = self._timed(self.c.read_present_position)
        return v[0], a, b

    def pwm(self):
        v, a, b = self._timed(self.c.read_present_pwm)
        return pwm_to_duty(v[0]), a, b

    def velocity(self):
        v, a, b = self._timed(self.c.read_present_velocity)
        return v[0] * VELOCITY_UNIT, a, b

    def goal_readback(self):
        v, a, b = self._timed(self.c.read_goal_position)
        return v[0], a, b

    def voltage(self):
        v, a, b = self._timed(self.c.read_present_input_voltage)
        return v[0] / 10.0, a, b

    def temperature(self):
        v, a, b = self._timed(self.c.read_present_temperature)
        return v[0], a, b

    # Writes
    def set_goal(self, position: float):
        return self._timed(self.c.write_goal_position, position)

    def set_torque(self, enable: bool):
        return self._timed(self.c.write_torque_enable, enable)

    def set_kp(self, kp: int):
        return self._timed(self.c.write_position_p_gain, kp)


class Detector:
    """First crossing of a threshold, keeping the bracketing sample times."""

    def __init__(self, name: str, baseline: float, threshold: float):
        self.name = name
        self.baseline = baseline
        self.threshold = threshold
        self.t_last_below = None
        self.t_detected = None
        self.value = None

    @property
    def triggered(self) -> bool:
        return self.t_detected is not None

    def update(self, value: float, t: float):
        if self.triggered:
            return
        if abs(value - self.baseline) > self.threshold:
            self.t_detected = t
            self.value = value
        else:
            self.t_last_below = t


def goto(bus: Bus, position: float, settle: float):
    """Hold a goal position for `settle` seconds (the servo is left there)."""
    bus.set_goal(position)
    end = time.time() + settle
    while time.time() < end:
        bus.set_goal(position)
        time.sleep(0.01)


def measure_baseline(bus: Bus, samples: int = 20):
    """Sample position/pwm at rest, returning (mean, max deviation) for each."""
    positions, pwms = [], []
    for _ in range(samples):
        positions.append(bus.position()[0])
        pwms.append(bus.pwm()[0])

    def stats(values):
        mean = statistics.fmean(values)
        return mean, max(abs(v - mean) for v in values)

    return stats(positions), stats(pwms)


CRITERIA = ("goalback", "pwm", "tick", "move", "velocity")

# Which register a trial has to poll to observe a given criterion
SIGNALS = {
    "goalback": "goalback",
    "pwm": "pwm",
    "tick": "position",
    "move": "position",
    "velocity": "velocity",
}


def make_detectors(q0, target, pos_base, pos_noise, pwm_base, pwm_noise):
    """Detectors for all criteria, with noise-aware thresholds."""
    return {
        # First single encoder tick: earliest physical proof of motion
        "tick": Detector("tick", pos_base, max(0.5 * POSITION_UNIT, 2 * pos_noise)),
        # 3 encoder ticks, or 4x the observed rest noise if the axis is shaky
        "move": Detector("move", pos_base, max(3 * POSITION_UNIT, 4 * pos_noise)),
        "pwm": Detector("pwm", pwm_base, max(0.02, 4 * pwm_noise)),
        "velocity": Detector("velocity", 0.0, 1.5 * VELOCITY_UNIT),
        # Goal readback: triggers once the register has moved halfway to the target
        "goalback": Detector("goalback", q0, abs(target - q0) / 2.0),
    }


def run_trial_tight(bus: Bus, q0: float, target: float, args, signal: str) -> dict:
    """Step the goal, then poll a *single* register as fast as the bus allows.

    Polling several registers per cycle would divide the time resolution by as
    many: one signal per trial keeps it at one bus transaction (~1 ms).
    """
    (pos_base, pos_noise), (pwm_base, pwm_noise) = measure_baseline(bus)
    detectors = make_detectors(q0, target, pos_base, pos_noise, pwm_base, pwm_noise)
    # Only the criteria observable through the polled register are measured here
    detectors = {k: d for k, d in detectors.items() if SIGNALS[k] == signal}

    read = {
        "position": bus.position,
        "pwm": bus.pwm,
        "velocity": bus.velocity,
        "goalback": bus.goal_readback,
    }[signal]

    samples = []
    _, t_write_before, t_write_after = bus.set_goal(target)

    while bus.now() - t_write_before < args.timeout:
        value, ta, tb = read()
        t = (ta + tb) / 2.0
        for detector in detectors.values():
            detector.update(value, t)
        samples.append({"t": t - t_write_before, signal: value})

        if all(d.triggered for d in detectors.values()):
            break

    trial = pack_trial(bus, q0, target, t_write_before, t_write_after, detectors,
                       samples, pos_base, pos_noise, pwm_base, pwm_noise)
    trial["signal"] = signal
    return trial


def run_trial_record(bus: Bus, q0: float, target: float, args) -> dict:
    """Same as above, but with the exact cadence of bam/dynamixel/record.py.

    The recording loop writes the goal, sleeps 1 ms, then performs five separate
    reads. The delay seen here is the one that is actually baked into the logs.
    """
    (pos_base, pos_noise), (pwm_base, pwm_noise) = measure_baseline(bus)

    detectors = make_detectors(q0, target, pos_base, pos_noise, pwm_base, pwm_noise)

    samples = []
    _, t_write_before, t_write_after = bus.set_goal(target)
    time.sleep(0.001)

    while bus.now() - t_write_before < args.timeout:
        # The five reads of the recording loop, in the same order
        position, ta, tb = bus.position()
        t_pos = (ta + tb) / 2.0
        velocity, ta, tb = bus.velocity()
        t_vel = (ta + tb) / 2.0
        pwm, ta, tb = bus.pwm()
        t_pwm = (ta + tb) / 2.0
        bus.voltage()
        bus.temperature()
        goal_rb, ta, tb = bus.goal_readback()
        t_goal = (ta + tb) / 2.0

        detectors["tick"].update(position, t_pos)
        detectors["move"].update(position, t_pos)
        detectors["pwm"].update(pwm, t_pwm)
        detectors["velocity"].update(velocity, t_vel)
        detectors["goalback"].update(goal_rb, t_goal)

        samples.append(
            {
                "t": t_pos - t_write_before,
                "position": position,
                "pwm": pwm,
                "speed": velocity,
                "goal_readback": goal_rb,
            }
        )

        if all(d.triggered for d in detectors.values()):
            break

        # The recording loop keeps writing the goal every cycle
        bus.set_goal(target)
        time.sleep(0.001)

    return pack_trial(bus, q0, target, t_write_before, t_write_after, detectors, samples,
                      pos_base, pos_noise, pwm_base, pwm_noise)


def pack_trial(bus, q0, target, t_write_before, t_write_after, detectors, samples,
               pos_base, pos_noise, pwm_base, pwm_noise) -> dict:
    """Express all detections relatively to the moment the write call started."""
    trial = {
        "q0": q0,
        "target": target,
        "write_duration": t_write_after - t_write_before,
        "position_baseline": pos_base,
        "position_noise": pos_noise,
        "pwm_baseline": pwm_base,
        "pwm_noise": pwm_noise,
        "samples": samples,
    }
    for name, d in detectors.items():
        trial[name] = {
            # Upper bound: the sample that saw the change
            "delay": None if d.t_detected is None else d.t_detected - t_write_before,
            # Lower bound: the last sample that did not
            "delay_lower": (
                None if d.t_last_below is None else d.t_last_below - t_write_before
            ),
            "threshold": d.threshold,
        }
    return trial


HARDWARE_ERRORS = {
    0: "input voltage out of the [min, max] voltage limits",
    2: "overheating",
    3: "motor encoder error",
    4: "electrical shock / insufficient power",
    5: "overload",
}


def check_ready(bus: Bus):
    """Fail fast (and explain why) if the servo refuses to hold torque.

    A servo latching a hardware error silently ignores torque enable: every
    write is acknowledged, nothing ever moves, and the measurement would report
    "never detected" for every trial.
    """
    if bus.c.read_torque_enable(bus.id)[0]:
        return

    error = bus.c.read_hardware_error_status(bus.id)[0]
    volts = bus.c.read_present_input_voltage(bus.id)[0] / 10.0
    v_min = bus.c.read_min_voltage_limit(bus.id)[0] / 10.0
    v_max = bus.c.read_max_voltage_limit(bus.id)[0] / 10.0
    reasons = [text for bit, text in HARDWARE_ERRORS.items() if error & (1 << bit)]

    raise Exception(
        f"Torque enable was refused (hardware error status: {error:#04x}"
        + (f" - {', '.join(reasons)}" if reasons else "")
        + f"). Input voltage is {volts} V for limits [{v_min}, {v_max}] V. "
        "Fix the cause, then power-cycle (or reboot) the servo to clear the latched error."
    )


def summarize(values: list) -> dict:
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {
        "n": len(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def main():
    args = parse_args()

    bus = Bus(args.port, args.id, low_latency=not args.no_low_latency)

    # Same initialisation as bam/dynamixel/record.py: 1 s of torque enable + kp
    start = time.time()
    while time.time() - start < 1.0:
        bus.set_torque(True)
        bus.set_kp(args.kp)

    check_ready(bus)

    q0 = bus.position()[0]

    info = {
        "port": args.port,
        "id": args.id,
        "kp": args.kp,
        "baudrate": 1000000,
        "mode": args.mode,
        "step": args.step,
        "latency_timer": latency_timer(args.port),
        "return_delay_time": bus.c.read_return_delay_time(args.id)[0],
        "firmware_version": bus.c.read_firmware_version(args.id)[0],
        "q0": q0,
    }
    print(json.dumps(info, indent=2))

    # In tight mode a trial only polls one register, so the signals are cycled
    signals = ["pwm", "position", "velocity", "goalback"]
    trials = []

    try:
        for i in range(args.trials):
            signal = signals[i % len(signals)]
            # Alternate the direction so gravity / friction bias cancels out
            sign = 1 if (i // len(signals)) % 2 == 0 else -1
            goto(bus, q0, args.settle)
            bus.durations.clear()

            target = q0 + sign * args.step
            if args.mode == "tight":
                trial = run_trial_tight(bus, q0, target, args, signal)
            else:
                trial = run_trial_record(bus, q0, target, args)
            trial["index"] = i
            trial["poll_period"] = summarize(bus.durations)
            trials.append(trial)

            print(
                f"[{i + 1}/{args.trials}] "
                + " ".join(
                    f"{name}={fmt_ms(trial[name]['delay'])}"
                    for name in CRITERIA
                    if name in trial
                )
            )
    finally:
        goto(bus, q0, 0.3)
        bus.set_torque(False)

    report(info, trials, args)


def fmt_ms(value) -> str:
    return "  n/a" if value is None else f"{value * 1000:5.1f}ms"


def report(info: dict, trials: list, args):
    summary = {
        name: {
            "detected": summarize([t[name]["delay"] for t in trials if name in t]),
            "lower_bound": summarize(
                [t[name]["delay_lower"] for t in trials if name in t]
            ),
        }
        for name in CRITERIA
    }
    summary["write_duration"] = summarize([t["write_duration"] for t in trials])
    summary["poll_transaction"] = summarize(
        [t["poll_period"]["median"] for t in trials if t["poll_period"]]
    )

    print()
    print(f"=== {args.mode} mode, {len(trials)} trials, kp={info['kp']}, "
          f"step={args.step} rad, latency_timer={info['latency_timer']} ===")
    print(f"{'criterion':<12} {'median':>9} {'mean':>9} {'std':>9} "
          f"{'min':>9} {'max':>9}   (lower bound median)")
    for name in CRITERIA:
        s = summary[name]["detected"]
        if s is None:
            print(f"{name:<12}     never detected")
            continue
        lower = summary[name]["lower_bound"]
        lower_str = "n/a" if lower is None else f"{lower['median'] * 1000:.1f}ms"
        print(
            f"{name:<12} {s['median'] * 1000:8.2f}ms {s['mean'] * 1000:8.2f}ms "
            f"{s['std'] * 1000:8.2f}ms {s['min'] * 1000:8.2f}ms "
            f"{s['max'] * 1000:8.2f}ms   {lower_str}"
        )
    for name in ("write_duration", "poll_transaction"):
        s = summary[name]
        if s is not None:
            print(f"{name:<12} {s['median'] * 1000:8.2f}ms {s['mean'] * 1000:8.2f}ms "
                  f"{s['std'] * 1000:8.2f}ms {s['min'] * 1000:8.2f}ms "
                  f"{s['max'] * 1000:8.2f}ms")

    output = args.output
    if output is None:
        date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
        output = f"/tmp/delay_{args.mode}_{date}.json"
    with open(output, "w") as f:
        json.dump({"info": info, "summary": summary, "trials": trials}, f)
    print(f"\nSaved to {output}")


if __name__ == "__main__":
    main()
