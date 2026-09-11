# Copyright 2026 BAM Project
# Licensed under the Apache License, Version 2.0 (the "License");

import argparse
import datetime
import json
import os
import time
import numpy as np

from .actuator import pd_current
from .driver import FeetechHLSDriver
from bam.trajectory import trajectories


def angle_wrap(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def main():
    arg_parser = argparse.ArgumentParser(description="Record BAM trajectory using Feetech HLS servo")
    arg_parser.add_argument("--mass", type=float, required=True, help="Tip mass in kg")
    arg_parser.add_argument(
        "--arm-mass",
        type=float,
        required=True,
        help="Mass of the swinging arm itself [kg]. Required, not defaulted: "
             "Pendulum's gravity and inertia terms both scale with it, so 0.0 is "
             "never right and fails silently.",
    )
    arg_parser.add_argument("--length", type=float, required=True, help="Pendulum length in meters")
    arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port")
    arg_parser.add_argument("--id", type=int, default=1, help="Servo ID")
    arg_parser.add_argument("--logdir", type=str, required=True, help="Output directory for raw logs")
    arg_parser.add_argument("--trajectory", type=str, default="lift_and_drop", help="Trajectory name")
    arg_parser.add_argument("--motor", type=str, default="feetech_hls", help="Motor identifier")
    arg_parser.add_argument(
        "--kp",
        type=float,
        default=3.0,
        help="Proportional gain in REAL A/rad. Integer values only, so the run "
             "can later be held out with bam.fit --validation_kp.",
    )
    arg_parser.add_argument(
        "--damping",
        type=float,
        default=0.08,
        help="Damping gain. Measured on this rig: 0.5 (the value inherited from "
             "the much larger eRob joint) saturated the 0.36 A clamp on 82%% of "
             "steps, while 0.08 clips only 9%% and cuts the tracking error 4x. "
             "0.08 is also the physically sensible point: zeta = damping * "
             "sqrt(kt / 4J) = 0.94, i.e. essentially critically damped.",
    )
    arg_parser.add_argument(
        "--vin",
        type=float,
        default=None,
        help="Supply voltage [V]. Default: read the servo's own bus voltage.",
    )
    arg_parser.add_argument(
        "--cooldown",
        type=float,
        default=5.0,
        help="Minimum wait before recording [s]; the recording also waits until "
             "the servo is at or below --cooldown-temp.",
    )
    arg_parser.add_argument(
        "--cooldown-temp",
        type=float,
        default=42.0,
        help="Start recording only once the servo is at or below this "
             "temperature [C]. The rig sheds about 0.13 C/s when idle, so "
             "back-to-back runs at a short fixed wait cook the servo: sixteen of "
             "them tripped the firmware's thermal limit and it stopped answering "
             "the bus. Staying in one thermal band also keeps the copper "
             "resistance (0.39%%/K) consistent across logs that share one fitted "
             "parameter set.",
    )
    arg_parser.add_argument(
        "--goal-min",
        type=float,
        default=-1.75,
        help="Lower clamp on the commanded angle [rad]. This rig has a hard "
             "mechanical stop: measured, the arm reaches +1.44 rad but no "
             "further, while the negative side is free to -1.76. Commands past "
             "the stop stall the arm with the current saturated, and bam's "
             "simulator has no joint limit, so the model cannot reproduce those "
             "segments - it can only explain 'it did not move' as enormous "
             "friction, which biases the fitted friction terms. Clamping the "
             "reference adapts the trajectory to the rig, as bam's docs allow.",
    )
    arg_parser.add_argument(
        "--goal-max",
        type=float,
        default=1.35,
        help="Upper clamp on the commanded angle [rad]; see --goal-min.",
    )
    arg_parser.add_argument(
        "--angle-offset",
        type=float,
        default=0.0,
        help="Constant added to the LOGGED angles (position and goal) [rad]; the "
             "angles sent to the servo are untouched, so the motion is unchanged. "
             "Corrects the frame so that the logged zero is the true vertical. "
             "Measured on this rig with the friction-cancelling sweep (drive the "
             "arm slowly up and down, average the two currents at each angle so "
             "friction cancels, then fit mgl*sin(q - q0)): the vertical sits at "
            "-0.179 rad (mgl = 0.131 N.m, against the 0.122 expected from the "
            "CAD). bam.model gives q_offset only +-0.1, so without this the fit "
            "pushes q_offset onto that bound and cannot phase the gravity term "
            "properly - it did, in all six fits, before this option existed.",
    )
    arg_parser.add_argument(
        "--max-amps",
        type=float,
        default=0.36,
        help="Current saturation limit in REAL amperes. 0.36 A keeps the duty "
             "cycle near 22%% (~1 W in the winding); the servo's measured "
             "maximum is 1.37 A, past which it only heats up.",
    )
    args = arg_parser.parse_args()

    os.makedirs(args.logdir, exist_ok=True)
    if args.trajectory not in trajectories:
        raise ValueError(f"Unknown trajectory: {args.trajectory}. Available: {list(trajectories.keys())}")

    trajectory = trajectories[args.trajectory]
    driver = FeetechHLSDriver(port=args.port, baudrate=1000000)

    print(f"* Connecting to servo ID {args.id} on {args.port}...")
    if not driver.ping(args.id):
        raise ConnectionError(f"Could not connect to servo ID {args.id} on {args.port}")

    def release_torque(attempts: int = 5):
        """Zero the current and disable torque, retrying until it takes.

        Retried on purpose: if this fails, the servo keeps driving with the last
        commanded current and heats up on its own.
        """
        for attempt in range(attempts):
            try:
                driver.set_goal_current(args.id, 0.0)
                driver.set_torque_enable(args.id, False)
                # Confirm it actually took effect.
                if driver.read_bytes(args.id, driver.ADDR_TORQUE_ENABLE, 1)[0] == 0:
                    return True
            except Exception:
                pass
            time.sleep(0.05)
        print("⚠️ warning: could not confirm torque release after "
              f"{attempts} attempts — check the servo!", flush=True)
        return False

    # Cooldown between runs: wait for the servo to come back down to a target
    # temperature.
    #
    # A fixed wait is not enough: measured on this rig the servo sheds about
    # 0.13 C/s when idle, so cooling from 78 C to 42 C takes ~4.5 minutes. Sixteen
    # back-to-back runs at an 18 s cooldown pushed it past the firmware's thermal
    # limit (status bit 0x04), after which it stopped answering the bus and the
    # remaining runs failed. Cooling to a target keeps the whole batch in one
    # thermal band, which matters because copper resistance drifts 0.39%/K and a
    # single parameter set has to explain every log.
    #
    # Safety net: if the temperature *rises* while waiting, something is still
    # driving the motor - a bus glitch can leave torque enabled with the last
    # commanded current after a failed release. Release again instead of waiting
    # forever on a servo that is heating itself.
    cooldown_started = time.time()
    held = None
    while True:
        temp = driver.read_status(args.id)["temp"]
        elapsed = time.time() - cooldown_started
        if held is not None and temp > held + 1.0:
            print(f"\n* 温度在等待中升到 {temp}°C —— 疑似未释放，强制卸载")
            release_torque()
            held = None
        else:
            held = temp
        if elapsed >= args.cooldown and temp <= args.cooldown_temp:
            break
        if temp >= 68.0:
            raise RuntimeError(f"温度 {temp}°C 逼近过热保护线 (70°C)，停止录制以免损伤舵机")
        if elapsed > 600.0:
            print(f"\n* 警告: 等待 {elapsed:.0f}s 后仍为 {temp}°C，继续录制")
            break
        print(f"* 冷却中 {elapsed:4.0f}s ({temp}°C, 目标 ≤{args.cooldown_temp:.0f}°C) ...", end="\r")
        time.sleep(3.0)
    print(f"* 冷却完成 {elapsed:.0f}s ({temp}°C){' ' * 28}")

    # Set Mode 2 (Constant Current) and enable torque
    driver.set_mode(args.id, 2)
    time.sleep(0.02)
    driver.set_torque_enable(args.id, True)
    time.sleep(0.02)

    # The pendulum angle is defined with 0 rad at the arm hanging straight down,
    # and we log the servo's own position directly - the same convention as
    # bam's other recorders. No per-run startup offset: gearbox stiction lets the
    # arm rest anywhere in a band (13 degrees wide on this rig), so "wherever it
    # happens to be" would inject a differently-phased gravity term into every
    # log. Run tools/calibrate_zero.py once instead, with the arm held vertical.
    init_st = driver.read_status(args.id)
    # The supply voltage sets the current loop's voltage-limited window in the
    # model, so record what the servo actually sees instead of a fixed default.
    vin = round(init_st["input_volts"], 1) if args.vin is None else args.vin
    start_angle = angle_wrap(init_st["position"])
    # A calibrated zero still leaves the arm free to park anywhere inside its
    # stiction band, which on this rig is about +-0.8 rad wide (friction is
    # comparable to the gravity torque), so only a start angle far outside that
    # band indicates a zero that was never calibrated at all.
    if abs(start_angle) > 1.5:
        print(f"* 警告: 起始角度 {start_angle:+.3f} rad 远超静摩擦带 —— 零点很可能还没校准。")
        print("         请先把摆杆扶正，运行: python3 tools/calibrate_zero.py")
    print(f"* 起始角度 {start_angle:+.4f} rad (舵机自身零点)")
    print(f"* Supply voltage: {vin:.1f} V (servo bus reads {init_st['input_volts']:.1f} V)")

    # Bring the arm to the trajectory's starting angle before logging anything.
    #
    # Each run otherwise begins from wherever the arm happened to park, which on
    # this rig can be anywhere inside its stiction band - about +-0.8 rad here,
    # because friction is comparable to the gravity torque. Successive runs of
    # the same trajectory then started up to 0.5 rad away from the goal, so the
    # first ~0.5 s of every log was a clamp-saturated catch-up transient rather
    # than tracking, and each log's initial condition was different.
    #
    # A no-op when the arm already starts at the goal (the common case for a rig
    # whose friction is small relative to gravity), so it is safe in general.
    start_goal, _ = trajectory(0.0)
    settle_amps = min(0.15, args.max_amps)   # gentle: avoid a bang-bang approach
    t_pre = time.perf_counter()
    pre_steps = 0
    confirmed = 0
    # Require the settled condition to hold for several consecutive samples:
    # catching a single instant where the arm happens to be slow lets it drift
    # away again before logging starts.
    while time.perf_counter() - t_pre < 6.0:
        st = driver.read_status(args.id)
        pos = angle_wrap(st["position"])
        if abs(start_goal - pos) < 0.02 and abs(st["speed"]) < 0.15:
            confirmed += 1
            if confirmed >= 3:
                break
        else:
            confirmed = 0
        cmd = pd_current(start_goal, pos, st["speed"], args.kp, args.damping)
        driver.set_goal_current(args.id, max(-settle_amps, min(settle_amps, cmd)))
        pre_steps += 1
        time.sleep(0.005)
    st = driver.read_status(args.id)
    print(f"* 已就位 {angle_wrap(st['position']):+.4f} rad (轨迹起点 {start_goal:+.4f}, "
          f"残差 {abs(start_goal - angle_wrap(st['position'])):.4f} rad, {pre_steps} 步, "
          f"速度 {st['speed']:+.3f})")

    data = {
        "mass": args.mass,
        "arm_mass": args.arm_mass,
        "length": args.length,
        "kp": args.kp,
        "damping": args.damping,
        "vin": vin,
        "motor": args.motor,
        "trajectory": args.trajectory,
        "max_amps": args.max_amps,
        "entries": [],
    }

    start = time.perf_counter()
    try:
        while True:
            now = time.perf_counter()
            t = now - start
            if t >= trajectory.duration:
                break

            goal_pos, torque_enable = trajectory(t)
            # Adapt the reference to what the rig can actually reach: past the
            # stop the arm simply stalls, and the model has no way to represent
            # that. The clamped value is both commanded and logged, so the
            # identification replays the same reference.
            goal_pos = max(args.goal_min, min(args.goal_max, goal_pos))
            st = driver.read_status(args.id)
            pos = angle_wrap(st["position"])
            spd = st["speed"]

            # Current command. The law itself is shared with the model, so the
            # identification replays exactly the controller that produced the log.
            if torque_enable:
                cmd_amps = pd_current(goal_pos, pos, spd, args.kp, args.damping)
                cmd_amps = max(-args.max_amps, min(args.max_amps, cmd_amps))
            else:
                cmd_amps = 0.0

            driver.set_goal_current(args.id, cmd_amps if torque_enable else 0.0)

            entry = {
                "timestamp": float(t),
                "position": float(pos + args.angle_offset),
                "speed": float(spd),
                "control": float(cmd_amps),
                "measured_current": float(st["current"]),
                "goal_position": float(goal_pos + args.angle_offset),
                "torque_enable": bool(torque_enable),
                "load": float(st["load"]),
                "input_volts": float(st["input_volts"]),
                "temp": float(st["temp"]),
            }
            data["entries"].append(entry)

            # Target 200 Hz
            elapsed = time.perf_counter() - now
            time.sleep(max(0.0, 0.005 - elapsed))
    finally:
        # Never mask the original exception, and never give up on zeroing the
        # torque: a comms glitch here would otherwise leave the motor driving
        # with the last commanded current, heating itself indefinitely (that
        # happened, and the run's own cooldown then waited on a rising
        # temperature until the batch was stopped by hand).
        release_torque()
        driver.close()

    date_str = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
    out_file = os.path.join(args.logdir, f"{date_str}_{args.trajectory}.json")
    with open(out_file, "w") as f:
        json.dump(data, f, indent=2)

    print(f"* Completed! Log saved with {len(data['entries'])} entries to: {out_file}")


if __name__ == "__main__":
    main()
