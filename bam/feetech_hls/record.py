# Copyright 2026 BAM Project
# Licensed under the Apache License, Version 2.0 (the "License");

import argparse
import datetime
import json
import os
import time


from .driver import FeetechHLSDriver
from bam.trajectory import trajectories




def main():
    parser = argparse.ArgumentParser(
        description="Record a BAM trajectory with an HLS2915 in Position mode"
    )
    parser.add_argument("--mass", type=float, required=True, help="Tip mass [kg]")
    parser.add_argument("--arm-mass", type=float, required=True, help="Arm mass [kg]")
    parser.add_argument("--length", type=float, required=True, help="Pendulum length [m]")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--id", type=int, default=1, help="Servo ID")
    parser.add_argument("--logdir", required=True, help="Raw-log output directory")
    parser.add_argument("--trajectory", default="lift_and_drop", help="Trajectory name")
    parser.add_argument("--motor", default="feetech_hls2915", help="Motor identifier")
    parser.add_argument(
        "--kp",
        type=int,
        default=8,
        help="HLS Position-mode firmware Kp register [0, 255]",
    )
    parser.add_argument(
        "--vin",
        type=float,
        default=None,
        help="Supply voltage [V]; default reads the servo telemetry",
    )
    parser.add_argument("--cooldown", type=float, default=5.0, help="Minimum cooling wait [s]")
    parser.add_argument(
        "--cooldown-temp",
        type=float,
        default=42.0,
        help="Start only at or below this temperature [C]",
    )
    parser.add_argument("--goal-min", type=float, default=-1.20, help="Lower target clamp [rad]")
    parser.add_argument("--goal-max", type=float, default=1.20, help="Upper target clamp [rad]")
    args = parser.parse_args()

    if args.trajectory not in trajectories:
        raise ValueError(f"Unknown trajectory: {args.trajectory}. Available: {list(trajectories)}")
    os.makedirs(args.logdir, exist_ok=True)

    trajectory = trajectories[args.trajectory]
    driver = FeetechHLSDriver(port=args.port, baudrate=1_000_000)

    print(f"* Connecting to servo ID {args.id} on {args.port}...")
    if not driver.ping(args.id):
        raise ConnectionError(f"Could not connect to servo ID {args.id} on {args.port}")

    def release_torque(attempts: int = 5) -> bool:
        """Stop output, including the old current command if the servo has it."""
        for _ in range(attempts):
            try:
                if driver.read_bytes(args.id, driver.ADDR_MODE, 1)[0] == 2:
                    driver.set_goal_current(args.id, 0.0)
                driver.set_torque_enable(args.id, False)
                if driver.read_bytes(args.id, driver.ADDR_TORQUE_ENABLE, 1)[0] == 0:
                    return True
            except Exception:
                pass
            time.sleep(0.05)
        return False

    try:
        # Never change configuration while driving. Mode and Kp are RAM-only for
        # this experiment; the production servo configuration is left untouched.
        if not release_torque():
            raise RuntimeError("Could not disable HLS torque before recording")
        driver.set_mode(args.id, 0)
        driver.write_bytes(args.id, driver.ADDR_LOCK, [1])
        # Position mode needs a nonzero running-speed ceiling: ADDR_GOAL_SPEED=0
        # freezes motion on this firmware. 16000 steps/s is well above bench
        # trajectories (HLS logs peak at ~10 rad/s). Acceleration 0 = unlimited.
        driver.write_bytes(args.id, driver.ADDR_GOAL_SPEED, [0x80, 0x3E])
        driver.write_bytes(args.id, driver.ADDR_ACCELERATION, [0])
        driver.set_position_kp(args.id, args.kp)
        time.sleep(0.02)
        mode = driver.read_bytes(args.id, driver.ADDR_MODE, 1)[0]
        kp = driver.read_bytes(args.id, driver.ADDR_KP, 1)[0]
        if mode != 0 or kp != args.kp:
            raise RuntimeError(f"HLS position-mode setup failed: mode={mode}, kp={kp}")

        cooldown_started = time.time()
        while True:
            status = driver.read_status(args.id)
            elapsed = time.time() - cooldown_started
            if elapsed >= args.cooldown and status["temp"] <= args.cooldown_temp:
                break
            if status["temp"] >= 68.0:
                raise RuntimeError(f"Temperature {status['temp']}°C is too high to record")
            print(
                f"* Cooling {elapsed:4.0f}s ({status['temp']}°C, "
                f"target <= {args.cooldown_temp:.0f}°C) ...",
                end="\r",
            )
            time.sleep(3.0)
        print(f"* Position mode Kp={kp}; cooled to {status['temp']}°C{' ' * 20}")

        vin = round(status["input_volts"], 1) if args.vin is None else args.vin
        start_goal, _ = trajectory(0.0)
        start_goal = max(args.goal_min, min(args.goal_max, start_goal))
        commanded_goal = status["position"]
        driver.set_goal_position(args.id, commanded_goal)
        driver.set_torque_enable(args.id, True)
        time.sleep(0.02)

        # Ramp the firmware target, rather than making Position mode jump to the
        # trajectory start from an arbitrary stiction-rest pose.
        settled = 0
        pre_steps = 0
        deadline = time.perf_counter() + 6.0
        while time.perf_counter() < deadline:
            commanded_goal += max(-0.01, min(0.01, start_goal - commanded_goal))
            driver.set_goal_position(args.id, commanded_goal)
            status = driver.read_status(args.id)
            position = status["position"]
            if abs(start_goal - position) < 0.02 and abs(status["speed"]) < 0.15:
                settled += 1
                if settled >= 3:
                    break
            else:
                settled = 0
            pre_steps += 1
            time.sleep(0.005)
        print(
            f"* Settled {status['position']:+.4f} rad "
            f"(target {start_goal:+.4f}, residual {abs(start_goal - status['position']):.4f} rad, "
            f"{pre_steps} steps, speed {status['speed']:+.3f})"
        )

        data = {
            "mass": args.mass,
            "arm_mass": args.arm_mass,
            "length": args.length,
            "kp": kp,
            "vin": vin,
            "motor": args.motor,
            "trajectory": args.trajectory,
            "control_mode": "position_voltage",
            "entries": [],
        }

        torque_enabled = True
        start = time.perf_counter()
        while True:
            now = time.perf_counter()
            t = now - start
            if t >= trajectory.duration:
                break

            goal_position, desired_torque_enable = trajectory(t)
            goal_position = max(args.goal_min, min(args.goal_max, goal_position))
            if desired_torque_enable != torque_enabled:
                driver.set_torque_enable(args.id, desired_torque_enable)
                torque_enabled = desired_torque_enable
            driver.set_goal_position(args.id, goal_position)
            status = driver.read_status(args.id)

            # HLS reports PWM in its motor convention, opposite to this rig's
            # calibrated positive joint direction. Store BAM's joint-positive
            # voltage; fitting independently recomputes it from goal/position.
            control_volts = -status["input_volts"] * status["load"] / 100.0
            data["entries"].append(
                {
                    "timestamp": float(t),
                    "position": float(status["position"]),
                    "speed": float(status["speed"]),
                    "control": float(control_volts),
                    "measured_current": float(status["current"]),
                    "goal_position": float(goal_position),
                    "torque_enable": bool(desired_torque_enable),
                    "load": float(status["load"]),
                    "input_volts": float(status["input_volts"]),
                    "temp": float(status["temp"]),
                }
            )
            time.sleep(max(0.0, 0.005 - (time.perf_counter() - now)))
    finally:
        release_torque()
        driver.close()

    filename = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
    output = os.path.join(args.logdir, f"{filename}_{args.trajectory}.json")
    with open(output, "w") as stream:
        json.dump(data, stream, indent=2)
    print(f"* Completed: {len(data['entries'])} entries -> {output}")


if __name__ == "__main__":
    main()
