# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import json
import datetime
import os
import numpy as np
import argparse
import time
import struct
from .dynamixel import DynamixelActuatorV1, DynamixelXL320
from bam.trajectory import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--arm-mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="lift_and_drop")
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--kp", type=int, default=32)
arg_parser.add_argument("--vin", type=float, default=15.0)
args = arg_parser.parse_args()

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

_XL330_MOTORS = {"xl330", "xl330i", "xl330m288t-MG"}

# Registers forced before recording so the bench matches the robot's servo config
# (microduck writes I = D = 0 at startup and asserts pwm_slope = 255 and
# current_limit = 1000 mA). The factory pwm_slope (140) rate-limits the output
# voltage, which BAM does not model.
_XL330_ROBOT_REGISTERS = {
    "xl330m288t-MG": {
        "position_i_gain": 0,
        "position_d_gain": 0,
        "pwm_slope": 255,
        "current_limit": 1000,
        # Input-voltage bit cleared: a 2S supply sits above the default 7.0 V
        # Max Voltage Limit, which would otherwise latch torque off (a firmware
        # update can restore the factory 53). A fault already latched needs a reboot.
        "shutdown": 52,
    },
}

# The metal-gear servos run ROBOTIS's custom "impact guard" firmware V4, the one that
# enforces current_limit in position control mode (the factory firmware only does so
# in current-based modes). Its guard is switched by Impact Guard Enable (register 67,
# RAM, off at power-up); it is kept off here, as on the robot (bus.impact_guard = false).
_XL330_REQUIRED_FIRMWARE = {"xl330m288t-MG": 4}
_IMPACT_GUARD_ENABLE_ADDR = 67

# XL330-specific conversion helpers
_XL330_RADS_PER_SEC_PER_COUNT = 0.229 * (2.0 * np.pi / 60.0)
_XL330_PWM_LIMIT = 885  # XL330 Present PWM limit (counts)


def convert_xl330_velocity(raw_signed: float) -> float:
    return float(raw_signed) * _XL330_RADS_PER_SEC_PER_COUNT


def convert_xl330_pwm_to_duty(raw: float) -> float:
    x = float(raw)
    if x > 2**15 - 1:
        x -= 2**16
    return np.clip(x / _XL330_PWM_LIMIT, -1.0, 1.0)


# Present PWM (124, int16), Present Current (126, int16, mA), Present Velocity
# (128, int32, 0.229 rpm) and Present Position (132, int32, 4096 counts/rev, 2048
# at 0 rad) are contiguous: one 12-byte read instead of one transaction each.
_XL330_STATE_ADDR = 124
_XL330_STATE_FORMAT = "<hhii"
_XL330_SLOW_READ_PERIOD = 20


def read_xl330_state(c, ID) -> dict:
    raw = bytes(c.read_raw_data(ID, _XL330_STATE_ADDR, 12))
    pwm, current_ma, velocity, position = struct.unpack(_XL330_STATE_FORMAT, raw)
    return {
        "position": (position - 2048) * 2.0 * np.pi / 4096,
        "speed": convert_xl330_velocity(velocity),
        "load": float(np.clip(pwm / _XL330_PWM_LIMIT, -1.0, 1.0)),
        # Supply-side (bus) current [A]: the XL330 senses it before the H-bridge
        "current": current_ma / 1000.0,
    }


trajectory = trajectories[args.trajectory]

# ── Controller initialisation ────────────────────────────────────────────────
if args.motor in _XL330_MOTORS:
    from rustypot import Xl330PyController

    c = Xl330PyController(args.port, baudrate=1000000, timeout=0.01)
    ID = 1

    if args.motor in _XL330_REQUIRED_FIRMWARE:
        firmware = c.read_firmware_version(ID)[0]
        if firmware != _XL330_REQUIRED_FIRMWARE[args.motor]:
            raise RuntimeError(
                f"{args.motor} expects firmware_version "
                f"{_XL330_REQUIRED_FIRMWARE[args.motor]}, the servo reports {firmware}"
            )

    if args.motor in _XL330_ROBOT_REGISTERS:
        # pwm_slope and current_limit live in EEPROM, only writable with torque off
        c.write_torque_enable(ID, False)
        if args.motor in _XL330_REQUIRED_FIRMWARE:
            c.write_raw_data(ID, _IMPACT_GUARD_ENABLE_ADDR, [0])
            guard = c.read_raw_data(ID, _IMPACT_GUARD_ENABLE_ADDR, 1)
            if list(guard) != [0]:
                raise RuntimeError(f"Impact Guard Enable reads {guard}, expected [0]")
        for register, value in _XL330_ROBOT_REGISTERS[args.motor].items():
            getattr(c, f"write_{register}")(ID, value)
            read = getattr(c, f"read_{register}")(ID)[0]
            if read != value:
                raise RuntimeError(f"{register}: wrote {value}, read back {read}")

    start = time.time()
    while time.time() - start < 1.0:
        goal_position, torque_enable = trajectory(0)
        if torque_enable:
            c.write_goal_position(ID, goal_position)
        c.write_torque_enable(ID, torque_enable)
        c.write_position_p_gain(ID, args.kp)
else:
    dxl = DynamixelActuatorV1(args.port)

    start = time.time()
    while time.time() - start < 1.0:
        goal_position, torque_enable = trajectory(0)
        if torque_enable:
            dxl.set_goal_position(goal_position)
        dxl.set_torque(torque_enable)
        dxl.set_p_gain(args.kp)

# ── Recording loop ───────────────────────────────────────────────────────────
start = time.time()
data = {
    "mass": args.mass,
    "arm-mass": args.arm_mass,
    "length": args.length,
    "kp": args.kp,
    "vin": args.vin,
    "motor": args.motor,
    "trajectory": args.trajectory,
    "entries": [],
}

while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, new_torque_enable = trajectory(t)
    if new_torque_enable != torque_enable:
        if args.motor in _XL330_MOTORS:
            c.write_torque_enable(ID, new_torque_enable)
        else:
            dxl.set_torque(new_torque_enable)
        torque_enable = new_torque_enable
        time.sleep(0.001)
    if torque_enable:
        if args.motor in _XL330_MOTORS:
            c.write_goal_position(ID, goal_position)
        else:
            dxl.set_goal_position(goal_position)
        time.sleep(0.001)

    t0 = time.time() - start

    if args.motor in _XL330_MOTORS:
        entry = read_xl330_state(c, ID)
        # Supply voltage and temperature change slowly: refreshed every few samples
        if len(data["entries"]) % _XL330_SLOW_READ_PERIOD == 0:
            slow = {
                "input_volts": c.read_present_input_voltage(ID)[0] / 10.0,
                "temp": c.read_present_temperature(ID)[0],
            }
        entry.update(slow)
    else:
        entry = dxl.read_data()

    t1 = time.time() - start

    entry["timestamp"] = (t0 + t1) / 2.0
    entry["goal_position"] = goal_position
    entry["torque_enable"] = torque_enable
    data["entries"].append(entry)

# ── Return to zero ───────────────────────────────────────────────────────────
goal_position = data["entries"][-1]["position"]
return_dt = 0.01
max_variation = return_dt * 1.0
while abs(goal_position) > 0:
    if goal_position > 0:
        goal_position = max(0, goal_position - max_variation)
    else:
        goal_position = min(0, goal_position + max_variation)
    if args.motor in _XL330_MOTORS:
        c.write_goal_position(ID, goal_position)
    else:
        dxl.set_goal_position(goal_position)
    time.sleep(return_dt)

if args.motor in _XL330_MOTORS:
    c.write_torque_enable(ID, False)
else:
    dxl.set_torque(False)

# ── Save ─────────────────────────────────────────────────────────────────────
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))
